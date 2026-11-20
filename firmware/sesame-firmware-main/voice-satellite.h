#pragma once

// Wyoming protocol voice satellite for Home Assistant.
//
// Hardware: INMP441 I2S microphone + MAX98357A I2S amplifier sharing one
// full-duplex I2S bus (common BCLK/WS). Home Assistant connects to TCP port
// WYOMING_PORT (advertised over mDNS as _wyoming._tcp), the robot streams its
// microphone continuously, and wake word / speech-to-text / text-to-speech all
// run in Home Assistant (openWakeWord, Whisper, Piper).
//
// Threading: everything here runs in two FreeRTOS tasks on core 0 so the
// blocking animation code on core 1 never starves the audio stream.
//   - net task:     owns the TCP socket, reads the mic, parses Wyoming events
//   - speaker task: drains the playback ring buffer into I2S
// The main loop only talks to this module through the small API at the bottom
// (atomics), and applies face changes itself because the OLED is not thread-safe.

#include <Arduino.h>
#include <WiFi.h>
#include <ESP_I2S.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <atomic>

#ifndef VOICE_I2S_BCLK
#define VOICE_I2S_BCLK 9    // INMP441 SCK + MAX98357A BCLK
#endif
#ifndef VOICE_I2S_WS
#define VOICE_I2S_WS 8      // INMP441 WS + MAX98357A LRC
#endif
#ifndef VOICE_I2S_DIN
#define VOICE_I2S_DIN 4     // INMP441 SD (mic data into the ESP32)
#endif
#ifndef VOICE_I2S_DOUT
#define VOICE_I2S_DOUT 5    // MAX98357A DIN (speaker data out of the ESP32)
#endif
#ifndef VOICE_AMP_SD_PIN
#define VOICE_AMP_SD_PIN -1 // Optional MAX98357A SD pin: amp is shut down while silent
#endif
#ifndef VOICE_MIC_RIGHT_CHANNEL
#define VOICE_MIC_RIGHT_CHANNEL 0  // 0 = INMP441 L/R tied to GND, 1 = tied to 3V3
#endif
#ifndef VOICE_SAMPLE_RATE
#define VOICE_SAMPLE_RATE 22050    // Matches Home Assistant's TTS rate, so speech plays without resampling
#endif
#ifndef VOICE_MIC_GAIN_SHIFT
#define VOICE_MIC_GAIN_SHIFT 12    // 32-bit sample >> 16 is unity gain; each step lower doubles the gain
#endif
#ifndef VOICE_SPEAKER_VOLUME
#define VOICE_SPEAKER_VOLUME 60    // Percent
#endif
#ifndef VOICE_MIC_MUTE_TAIL_MS
#define VOICE_MIC_MUTE_TAIL_MS 400 // Keep the mic muted this long after servos/speaker stop
#endif
#ifndef VOICE_PLAYBACK_BUFFER_BYTES
#define VOICE_PLAYBACK_BUFFER_BYTES (256 * 1024)  // PSRAM; ~6 s of speech
#endif
#ifndef VOICE_FALLBACK_BUFFER_BYTES
#define VOICE_FALLBACK_BUFFER_BYTES (32 * 1024)   // Internal RAM when PSRAM is unavailable
#endif
#ifndef WYOMING_PORT
#define WYOMING_PORT 10700
#endif
#ifndef VOICE_SATELLITE_NAME
#define VOICE_SATELLITE_NAME "Sesame Robot"
#endif

enum VoiceFaceEvent : uint8_t {
  VOICE_FACE_NONE = 0,
  VOICE_FACE_LISTENING,
  VOICE_FACE_THINKING,
  VOICE_FACE_TALKING,
  VOICE_FACE_DONE,
  VOICE_FACE_ERROR
};

namespace voice_sat {

// Single-producer / single-consumer byte ring. Capacity must be a power of two
// so the free-running head/tail counters stay correct when they wrap.
class SpscRing {
 public:
  bool begin(size_t want) {
    buf_ = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf_ == nullptr) {
      want = VOICE_FALLBACK_BUFFER_BYTES;
      buf_ = (uint8_t*)malloc(want);
    }
    mask_ = buf_ ? want - 1 : 0;
    cap_ = buf_ ? want : 0;
    return buf_ != nullptr;
  }
  size_t capacity() const { return cap_; }
  size_t used() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  size_t freeSpace() const { return cap_ - used(); }

  // Producer side.
  size_t write(const uint8_t* data, size_t n) {
    size_t h = head_.load(std::memory_order_relaxed);
    size_t space = cap_ - (h - tail_.load(std::memory_order_acquire));
    if (n > space) n = space;
    size_t pos = h & mask_;
    size_t first = min(n, cap_ - pos);
    memcpy(buf_ + pos, data, first);
    memcpy(buf_, data + first, n - first);
    head_.store(h + n, std::memory_order_release);
    return n;
  }

  // Consumer side.
  size_t read(uint8_t* out, size_t n) {
    size_t t = tail_.load(std::memory_order_relaxed);
    size_t avail = head_.load(std::memory_order_acquire) - t;
    if (n > avail) n = avail;
    size_t pos = t & mask_;
    size_t first = min(n, cap_ - pos);
    memcpy(out, buf_ + pos, first);
    memcpy(out + first, buf_, n - first);
    tail_.store(t + n, std::memory_order_release);
    return n;
  }
  void clearFromConsumer() {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
  }

 private:
  uint8_t* buf_ = nullptr;
  size_t cap_ = 0;
  size_t mask_ = 0;
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
};

static_assert((VOICE_PLAYBACK_BUFFER_BYTES & (VOICE_PLAYBACK_BUFFER_BYTES - 1)) == 0,
              "VOICE_PLAYBACK_BUFFER_BYTES must be a power of two");
static_assert((VOICE_FALLBACK_BUFFER_BYTES & (VOICE_FALLBACK_BUFFER_BYTES - 1)) == 0,
              "VOICE_FALLBACK_BUFFER_BYTES must be a power of two");

static const size_t MIC_FRAMES = 512;            // ~23 ms per audio-chunk at 22050 Hz
static const size_t MAX_PAYLOAD = 8192;          // Home Assistant sends 2048-byte chunks
static const size_t MAX_DATA = 4096;             // Event data (e.g. long synthesize text)
static const size_t MAX_HEADER = 1024;
static const size_t RING_HEADROOM = 3 * MAX_PAYLOAD;  // Worst case growth of one payload after resampling

static I2SClass i2s;
static WiFiServer tcpServer(WYOMING_PORT);
static WiFiClient client;
static SpscRing playRing;
static bool started = false;

static std::atomic<bool> clientConnected{false};
static std::atomic<bool> satelliteRunning{false};
static std::atomic<bool> streaming{false};
static std::atomic<bool> playbackActive{false};
static std::atomic<bool> playbackEnded{false};
static std::atomic<bool> playedPending{false};
static std::atomic<bool> flushRequested{false};
static std::atomic<bool> motionActive{false};
static std::atomic<uint32_t> lastMotionMs{0};
static std::atomic<uint32_t> lastPlaybackMs{0};
static std::atomic<uint8_t> faceEvent{VOICE_FACE_NONE};

// Net task only.
static uint32_t pipelineRestartAtMs = 0;
static bool inConversation = false;

struct PlaybackFormat {
  uint32_t rate = VOICE_SAMPLE_RATE;
  uint8_t width = 2;
  uint8_t channels = 1;
};
static PlaybackFormat playFmt;

// Linear resampler from the TTS rate to VOICE_SAMPLE_RATE (16.16 fixed point).
struct Resampler {
  uint32_t step = 65536;
  uint32_t frac = 0;
  int16_t prev = 0;
  bool primed = false;
};
static Resampler rs;

enum ParseState { P_HEADER, P_DATA, P_PAYLOAD };
static ParseState pstate = P_HEADER;
static char headerBuf[MAX_HEADER];
static size_t headerLen = 0;
static char dataBuf[MAX_DATA + 1];
static size_t dataLen = 0, dataNeed = 0;
alignas(4) static uint8_t payloadBuf[MAX_PAYLOAD];
static size_t payloadLen = 0, payloadNeed = 0;
static JsonDocument eventDoc;

static void postFace(VoiceFaceEvent ev) { faceEvent.store(ev); }

static void setAmp(bool on) {
  if (VOICE_AMP_SD_PIN >= 0) digitalWrite(VOICE_AMP_SD_PIN, on ? HIGH : LOW);
}

static bool micGated() {
  uint32_t now = millis();
  if (motionActive.load() || playbackActive.load()) return true;
  if (now - lastMotionMs.load() < VOICE_MIC_MUTE_TAIL_MS) return true;
  if (now - lastPlaybackMs.load() < VOICE_MIC_MUTE_TAIL_MS) return true;
  return false;
}

// ---------------------------------------------------------------- sending ---

static bool writeAll(const uint8_t* data, size_t len) {
  return client.write(data, len) == len;
}

static bool sendEvent(JsonDocument& ev) {
  String line;
  serializeJson(ev, line);
  line += '\n';
  return writeAll((const uint8_t*)line.c_str(), line.length());
}

static bool sendSimple(const char* type) {
  JsonDocument ev;
  ev["type"] = type;
  return sendEvent(ev);
}

static bool sendAudioChunk(const int16_t* pcm, size_t samples) {
  static uint8_t tx[128 + MIC_FRAMES * 2];
  int n = snprintf((char*)tx, 128,
                   "{\"type\":\"audio-chunk\",\"data\":{\"rate\":%u,\"width\":2,\"channels\":1},"
                   "\"payload_length\":%u}\n",
                   (unsigned)VOICE_SAMPLE_RATE, (unsigned)(samples * 2));
  memcpy(tx + n, pcm, samples * 2);
  return writeAll(tx, n + samples * 2);
}

static void addArtifact(JsonObject obj, const char* name, const char* description) {
  obj["name"] = name;
  obj["attribution"]["name"] = "Sesame Robot";
  obj["attribution"]["url"] = "https://github.com/dorianborian/sesame-robot";
  obj["installed"] = true;
  obj["description"] = description;
  obj["version"] = "1.0.0";
}

static bool sendInfo() {
  JsonDocument ev;
  ev["type"] = "info";
  JsonObject data = ev["data"].to<JsonObject>();

  JsonObject sat = data["satellite"].to<JsonObject>();
  addArtifact(sat, VOICE_SATELLITE_NAME, "Sesame robot voice satellite");
  sat["has_vad"] = false;
  sat["active_wake_words"].to<JsonArray>();
  sat["max_active_wake_words"] = 0;
  sat["supports_trigger"] = false;

  JsonObject mic = data["mic"].to<JsonArray>().add<JsonObject>();
  addArtifact(mic, "inmp441", "INMP441 I2S microphone");
  mic["mic_format"]["rate"] = VOICE_SAMPLE_RATE;
  mic["mic_format"]["width"] = 2;
  mic["mic_format"]["channels"] = 1;

  JsonObject snd = data["snd"].to<JsonArray>().add<JsonObject>();
  addArtifact(snd, "max98357a", "MAX98357A I2S speaker amplifier");
  snd["snd_format"]["rate"] = VOICE_SAMPLE_RATE;
  snd["snd_format"]["width"] = 2;
  snd["snd_format"]["channels"] = 1;

  return sendEvent(ev);
}

// Ask Home Assistant to run wake word -> STT -> intent -> TTS on our stream,
// restarting automatically after every successful run.
static bool startPipeline() {
  JsonDocument ev;
  ev["type"] = "run-pipeline";
  JsonObject data = ev["data"].to<JsonObject>();
  data["start_stage"] = "wake";
  data["end_stage"] = "tts";
  data["restart_on_end"] = true;
  streaming.store(true);
  return sendEvent(ev);
}

// --------------------------------------------------------------- playback ---

// Wait (bounded) for the speaker task to drop any queued audio.
static void flushPlayback() {
  flushRequested.store(true);
  for (int i = 0; i < 50 && flushRequested.load(); i++) vTaskDelay(pdMS_TO_TICKS(2));
}

static void beginPlayback(uint32_t rate, uint8_t width, uint8_t channels) {
  flushPlayback();
  playFmt.rate = rate ? rate : VOICE_SAMPLE_RATE;
  playFmt.width = width;
  playFmt.channels = channels ? channels : 1;
  rs = Resampler();
  rs.step = (uint32_t)(((uint64_t)playFmt.rate << 16) / VOICE_SAMPLE_RATE);
  playbackEnded.store(false);
  playbackActive.store(true);
  postFace(VOICE_FACE_TALKING);
}

static void pushPlayback(const uint8_t* payload, size_t len) {
  if (playFmt.width != 2) return;  // Home Assistant always sends 16-bit PCM
  const int16_t* in = (const int16_t*)payload;
  size_t frames = len / (2 * playFmt.channels);
  static int16_t out[512];
  size_t outN = 0;

  auto emit = [&](int16_t s) {
    out[outN++] = s;
    if (outN == sizeof(out) / sizeof(out[0])) {
      playRing.write((const uint8_t*)out, outN * 2);
      outN = 0;
    }
  };

  for (size_t f = 0; f < frames; f++) {
    int32_t s = in[f * playFmt.channels];
    if (playFmt.channels > 1) s = (s + in[f * playFmt.channels + 1]) / 2;

    if (rs.step == 65536) {
      emit((int16_t)s);
      continue;
    }
    if (!rs.primed) {
      rs.prev = (int16_t)s;
      rs.primed = true;
    }
    while (rs.frac < 65536) {
      int32_t v = rs.prev + (int32_t)(((int64_t)(s - rs.prev) * rs.frac) >> 16);
      emit((int16_t)v);
      rs.frac += rs.step;
    }
    rs.frac -= 65536;
    rs.prev = (int16_t)s;
  }
  if (outN) playRing.write((const uint8_t*)out, outN * 2);
}

static void speakerTask(void*) {
  static int16_t mono[256];
  static int32_t stereo[512];
  for (;;) {
    if (flushRequested.load()) {
      playRing.clearFromConsumer();
      flushRequested.store(false);
    }

    size_t n = playRing.read((uint8_t*)mono, sizeof(mono)) / 2;
    if (n == 0) {
      if (playbackActive.load() && playbackEnded.load()) {
        lastPlaybackMs.store(millis());
        playbackEnded.store(false);
        playbackActive.store(false);
        playedPending.store(true);
        setAmp(false);
      }
      // The I2S driver auto-clears its DMA buffers, so the amp outputs silence.
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    setAmp(true);
    for (size_t i = 0; i < n; i++) {
      int32_t s = ((int32_t)mono[i] * VOICE_SPEAKER_VOLUME) / 100;
      stereo[2 * i] = stereo[2 * i + 1] = s << 16;
    }
    i2s.write((const uint8_t*)stereo, n * 8);  // Paced by the DMA
    lastPlaybackMs.store(millis());
  }
}

// ---------------------------------------------------------------- parsing ---

static void resetParser() {
  pstate = P_HEADER;
  headerLen = dataLen = dataNeed = payloadLen = payloadNeed = 0;
}

static void dispatch(const uint8_t* payload, size_t payloadSize) {
  const char* type = eventDoc["type"] | "";
  JsonObjectConst data = eventDoc["data"].as<JsonObjectConst>();

  if (strcmp(type, "describe") == 0) {
    sendInfo();
  } else if (strcmp(type, "run-satellite") == 0) {
    satelliteRunning.store(true);
    pipelineRestartAtMs = 0;
    startPipeline();
  } else if (strcmp(type, "pause-satellite") == 0) {
    satelliteRunning.store(false);
    streaming.store(false);
    pipelineRestartAtMs = 0;
  } else if (strcmp(type, "ping") == 0) {
    JsonDocument ev;
    ev["type"] = "pong";
    ev["data"]["text"] = data["text"];
    sendEvent(ev);
  } else if (strcmp(type, "detection") == 0 || strcmp(type, "transcribe") == 0) {
    // "transcribe" also arrives without a wake word (e.g. pipeline started at STT).
    if (strcmp(type, "detection") == 0 || !inConversation) postFace(VOICE_FACE_LISTENING);
    inConversation = true;
  } else if (strcmp(type, "voice-stopped") == 0 || strcmp(type, "transcript") == 0) {
    postFace(VOICE_FACE_THINKING);
  } else if (strcmp(type, "audio-start") == 0) {
    beginPlayback(data["rate"] | VOICE_SAMPLE_RATE, data["width"] | 2, data["channels"] | 1);
    inConversation = true;
  } else if (strcmp(type, "audio-chunk") == 0) {
    if (playbackActive.load() && !playbackEnded.load() && payload) pushPlayback(payload, payloadSize);
  } else if (strcmp(type, "audio-stop") == 0) {
    if (playbackActive.load()) {
      playbackEnded.store(true);
    } else {
      playedPending.store(true);
    }
  } else if (strcmp(type, "error") == 0) {
    Serial.print(F("Wyoming error: "));
    Serial.println(data["text"] | "");
    inConversation = false;
    postFace(VOICE_FACE_ERROR);
    // Home Assistant does not restart a pipeline that ended in an error.
    if (satelliteRunning.load()) pipelineRestartAtMs = millis() + 2000;
  }
  // timer-*, synthesize, voice-started and unknown events are ignored.
}

static bool onHeader() {
  headerBuf[headerLen] = '\0';
  eventDoc.clear();
  if (deserializeJson(eventDoc, headerBuf, headerLen)) return false;
  dataNeed = eventDoc["data_length"] | 0;
  payloadNeed = eventDoc["payload_length"] | 0;
  dataLen = payloadLen = 0;
  if (dataNeed > 0) {
    pstate = P_DATA;
  } else if (payloadNeed > 0) {
    pstate = P_PAYLOAD;
  } else {
    dispatch(nullptr, 0);
    pstate = P_HEADER;
  }
  return true;
}

static void onData() {
  if (dataLen <= MAX_DATA) {
    // Additional data is merged over the header's "data" object.
    JsonDocument extra;
    if (!deserializeJson(extra, dataBuf, dataLen)) {
      JsonObject target = eventDoc["data"].is<JsonObject>() ? eventDoc["data"].as<JsonObject>()
                                                             : eventDoc["data"].to<JsonObject>();
      for (JsonPair kv : extra.as<JsonObject>()) target[kv.key()] = kv.value();
    }
  }
  if (payloadNeed > 0) {
    pstate = P_PAYLOAD;
  } else {
    dispatch(nullptr, 0);
    pstate = P_HEADER;
  }
}

static bool feed(const uint8_t* p, size_t n) {
  size_t i = 0;
  while (i < n) {
    if (pstate == P_HEADER) {
      uint8_t c = p[i++];
      if (c == '\n') {
        if (!onHeader()) return false;
        headerLen = 0;
      } else {
        if (headerLen >= MAX_HEADER - 1) return false;
        headerBuf[headerLen++] = (char)c;
      }
    } else if (pstate == P_DATA) {
      size_t take = min(n - i, dataNeed - dataLen);
      if (dataLen + take <= MAX_DATA) memcpy(dataBuf + dataLen, p + i, take);
      dataLen += take;
      i += take;
      if (dataLen == dataNeed) onData();
    } else {
      size_t take = min(n - i, payloadNeed - payloadLen);
      if (payloadLen + take <= MAX_PAYLOAD) memcpy(payloadBuf + payloadLen, p + i, take);
      payloadLen += take;
      i += take;
      if (payloadLen == payloadNeed) {
        // Oversized payloads are skipped rather than truncated.
        if (payloadNeed <= MAX_PAYLOAD) dispatch(payloadBuf, payloadLen);
        else dispatch(nullptr, 0);
        pstate = P_HEADER;
      }
    }
  }
  return true;
}

// ------------------------------------------------------------- connection ---

static void onDisconnect() {
  client.stop();
  clientConnected.store(false);
  satelliteRunning.store(false);
  streaming.store(false);
  pipelineRestartAtMs = 0;
  resetParser();
  flushPlayback();
  playbackActive.store(false);
  playbackEnded.store(false);
  setAmp(false);
  playedPending.store(false);
  if (inConversation) postFace(VOICE_FACE_DONE);
  inConversation = false;
  Serial.println(F("Wyoming client disconnected"));
}

static void pumpIncoming() {
  static uint8_t buf[1024];
  for (int iter = 0; iter < 8; iter++) {
    // Back-pressure: leave TTS audio in the socket until the speaker catches up,
    // but keep the loop (and the mic stream that keeps Home Assistant's ping
    // timeout happy) running.
    if (playRing.freeSpace() < RING_HEADROOM) break;
    int avail = client.available();
    if (avail <= 0) break;
    int n = client.read(buf, min((size_t)avail, sizeof(buf)));
    if (n <= 0) break;
    if (!feed(buf, (size_t)n)) {
      Serial.println(F("Wyoming protocol error"));
      onDisconnect();
      return;
    }
  }
}

static void netTask(void*) {
  static int32_t raw[MIC_FRAMES * 2];
  static int16_t pcm[MIC_FRAMES];

  for (;;) {
    if (tcpServer.hasClient()) {
      WiFiClient incoming = tcpServer.accept();
      if (clientConnected.load()) onDisconnect();  // Newest connection wins (HA reconnects)
      client = incoming;
      client.setNoDelay(true);
      resetParser();
      clientConnected.store(true);
      Serial.println(F("Wyoming client connected"));
    }
    if (clientConnected.load() && !client.connected()) onDisconnect();

    // Blocks for one chunk of audio; this paces the whole task.
    size_t got = i2s.readBytes((char*)raw, sizeof(raw));
    size_t frames = got / 8;
    if (frames == 0) {
      vTaskDelay(pdMS_TO_TICKS(20));
    } else {
      bool gated = micGated();
      for (size_t i = 0; i < frames; i++) {
        int32_t s = gated ? 0 : (raw[2 * i + VOICE_MIC_RIGHT_CHANNEL] >> VOICE_MIC_GAIN_SHIFT);
        pcm[i] = (int16_t)constrain(s, -32768, 32767);
      }
      if (clientConnected.load() && streaming.load() && !sendAudioChunk(pcm, frames)) onDisconnect();
    }

    if (clientConnected.load()) pumpIncoming();

    if (clientConnected.load() && playedPending.exchange(false)) {
      sendSimple("played");
      inConversation = false;
      postFace(VOICE_FACE_DONE);
    }

    if (pipelineRestartAtMs != 0 && (int32_t)(millis() - pipelineRestartAtMs) >= 0) {
      pipelineRestartAtMs = 0;
      if (clientConnected.load() && satelliteRunning.load()) startPipeline();
    }
  }
}

}  // namespace voice_sat

// ------------------------------------------------------------- public API ---

inline bool voiceSetup() {
  using namespace voice_sat;
  if (started) return true;

  if (!playRing.begin(VOICE_PLAYBACK_BUFFER_BYTES)) {
    Serial.println(F("Voice: no memory for playback buffer"));
    return false;
  }
  Serial.printf("Voice: playback buffer %u bytes (%s)\n", (unsigned)playRing.capacity(),
                playRing.capacity() == VOICE_PLAYBACK_BUFFER_BYTES ? "PSRAM" : "internal RAM");

  if (VOICE_AMP_SD_PIN >= 0) {
    pinMode(VOICE_AMP_SD_PIN, OUTPUT);
    setAmp(false);
  }

  i2s.setPins(VOICE_I2S_BCLK, VOICE_I2S_WS, VOICE_I2S_DOUT, VOICE_I2S_DIN);
  if (!i2s.begin(I2S_MODE_STD, VOICE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println(F("Voice: I2S init failed"));
    return false;
  }

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  // Core 0, below the WiFi stack's priority; the Arduino loop runs on core 1.
  xTaskCreatePinnedToCore(netTask, "wyoming", 8192, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(speakerTask, "voice_spk", 4096, nullptr, 4, nullptr, 0);
  started = true;
  Serial.printf("Voice satellite listening on port %d\n", WYOMING_PORT);
  return true;
}

// Mute the mic while servos move (motor noise would trigger wake words).
inline void voiceSetMotionActive(bool active) {
  voice_sat::motionActive.store(active);
  if (!active) voice_sat::lastMotionMs.store(millis());
}

inline VoiceFaceEvent voiceTakeFaceEvent() {
  return (VoiceFaceEvent)voice_sat::faceEvent.exchange(VOICE_FACE_NONE);
}

inline const char* voiceStatus() {
  using namespace voice_sat;
  if (!started) return "offline";
  if (!clientConnected.load()) return "waiting";
  if (playbackActive.load()) return "speaking";
  if (streaming.load()) return "listening";
  return "connected";
}
