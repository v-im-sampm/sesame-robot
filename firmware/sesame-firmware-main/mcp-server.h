#pragma once

// Model Context Protocol (MCP) server for Home Assistant and other AI clients.
//
// Implements the stateless subset of the MCP "Streamable HTTP" transport:
// every JSON-RPC request is a POST to /mcp and gets a single application/json
// response. No sessions, no server-initiated messages, no SSE stream (GET/DELETE
// return 405, which MCP clients treat as "not supported").
//
// Tools only change robot state (like /api/command) and return immediately, so
// they are safe to run from inside the blocking animation loops.

#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#ifndef MCP_MAX_MOVE_SECONDS
#define MCP_MAX_MOVE_SECONDS 10
#endif
#ifndef MCP_DEFAULT_MOVE_SECONDS
#define MCP_DEFAULT_MOVE_SECONDS 3
#endif

#define MCP_STR_(x) #x
#define MCP_STR(x) MCP_STR_(x)

extern WebServer server;
extern String currentCommand;
extern void startCommand(const String& cmd, unsigned long durationMs);
extern void stopCommand();
extern bool firmwareUpdateInProgress();
extern void setFace(const String& faceName);
extern void recordInput();
extern String buildStatusJson();

static const char* const MCP_DIRECTIONS[] = {"forward", "backward", "left", "right"};
static const char* const MCP_ACTIONS[] = {
  "wave", "dance", "bow", "point", "pushup", "shake", "shrug", "cute",
  "freaky", "worm", "swim", "crab", "dead", "stand", "rest"
};
static const char* const MCP_FACES[] = {
  "happy", "sad", "angry", "surprised", "sleepy", "love", "excited",
  "confused", "thinking", "default"
};

// Descriptions are short and enum-constrained so small local LLMs pick the
// right tool and arguments.
static const char MCP_TOOLS_JSON[] PROGMEM =
  R"JSON([
{"name":"move",
 "description":"Make the Sesame robot walk forward or backward, or turn left or right in place, for a number of seconds. It stops and stands up afterwards.",
 "inputSchema":{"type":"object","properties":{
   "direction":{"type":"string","enum":["forward","backward","left","right"]},
   "seconds":{"type":"integer","minimum":1,"maximum":)JSON" MCP_STR(MCP_MAX_MOVE_SECONDS) R"JSON(,"default":)JSON" MCP_STR(MCP_DEFAULT_MOVE_SECONDS) R"JSON(,"description":"How long to keep moving"}},
  "required":["direction"]}},
{"name":"perform",
 "description":"Make the Sesame robot perform a gesture or trick once. wave = say hello, bow = greet politely, dance = celebrate, dead = play dead, stand = stand up, rest = sit down and relax, point = point forward, pushup = do pushups, shake = shake body, shrug = shrug, cute = act cute.",
 "inputSchema":{"type":"object","properties":{
   "action":{"type":"string","enum":["wave","dance","bow","point","pushup","shake","shrug","cute","freaky","worm","swim","crab","dead","stand","rest"]}},
  "required":["action"]}},
{"name":"set_face",
 "description":"Change the expression on the Sesame robot's face display without moving.",
 "inputSchema":{"type":"object","properties":{
   "face":{"type":"string","enum":["happy","sad","angry","surprised","sleepy","love","excited","confused","thinking","default"]}},
  "required":["face"]}},
{"name":"stop",
 "description":"Stop whatever the Sesame robot is doing and stand still.",
 "inputSchema":{"type":"object","properties":{}}},
{"name":"get_status",
 "description":"Get what the Sesame robot is currently doing, its face, and its network status.",
 "inputSchema":{"type":"object","properties":{}}}
])JSON";

static const char MCP_INSTRUCTIONS[] PROGMEM =
  "Controls a small four-legged Sesame robot. Use move for walking or turning, "
  "perform for gestures and tricks, set_face for expressions, and stop to halt.";

template <size_t N>
static bool mcpInList(const char* value, const char* const (&list)[N]) {
  if (value == nullptr) return false;
  for (size_t i = 0; i < N; i++) {
    if (strcmp(value, list[i]) == 0) return true;
  }
  return false;
}

template <size_t N>
static String mcpJoin(const char* const (&list)[N]) {
  String out;
  for (size_t i = 0; i < N; i++) {
    if (i) out += ", ";
    out += list[i];
  }
  return out;
}

static void mcpSend(JsonDocument& res) {
  String out;
  serializeJson(res, out);
  server.send(200, "application/json", out);
}

static void mcpSendError(JsonVariantConst id, int code, const char* message) {
  JsonDocument res;
  res["jsonrpc"] = "2.0";
  res["id"] = id;  // null when the request id is unknown (e.g. parse error)
  JsonObject err = res["error"].to<JsonObject>();
  err["code"] = code;
  err["message"] = message;
  mcpSend(res);
}

static void mcpSendToolResult(JsonVariantConst id, const String& text, bool isError) {
  JsonDocument res;
  res["jsonrpc"] = "2.0";
  res["id"] = id;
  JsonObject result = res["result"].to<JsonObject>();
  JsonObject item = result["content"].to<JsonArray>().add<JsonObject>();
  item["type"] = "text";
  item["text"] = text;
  result["isError"] = isError;
  mcpSend(res);
}

static void mcpCallTool(JsonVariantConst id, const char* name, JsonObjectConst args) {
  if (firmwareUpdateInProgress()) {
    mcpSendToolResult(id, "The robot is updating its firmware and cannot move right now.", true);
    return;
  }

  if (strcmp(name, "move") == 0) {
    const char* direction = args["direction"];
    if (!mcpInList(direction, MCP_DIRECTIONS)) {
      mcpSendToolResult(id, "Invalid direction. Use one of: " + mcpJoin(MCP_DIRECTIONS), true);
      return;
    }
    // Small LLMs sometimes send numbers as strings ("3").
    JsonVariantConst secondsArg = args["seconds"];
    float seconds = secondsArg.isNull()               ? (float)MCP_DEFAULT_MOVE_SECONDS
                    : secondsArg.is<const char*>()    ? (float)atof(secondsArg.as<const char*>())
                                                      : secondsArg.as<float>();
    seconds = constrain(seconds, 1.0f, (float)MCP_MAX_MOVE_SECONDS);
    startCommand(direction, (unsigned long)(seconds * 1000.0f));
    String verb = (strcmp(direction, "left") == 0 || strcmp(direction, "right") == 0)
                    ? String("Turning ") + direction
                    : String("Walking ") + direction;
    mcpSendToolResult(id, verb + " for " + String(seconds, 0) + " seconds.", false);
    return;
  }

  if (strcmp(name, "perform") == 0) {
    const char* action = args["action"];
    if (!mcpInList(action, MCP_ACTIONS)) {
      mcpSendToolResult(id, "Invalid action. Use one of: " + mcpJoin(MCP_ACTIONS), true);
      return;
    }
    startCommand(action, 0);
    mcpSendToolResult(id, String("Performing ") + action + ".", false);
    return;
  }

  if (strcmp(name, "set_face") == 0) {
    const char* face = args["face"];
    if (!mcpInList(face, MCP_FACES)) {
      mcpSendToolResult(id, "Invalid face. Use one of: " + mcpJoin(MCP_FACES), true);
      return;
    }
    setFace(face);
    recordInput();
    mcpSendToolResult(id, String("Face set to ") + face + ".", false);
    return;
  }

  if (strcmp(name, "stop") == 0) {
    bool gesture = currentCommand.length() > 0 && !mcpInList(currentCommand.c_str(), MCP_DIRECTIONS);
    stopCommand();
    mcpSendToolResult(id, gesture ? "Stopping. The current gesture will finish first." : "Stopped.", false);
    return;
  }

  if (strcmp(name, "get_status") == 0) {
    mcpSendToolResult(id, buildStatusJson(), false);
    return;
  }

  mcpSendError(id, -32602, "Unknown tool");
}

// POST /mcp
void handleMcp() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }

  JsonDocument req;
  DeserializationError err = deserializeJson(req, server.arg("plain"));
  if (err) {
    mcpSendError(JsonVariantConst(), -32700, "Parse error");
    return;
  }
  if (!req.is<JsonObject>()) {
    mcpSendError(JsonVariantConst(), -32600, "Invalid request: batches are not supported");
    return;
  }

  JsonVariantConst id = req["id"];
  const char* method = req["method"];

  // Notifications and responses to (never sent) server requests: no body.
  if (method == nullptr || id.isNull()) {
    server.send(202);
    return;
  }

  if (strcmp(method, "initialize") == 0) {
    JsonDocument res;
    res["jsonrpc"] = "2.0";
    res["id"] = id;
    JsonObject result = res["result"].to<JsonObject>();
    // Echo the client's protocol version: this server only uses features that
    // are identical across published MCP revisions.
    const char* version = req["params"]["protocolVersion"] | "2025-03-26";
    result["protocolVersion"] = version;
    result["capabilities"]["tools"]["listChanged"] = false;
    result["serverInfo"]["name"] = "sesame-robot";
    result["serverInfo"]["version"] = "1.0.0";
    result["instructions"] = MCP_INSTRUCTIONS;
    mcpSend(res);
    return;
  }

  if (strcmp(method, "ping") == 0) {
    JsonDocument res;
    res["jsonrpc"] = "2.0";
    res["id"] = id;
    res["result"].to<JsonObject>();
    mcpSend(res);
    return;
  }

  if (strcmp(method, "tools/list") == 0) {
    JsonDocument res;
    res["jsonrpc"] = "2.0";
    res["id"] = id;
    res["result"]["tools"] = serialized(MCP_TOOLS_JSON);
    mcpSend(res);
    return;
  }

  if (strcmp(method, "tools/call") == 0) {
    const char* name = req["params"]["name"];
    if (name == nullptr) {
      mcpSendError(id, -32602, "Missing tool name");
      return;
    }
    mcpCallTool(id, name, req["params"]["arguments"].as<JsonObjectConst>());
    return;
  }

  mcpSendError(id, -32601, "Method not found");
}
