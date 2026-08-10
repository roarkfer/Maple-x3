#include <Arduino.h>
#include <FS.h>  // Include before SdFat/SDCardManager for compatibility.
#include <ArduinoJson.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <InputManager.h>
#include <SDCardManager.h>
#include <XteinkDetect.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// -----------------------------------------------------------------------------
// Maple X3 Receiver v0.2
// Protocol 1 / protocolRevision 1.2
//
// Maple is the structural authority. The X3 can only toggle actionable states
// and change exercise kg. Every local change is queued in changes.json until
// Maple acknowledges it during the next synchronization.
// -----------------------------------------------------------------------------

static constexpr const char* FW_VERSION = "0.2.0";
static constexpr int MAPLE_PROTOCOL = 1;
static constexpr const char* AP_SSID = "Maple-X3";
static constexpr const char* MDNS_HOST = "maple-x3";

static constexpr const char* MAPLE_DIR = "/.maple";
static constexpr const char* STATE_PATH = "/.maple/state.json";
static constexpr const char* STATE_TMP_PATH = "/.maple/state.tmp";
static constexpr const char* CHANGES_PATH = "/.maple/changes.json";

// XTEINK X3 display SPI pinout (same proven mapping as Maple X3 v0.1).
static constexpr int EPD_SCLK = 8;
static constexpr int EPD_MOSI = 10;
static constexpr int EPD_CS   = 21;
static constexpr int EPD_DC   = 4;
static constexpr int EPD_RST  = 5;
static constexpr int EPD_BUSY = 6;

EInkDisplay display(EPD_SCLK, EPD_MOSI, EPD_CS, EPD_DC, EPD_RST, EPD_BUSY);
InputManager input;
WebServer server(80);

JsonDocument stateDoc;
JsonDocument changesDoc;
bool storageReady = false;
bool stateLoaded = false;
bool reloadStatePending = false;
uint32_t currentRevision = 0;
uint32_t redrawCount = 0;

// -----------------------------------------------------------------------------
// Drawing helpers: portrait logical coordinates (528 x 792)
// -----------------------------------------------------------------------------

static uint16_t logicalWidth()  { return display.getDisplayHeight(); }
static uint16_t logicalHeight() { return display.getDisplayWidth(); }

static void pixelPhysical(int x, int y, bool black) {
  if (x < 0 || y < 0 || x >= display.getDisplayWidth() || y >= display.getDisplayHeight()) return;
  uint8_t* fb = display.getFrameBuffer();
  const uint16_t rowBytes = display.getDisplayWidthBytes();
  const size_t index = static_cast<size_t>(y) * rowBytes + (x >> 3);
  const uint8_t mask = 0x80 >> (x & 7);
  if (black) fb[index] &= ~mask;
  else       fb[index] |= mask;
}

static void pixel(int x, int y, bool black = true) {
  // Portrait -> physical landscape mapping validated by the v0.1 diagnostic.
  const int px = y;
  const int py = display.getDisplayHeight() - 1 - x;
  pixelPhysical(px, py, black);
}

static void fillRect(int x, int y, int w, int h, bool black = true) {
  if (w <= 0 || h <= 0) return;
  for (int yy = y; yy < y + h; ++yy) {
    for (int xx = x; xx < x + w; ++xx) pixel(xx, yy, black);
  }
}

static void drawRect(int x, int y, int w, int h, int thickness = 2, bool black = true) {
  fillRect(x, y, w, thickness, black);
  fillRect(x, y + h - thickness, w, thickness, black);
  fillRect(x, y, thickness, h, black);
  fillRect(x + w - thickness, y, thickness, h, black);
}

// A tiny radius-3 approximation matching Maple's slightly rounded check boxes.
static void drawRoundedRect(int x, int y, int w, int h, int thickness = 2, bool black = true) {
  fillRect(x + 3, y, w - 6, thickness, black);
  fillRect(x + 3, y + h - thickness, w - 6, thickness, black);
  fillRect(x, y + 3, thickness, h - 6, black);
  fillRect(x + w - thickness, y + 3, thickness, h - 6, black);
  fillRect(x + 1, y + 1, 3, thickness, black);
  fillRect(x + w - 4, y + 1, 3, thickness, black);
  fillRect(x + 1, y + h - 3, 3, thickness, black);
  fillRect(x + w - 4, y + h - 3, 3, thickness, black);
}

static uint8_t glyphRow(char c, int row) {
  // Compact 5x7 uppercase font. Unknown glyphs render as '?'.
  switch (c) {
    case 'A': { static const uint8_t r[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return r[row]; }
    case 'B': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return r[row]; }
    case 'C': { static const uint8_t r[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return r[row]; }
    case 'D': { static const uint8_t r[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return r[row]; }
    case 'E': { static const uint8_t r[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return r[row]; }
    case 'F': { static const uint8_t r[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return r[row]; }
    case 'G': { static const uint8_t r[7]={0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}; return r[row]; }
    case 'H': { static const uint8_t r[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return r[row]; }
    case 'I': { static const uint8_t r[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; return r[row]; }
    case 'J': { static const uint8_t r[7]={0x07,0x02,0x02,0x02,0x12,0x12,0x0C}; return r[row]; }
    case 'K': { static const uint8_t r[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return r[row]; }
    case 'L': { static const uint8_t r[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return r[row]; }
    case 'M': { static const uint8_t r[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return r[row]; }
    case 'N': { static const uint8_t r[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return r[row]; }
    case 'O': { static const uint8_t r[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return r[row]; }
    case 'P': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return r[row]; }
    case 'Q': { static const uint8_t r[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return r[row]; }
    case 'R': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return r[row]; }
    case 'S': { static const uint8_t r[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return r[row]; }
    case 'T': { static const uint8_t r[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return r[row]; }
    case 'U': { static const uint8_t r[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return r[row]; }
    case 'V': { static const uint8_t r[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return r[row]; }
    case 'W': { static const uint8_t r[7]={0x11,0x11,0x11,0x15,0x15,0x15,0x0A}; return r[row]; }
    case 'X': { static const uint8_t r[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return r[row]; }
    case 'Y': { static const uint8_t r[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return r[row]; }
    case 'Z': { static const uint8_t r[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return r[row]; }
    case '0': { static const uint8_t r[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return r[row]; }
    case '1': { static const uint8_t r[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return r[row]; }
    case '2': { static const uint8_t r[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return r[row]; }
    case '3': { static const uint8_t r[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return r[row]; }
    case '4': { static const uint8_t r[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return r[row]; }
    case '5': { static const uint8_t r[7]={0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}; return r[row]; }
    case '6': { static const uint8_t r[7]={0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}; return r[row]; }
    case '7': { static const uint8_t r[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return r[row]; }
    case '8': { static const uint8_t r[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return r[row]; }
    case '9': { static const uint8_t r[7]={0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}; return r[row]; }
    case '.': { static const uint8_t r[7]={0,0,0,0,0,0x06,0x06}; return r[row]; }
    case ',': { static const uint8_t r[7]={0,0,0,0,0x06,0x04,0x08}; return r[row]; }
    case ':': { static const uint8_t r[7]={0,0x06,0x06,0,0x06,0x06,0}; return r[row]; }
    case '-': { static const uint8_t r[7]={0,0,0,0x1F,0,0,0}; return r[row]; }
    case '/': { static const uint8_t r[7]={0x01,0x02,0x02,0x04,0x08,0x08,0x10}; return r[row]; }
    case '+': { static const uint8_t r[7]={0,0x04,0x04,0x1F,0x04,0x04,0}; return r[row]; }
    case '%': { static const uint8_t r[7]={0x11,0x12,0x02,0x04,0x08,0x09,0x11}; return r[row]; }
    case '?': { static const uint8_t r[7]={0x0E,0x11,0x01,0x02,0x04,0,0x04}; return r[row]; }
    case ' ': return 0;
    default:  { static const uint8_t r[7]={0x0E,0x11,0x01,0x02,0x04,0,0x04}; return r[row]; }
  }
}

static void drawChar(char c, int x, int y, int scale, bool black = true) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 32);
  for (int row = 0; row < 7; ++row) {
    const uint8_t bits = glyphRow(c, row);
    for (int col = 0; col < 5; ++col) {
      if (bits & (1 << (4 - col))) fillRect(x + col * scale, y + row * scale, scale, scale, black);
    }
  }
}

static String asciiUpper(const char* utf8) {
  String out;
  if (!utf8) return out;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(utf8);
  while (*p) {
    if (*p < 0x80) {
      char c = static_cast<char>(*p++);
      if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 32);
      if (c == '\n' || c == '\r' || (c >= 32 && c <= 126)) out += c;
      continue;
    }
    // Common Spanish UTF-8 accents: ÁÉÍÓÚÜÑ / áéíóúüñ.
    if (*p == 0xC3 && p[1]) {
      const uint8_t b = p[1];
      switch (b) {
        case 0x81: case 0xA1: out += 'A'; break;
        case 0x89: case 0xA9: out += 'E'; break;
        case 0x8D: case 0xAD: out += 'I'; break;
        case 0x93: case 0xB3: out += 'O'; break;
        case 0x9A: case 0xBA: case 0x9C: case 0xBC: out += 'U'; break;
        case 0x91: case 0xB1: out += 'N'; break;
        default: out += '?'; break;
      }
      p += 2;
    } else {
      out += '?';
      ++p;
      while ((*p & 0xC0) == 0x80) ++p;
    }
  }
  return out;
}

static int textWidth(const String& text, int scale) {
  return text.length() ? static_cast<int>(text.length()) * 6 * scale - scale : 0;
}

static void drawTextRaw(const String& text, int x, int y, int scale, bool black = true) {
  for (size_t i = 0; i < text.length(); ++i) {
    drawChar(text[i], x, y, scale, black);
    x += 6 * scale;
  }
}

static void drawText(const char* text, int x, int y, int scale, bool black = true) {
  drawTextRaw(asciiUpper(text), x, y, scale, black);
}

static void drawCentered(const char* text, int y, int scale, bool black = true) {
  const String s = asciiUpper(text);
  drawTextRaw(s, (static_cast<int>(logicalWidth()) - textWidth(s, scale)) / 2, y, scale, black);
}

static void drawTextClipped(const char* text, int x, int y, int scale, int maxWidth, bool black = true) {
  String s = asciiUpper(text);
  const int charW = 6 * scale;
  int maxChars = maxWidth / charW;
  if (maxChars < 1) return;
  if (static_cast<int>(s.length()) > maxChars) {
    if (maxChars >= 3) s = s.substring(0, maxChars - 2) + "..";
    else s = s.substring(0, maxChars);
  }
  drawTextRaw(s, x, y, scale, black);
}

static void drawCheck(int x, int y, bool checked, bool selected = false) {
  constexpr int S = 24;
  if (checked) {
    fillRect(x + 2, y + 2, S - 4, S - 4, true);
    // White check mark.
    for (int i = 0; i < 5; ++i) fillRect(x + 6 + i, y + 12 + i, 2, 2, false);
    for (int i = 0; i < 8; ++i) fillRect(x + 10 + i, y + 16 - i, 2, 2, false);
    drawRoundedRect(x, y, S, S, selected ? 3 : 2, true);
  } else {
    drawRoundedRect(x, y, S, S, selected ? 3 : 2, true);
  }
}

static void drawSelectionBar(int y) {
  fillRect(12, y + 2, 4, 29, true);
}

static void refreshDisplay(bool full = false) {
  ++redrawCount;
  if (full || redrawCount % 18 == 0) display.displayBuffer(EInkDisplay::FULL_REFRESH);
  else display.displayBuffer(EInkDisplay::FAST_REFRESH);
}

// -----------------------------------------------------------------------------
// Persistence
// -----------------------------------------------------------------------------

static bool writeJsonAtomic(const char* finalPath, const char* tempPath, JsonDocument& doc) {
  if (!storageReady) return false;
  if (SdMan.exists(tempPath)) SdMan.remove(tempPath);
  FsFile f = SdMan.open(tempPath, O_WRITE | O_CREAT | O_TRUNC);
  if (!f) return false;
  const size_t written = serializeJson(doc, f);
  f.flush();
  f.close();
  if (written == 0) return false;
  if (SdMan.exists(finalPath)) SdMan.remove(finalPath);
  return SdMan.rename(tempPath, finalPath);
}

static bool writeRawStateAtomic(const String& body) {
  if (!storageReady) return false;
  if (SdMan.exists(STATE_TMP_PATH)) SdMan.remove(STATE_TMP_PATH);
  FsFile f = SdMan.open(STATE_TMP_PATH, O_WRITE | O_CREAT | O_TRUNC);
  if (!f) return false;
  const size_t written = f.write(reinterpret_cast<const uint8_t*>(body.c_str()), body.length());
  f.flush();
  f.close();
  if (written != body.length()) return false;
  if (SdMan.exists(STATE_PATH)) SdMan.remove(STATE_PATH);
  return SdMan.rename(STATE_TMP_PATH, STATE_PATH);
}

static bool loadJsonFile(const char* path, JsonDocument& doc) {
  doc.clear();
  if (!storageReady || !SdMan.exists(path)) return false;
  FsFile f = SdMan.open(path, O_RDONLY);
  if (!f) return false;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  return !err;
}

static void initChangesDoc() {
  changesDoc.clear();
  changesDoc["protocol"] = MAPLE_PROTOCOL;
  changesDoc["protocolRevision"] = "1.2";
  changesDoc["device"] = "Maple X3";
  changesDoc["baseRevision"] = currentRevision;
  changesDoc["lastSeq"] = 0;
  changesDoc["changes"].to<JsonArray>();
}

static bool persistState() {
  return writeJsonAtomic(STATE_PATH, STATE_TMP_PATH, stateDoc);
}

static bool persistChanges() {
  // Reuse a separate temp name so an interrupted write can't damage state.tmp.
  static constexpr const char* TMP = "/.maple/changes.tmp";
  return writeJsonAtomic(CHANGES_PATH, TMP, changesDoc);
}

static bool loadState() {
  if (!loadJsonFile(STATE_PATH, stateDoc)) {
    stateLoaded = false;
    currentRevision = 0;
    return false;
  }
  if ((stateDoc["protocol"] | 0) != MAPLE_PROTOCOL) {
    stateDoc.clear();
    stateLoaded = false;
    currentRevision = 0;
    return false;
  }
  currentRevision = stateDoc["revision"] | 0;
  stateLoaded = true;
  return true;
}

static void loadChanges() {
  if (!loadJsonFile(CHANGES_PATH, changesDoc) || (changesDoc["protocol"] | 0) != MAPLE_PROTOCOL) {
    initChangesDoc();
    persistChanges();
  }
  if (!changesDoc["changes"].is<JsonArray>()) changesDoc["changes"].to<JsonArray>();
  if (!changesDoc["lastSeq"].is<uint32_t>()) changesDoc["lastSeq"] = 0;
}

static uint32_t pendingChangeCount() {
  return changesDoc["changes"].is<JsonArray>() ? changesDoc["changes"].as<JsonArray>().size() : 0;
}

static void acknowledgeChanges(uint32_t ackThrough, uint32_t newBaseRevision) {
  JsonArray arr = changesDoc["changes"].as<JsonArray>();
  for (int i = static_cast<int>(arr.size()) - 1; i >= 0; --i) {
    const uint32_t seq = arr[i]["seq"] | 0;
    if (seq <= ackThrough) arr.remove(i);
  }
  changesDoc["baseRevision"] = newBaseRevision;
  persistChanges();
}

static void queueChange(const char* kind, const char* id, bool value,
                        const char* date = nullptr, int setIndex = -1,
                        const char* projectId = nullptr, const char* folderId = nullptr) {
  JsonArray arr = changesDoc["changes"].as<JsonArray>();
  const uint32_t seq = (changesDoc["lastSeq"] | 0U) + 1;
  changesDoc["lastSeq"] = seq;
  JsonObject c = arr.add<JsonObject>();
  c["seq"] = seq;
  c["kind"] = kind;
  c["id"] = id;
  c["value"] = value;
  if (date) c["date"] = date;
  if (setIndex >= 0) c["setIndex"] = setIndex;
  if (projectId) c["projectId"] = projectId;
  if (folderId) c["folderId"] = folderId;
  persistChanges();
}

static void queueKgChange(const char* id, float value) {
  JsonArray arr = changesDoc["changes"].as<JsonArray>();
  const uint32_t seq = (changesDoc["lastSeq"] | 0U) + 1;
  changesDoc["lastSeq"] = seq;
  JsonObject c = arr.add<JsonObject>();
  c["seq"] = seq;
  c["kind"] = "exercise.kg";
  c["id"] = id;
  c["value"] = value;
  persistChanges();
}

static String currentDateKey() {
  const char* generated = stateDoc["generatedAt"] | "";
  String s(generated);
  if (s.length() >= 10) return s.substring(0, 10);
  return "1970-01-01";
}

// -----------------------------------------------------------------------------
// HTTP API
// -----------------------------------------------------------------------------

static void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type,X-Maple-Ack-Through");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Max-Age", "86400");
  server.sendHeader("Cache-Control", "no-store");
}

static void sendJson(int status, const String& json) {
  addCorsHeaders();
  server.send(status, "application/json; charset=utf-8", json);
}

static long extractTopLevelInt(const String& body, const char* key, long fallback = -1) {
  String needle = "\"";
  needle += key;
  needle += "\"";
  int p = body.indexOf(needle);
  if (p < 0) return fallback;
  p = body.indexOf(':', p + needle.length());
  if (p < 0) return fallback;
  ++p;
  while (p < static_cast<int>(body.length()) && (body[p] == ' ' || body[p] == '\t' || body[p] == '\n' || body[p] == '\r')) ++p;
  bool neg = false;
  if (p < static_cast<int>(body.length()) && body[p] == '-') { neg = true; ++p; }
  long value = 0;
  bool any = false;
  while (p < static_cast<int>(body.length()) && body[p] >= '0' && body[p] <= '9') {
    any = true;
    value = value * 10 + (body[p] - '0');
    ++p;
  }
  return any ? (neg ? -value : value) : fallback;
}

static void handleOptions() {
  addCorsHeaders();
  server.send(204, "text/plain", "");
}

static void handleStatus() {
  JsonDocument d;
  d["ok"] = true;
  d["device"] = "Maple X3";
  d["firmware"] = FW_VERSION;
  d["protocol"] = MAPLE_PROTOCOL;
  d["protocolRevision"] = "1.2";
  d["revision"] = currentRevision;
  d["pendingChanges"] = pendingChangeCount();
  d["lastSeq"] = changesDoc["lastSeq"] | 0U;
  d["stateLoaded"] = stateLoaded;
  d["storageReady"] = storageReady;
  String out;
  serializeJson(d, out);
  sendJson(200, out);
}

static void handleChanges() {
  String out;
  serializeJson(changesDoc, out);
  sendJson(200, out);
}

static void handlePostState() {
  if (!storageReady) {
    sendJson(507, "{\"ok\":false,\"error\":\"storage unavailable\"}");
    return;
  }
  const String body = server.arg("plain");
  if (body.length() < 10) {
    sendJson(400, "{\"ok\":false,\"error\":\"empty state\"}");
    return;
  }
  const long protocol = extractTopLevelInt(body, "protocol", -1);
  const long revision = extractTopLevelInt(body, "revision", -1);
  if (protocol != MAPLE_PROTOCOL || revision < 0) {
    sendJson(400, "{\"ok\":false,\"error\":\"invalid protocol or revision\"}");
    return;
  }
  if (!writeRawStateAtomic(body)) {
    sendJson(507, "{\"ok\":false,\"error\":\"could not persist state\"}");
    return;
  }

  uint32_t ack = 0;
  if (server.hasHeader("X-Maple-Ack-Through")) {
    ack = static_cast<uint32_t>(server.header("X-Maple-Ack-Through").toInt());
  }
  currentRevision = static_cast<uint32_t>(revision);
  acknowledgeChanges(ack, currentRevision);
  reloadStatePending = true;  // Parse after the HTTP body String is released.

  JsonDocument response;
  response["ok"] = true;
  response["revision"] = currentRevision;
  response["ackedThrough"] = ack;
  String out;
  serializeJson(response, out);
  sendJson(200, out);
}

static void setupHttp() {
  const char* headers[] = {"X-Maple-Ack-Through"};
  server.collectHeaders(headers, 1);

  server.on("/", HTTP_GET, []() {
    addCorsHeaders();
    server.send(200, "text/plain; charset=utf-8",
                "Maple X3 Receiver v0.2\nAPI: /api/v1/status\n");
  });
  server.on("/api/v1/status", HTTP_GET, handleStatus);
  server.on("/api/v1/changes", HTTP_GET, handleChanges);
  server.on("/api/v1/state", HTTP_POST, handlePostState);
  server.on("/api/v1/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/v1/changes", HTTP_OPTIONS, handleOptions);
  server.on("/api/v1/state", HTTP_OPTIONS, handleOptions);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) handleOptions();
    else sendJson(404, "{\"ok\":false,\"error\":\"not found\"}");
  });
  server.begin();
}

// -----------------------------------------------------------------------------
// Maple data helpers
// -----------------------------------------------------------------------------

enum class Page : uint8_t { Today = 0, Habits, Tasks, Gym, Projects, Write };
enum class Detail : uint8_t { None = 0, GymFolder, GymExercise, Project, Notebook };

Page page = Page::Today;
Detail detail = Detail::None;
int cursor = 0;
int subCursor = 0;
int openFolderIndex = -1;
int openExerciseIndex = -1;
int openProjectIndex = -1;
int openNotebookIndex = -1;
int notebookPage = 0;

static int clampCursor(int value, int count) {
  if (count <= 0) return 0;
  if (value < 0) return 0;
  if (value >= count) return count - 1;
  return value;
}

static const char* safeText(JsonVariantConst v, const char* key, const char* fallback = "") {
  const char* s = v[key] | fallback;
  return s ? s : fallback;
}

static int tasksCount() {
  return stateDoc["tasks"].is<JsonArray>() ? stateDoc["tasks"].as<JsonArray>().size() : 0;
}
static int habitsCount() {
  return stateDoc["habits"].is<JsonArray>() ? stateDoc["habits"].as<JsonArray>().size() : 0;
}
static int gymFoldersCount() {
  return stateDoc["exerciseFolders"].is<JsonArray>() ? stateDoc["exerciseFolders"].as<JsonArray>().size() : 0;
}
static int projectsCount() {
  return stateDoc["projects"].is<JsonArray>() ? stateDoc["projects"].as<JsonArray>().size() : 0;
}
static int notebooksCount() {
  return stateDoc["notebooks"].is<JsonArray>() ? stateDoc["notebooks"].as<JsonArray>().size() : 0;
}

static JsonObject getGymFolder(int fi) {
  JsonArray arr = stateDoc["exerciseFolders"].as<JsonArray>();
  return (fi >= 0 && fi < static_cast<int>(arr.size())) ? arr[fi].as<JsonObject>() : JsonObject();
}
static JsonObject getExercise(int fi, int ei) {
  JsonObject f = getGymFolder(fi);
  JsonArray arr = f["exercises"].as<JsonArray>();
  return (ei >= 0 && ei < static_cast<int>(arr.size())) ? arr[ei].as<JsonObject>() : JsonObject();
}
static JsonObject getProject(int pi) {
  JsonArray arr = stateDoc["projects"].as<JsonArray>();
  return (pi >= 0 && pi < static_cast<int>(arr.size())) ? arr[pi].as<JsonObject>() : JsonObject();
}

struct StepRef {
  bool valid = false;
  JsonObject step;
  const char* projectId = "";
  const char* folderId = "";
  const char* folderName = "";
};

static int projectStepCount(JsonObject project) {
  int n = 0;
  for (JsonObject folder : project["folders"].as<JsonArray>()) n += folder["steps"].as<JsonArray>().size();
  return n;
}

static StepRef projectStepAt(JsonObject project, int index) {
  StepRef r;
  int n = 0;
  for (JsonObject folder : project["folders"].as<JsonArray>()) {
    for (JsonObject step : folder["steps"].as<JsonArray>()) {
      if (n++ == index) {
        r.valid = true;
        r.step = step;
        r.projectId = project["id"] | "";
        r.folderId = folder["id"] | "";
        r.folderName = folder["name"] | "";
        return r;
      }
    }
  }
  return r;
}

static StepRef nextPendingStep(JsonObject project) {
  for (JsonObject folder : project["folders"].as<JsonArray>()) {
    for (JsonObject step : folder["steps"].as<JsonArray>()) {
      if (!(step["done"] | false)) {
        return StepRef{true, step, project["id"] | "", folder["id"] | "", folder["name"] | ""};
      }
    }
  }
  return StepRef{};
}

static int todayPendingTaskCount() {
  int n = 0;
  for (JsonObject t : stateDoc["tasks"].as<JsonArray>()) if (!(t["done"] | false)) ++n;
  return n;
}
static int todayPendingProjectCount() {
  int n = 0;
  for (JsonObject p : stateDoc["projects"].as<JsonArray>()) if (nextPendingStep(p).valid) ++n;
  return n;
}
static int todayCount() {
  return todayPendingTaskCount() + habitsCount() + todayPendingProjectCount();
}

static JsonObject todayPendingTaskAt(int index) {
  int n = 0;
  for (JsonObject t : stateDoc["tasks"].as<JsonArray>()) {
    if (!(t["done"] | false) && n++ == index) return t;
  }
  return JsonObject();
}

static JsonObject todayHabitAt(int index) {
  JsonArray arr = stateDoc["habits"].as<JsonArray>();
  return (index >= 0 && index < static_cast<int>(arr.size())) ? arr[index].as<JsonObject>() : JsonObject();
}

static StepRef todayProjectAt(int index) {
  int n = 0;
  for (JsonObject p : stateDoc["projects"].as<JsonArray>()) {
    StepRef r = nextPendingStep(p);
    if (r.valid && n++ == index) return r;
  }
  return StepRef{};
}

static const char* pageTitle() {
  switch (page) {
    case Page::Today: return "HOY";
    case Page::Habits: return "HABITOS";
    case Page::Tasks: return "TAREAS";
    case Page::Gym: return "EJERCICIOS";
    case Page::Projects: return "PROYECTOS";
    case Page::Write: return "ESCRIBIR";
  }
  return "MAPLE";
}

static int rootItemCount() {
  switch (page) {
    case Page::Today: return todayCount();
    case Page::Habits: return habitsCount();
    case Page::Tasks: return tasksCount();
    case Page::Gym: return gymFoldersCount();
    case Page::Projects: return projectsCount();
    case Page::Write: return notebooksCount();
  }
  return 0;
}

// -----------------------------------------------------------------------------
// Render Maple-like UI
// -----------------------------------------------------------------------------

static void drawHeader() {
  // North-star button: simple four-point star inside a circle.
  const int cx = logicalWidth() / 2;
  const int cy = 38;
  drawRoundedRect(cx - 27, cy - 27, 54, 54, 2, true);
  fillRect(cx - 2, cy - 16, 4, 32, true);
  fillRect(cx - 16, cy - 2, 32, 4, true);
  for (int i = 0; i < 8; ++i) {
    pixel(cx - 8 + i, cy - 8 + i, true);
    pixel(cx + 8 - i, cy - 8 + i, true);
  }

  const char* tabs[] = {"HAB", "TAR", "GYM", "PROY", "ESC"};
  const Page pages[] = {Page::Habits, Page::Tasks, Page::Gym, Page::Projects, Page::Write};
  const int gap = 5;
  const int left = 12;
  const int totalW = logicalWidth() - 24;
  const int boxW = (totalW - gap * 4) / 5;
  const int y = 78;
  for (int i = 0; i < 5; ++i) {
    const int x = left + i * (boxW + gap);
    const bool selected = page == pages[i] && detail == Detail::None;
    if (selected) fillRect(x, y, boxW, 42, true);
    else drawRoundedRect(x, y, boxW, 42, 2, true);
    const String s(tabs[i]);
    const int scale = i == 3 ? 1 : 2;
    const int tx = x + (boxW - textWidth(s, scale)) / 2;
    const int ty = y + (42 - 7 * scale) / 2;
    drawTextRaw(s, tx, ty, scale, !selected);
  }

  drawText(pageTitle(), 17, 139, 2, true);
  fillRect(17, 164, logicalWidth() - 34, 2, true);
}

static void drawListRow(int row, const char* text, bool selected, bool hasCheck = false, bool checked = false,
                        const char* right = nullptr) {
  const int y = 184 + row * 54;
  if (selected) drawSelectionBar(y);
  int x = 24;
  if (hasCheck) {
    drawCheck(24, y + 4, checked, selected);
    x = 60;
  }
  drawTextClipped(text, x, y + 7, 2, right ? 310 : 430, true);
  if (right && *right) drawTextClipped(right, 390, y + 10, 1, 110, true);
  fillRect(22, y + 42, logicalWidth() - 44, 1, true);
}

static void drawEmpty(const char* message) {
  drawCentered(message, 300, 2, true);
}

static void renderToday() {
  const int count = todayCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("NADA PENDIENTE"); return; }

  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  const int taskN = todayPendingTaskCount();
  const int habitN = habitsCount();
  const String date = currentDateKey();

  for (int r = 0; r < pageSize && start + r < count; ++r) {
    const int idx = start + r;
    if (idx < taskN) {
      JsonObject t = todayPendingTaskAt(idx);
      drawListRow(r, safeText(t, "text"), idx == cursor, true, t["done"] | false, "TAREA");
    } else if (idx < taskN + habitN) {
      JsonObject h = todayHabitAt(idx - taskN);
      bool on = h["marks"][date.c_str()] | false;
      drawListRow(r, safeText(h, "name"), idx == cursor, true, on, "HABITO");
    } else {
      StepRef s = todayProjectAt(idx - taskN - habitN);
      drawListRow(r, s.valid ? safeText(s.step, "text") : "", idx == cursor, true,
                  s.valid ? (s.step["done"] | false) : false, "PROY");
    }
  }
}

static void renderHabits() {
  const int count = habitsCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("SIN HABITOS"); return; }
  const String date = currentDateKey();
  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  JsonArray arr = stateDoc["habits"].as<JsonArray>();
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject h = arr[start + r].as<JsonObject>();
    bool on = h["marks"][date.c_str()] | false;
    drawListRow(r, safeText(h, "name"), start + r == cursor, true, on);
  }
}

static void renderTasks() {
  const int count = tasksCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("SIN TAREAS"); return; }
  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  JsonArray arr = stateDoc["tasks"].as<JsonArray>();
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject t = arr[start + r].as<JsonObject>();
    drawListRow(r, safeText(t, "text"), start + r == cursor, true, t["done"] | false);
  }
}

static void renderGymList() {
  const int count = gymFoldersCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("SIN RUTINAS"); return; }
  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  JsonArray arr = stateDoc["exerciseFolders"].as<JsonArray>();
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject f = arr[start + r].as<JsonObject>();
    char right[18];
    snprintf(right, sizeof(right), "%u EJ", static_cast<unsigned>(f["exercises"].as<JsonArray>().size()));
    drawListRow(r, safeText(f, "name"), start + r == cursor, false, false, right);
  }
}

static void renderGymFolder() {
  JsonObject f = getGymFolder(openFolderIndex);
  JsonArray ex = f["exercises"].as<JsonArray>();
  const int count = ex.size();
  subCursor = clampCursor(subCursor, count);
  drawTextClipped(safeText(f, "name"), 17, 139, 2, 390, true);
  if (!count) { drawEmpty("SIN EJERCICIOS"); return; }
  const int pageSize = 10;
  const int start = (subCursor / pageSize) * pageSize;
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject e = ex[start + r].as<JsonObject>();
    char right[32];
    const float kg = e["kg"] | 0.0f;
    const int reps = e["reps"] | 0;
    snprintf(right, sizeof(right), "%.1fKG %dR", kg, reps);
    drawListRow(r, safeText(e, "name"), start + r == subCursor, false, false, right);
  }
}

static void renderGymExercise() {
  JsonObject e = getExercise(openFolderIndex, openExerciseIndex);
  drawTextClipped(safeText(e, "name"), 17, 139, 2, 450, true);
  fillRect(17, 164, logicalWidth() - 34, 2, true);

  const int sets = max(0, e["sets"] | 0);
  subCursor = clampCursor(subCursor, sets + 1); // 0 = kg, 1..sets = set.
  const float kg = e["kg"] | 0.0f;
  const int reps = e["reps"] | 0;

  drawText("PESO", 40, 215, 2, true);
  if (subCursor == 0) drawSelectionBar(204);
  char weight[32];
  snprintf(weight, sizeof(weight), "< %.1f KG >", kg);
  drawText(weight, 235, 215, 2, true);

  drawText("REPS", 40, 285, 2, true);
  char repsText[16];
  snprintf(repsText, sizeof(repsText), "%d", reps);
  drawText(repsText, 300, 285, 2, true);

  drawText("SETS", 40, 365, 2, true);
  JsonArray done = e["done"].as<JsonArray>();
  const int box = 38;
  const int gap = 14;
  const int maxPerRow = 8;
  for (int i = 0; i < sets; ++i) {
    const int row = i / maxPerRow;
    const int col = i % maxPerRow;
    const int x = 40 + col * (box + gap);
    const int y = 410 + row * 62;
    const bool on = i < static_cast<int>(done.size()) ? (done[i] | false) : false;
    if (subCursor == i + 1) drawSelectionBar(y - 1);
    drawCheck(x + 8, y, on, subCursor == i + 1);
    char n[5]; snprintf(n, sizeof(n), "%d", i + 1);
    drawText(n, x + 16, y + 30, 1, true);
  }

  drawText("IZQ/DER: PESO  CONF: SET", 30, 700, 1, true);
}

static void renderProjectsList() {
  const int count = projectsCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("SIN PROYECTOS"); return; }
  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  JsonArray arr = stateDoc["projects"].as<JsonArray>();
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject p = arr[start + r].as<JsonObject>();
    const int total = projectStepCount(p);
    int done = 0;
    for (JsonObject folder : p["folders"].as<JsonArray>())
      for (JsonObject s : folder["steps"].as<JsonArray>()) if (s["done"] | false) ++done;
    char right[20];
    const int pct = total ? (done * 100 / total) : 0;
    snprintf(right, sizeof(right), "%d%%", pct);
    drawListRow(r, safeText(p, "name"), start + r == cursor, false, false, right);
  }
}

static void renderProjectDetail() {
  JsonObject p = getProject(openProjectIndex);
  drawTextClipped(safeText(p, "name"), 17, 139, 2, 450, true);
  fillRect(17, 164, logicalWidth() - 34, 2, true);
  const int count = projectStepCount(p);
  subCursor = clampCursor(subCursor, count);
  if (!count) { drawEmpty("SIN PASOS"); return; }
  const int pageSize = 10;
  const int start = (subCursor / pageSize) * pageSize;
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    StepRef s = projectStepAt(p, start + r);
    drawListRow(r, s.valid ? safeText(s.step, "text") : "", start + r == subCursor,
                true, s.valid ? (s.step["done"] | false) : false,
                s.valid ? s.folderName : "");
  }
}

static void renderWriteList() {
  const int count = notebooksCount();
  cursor = clampCursor(cursor, count);
  if (!count) { drawEmpty("SIN LIBRETAS"); return; }
  const int pageSize = 10;
  const int start = (cursor / pageSize) * pageSize;
  JsonArray arr = stateDoc["notebooks"].as<JsonArray>();
  for (int r = 0; r < pageSize && start + r < count; ++r) {
    JsonObject n = arr[start + r].as<JsonObject>();
    drawListRow(r, safeText(n, "name"), start + r == cursor, false, false, "LEER");
  }
}

static void renderNotebook() {
  JsonArray arr = stateDoc["notebooks"].as<JsonArray>();
  if (openNotebookIndex < 0 || openNotebookIndex >= static_cast<int>(arr.size())) return;
  JsonObject n = arr[openNotebookIndex].as<JsonObject>();
  drawTextClipped(safeText(n, "name"), 17, 139, 2, 450, true);
  fillRect(17, 164, logicalWidth() - 34, 2, true);

  String text = asciiUpper(safeText(n, "text"));
  text.replace("\r", "");
  const int charsPerLine = 40;
  const int linesPerPage = 22;
  const int charsPerPage = charsPerLine * linesPerPage;
  const int totalPages = max(1, static_cast<int>((text.length() + charsPerPage - 1) / charsPerPage));
  notebookPage = constrain(notebookPage, 0, totalPages - 1);
  int pos = notebookPage * charsPerPage;
  int y = 185;
  for (int line = 0; line < linesPerPage && pos < static_cast<int>(text.length()); ++line) {
    int newline = text.indexOf('\n', pos);
    int end = min(pos + charsPerLine, static_cast<int>(text.length()));
    if (newline >= pos && newline < end) end = newline;
    String part = text.substring(pos, end);
    drawTextRaw(part, 22, y, 1, true);
    y += 23;
    pos = (newline == end) ? end + 1 : end;
  }
  char footer[30];
  snprintf(footer, sizeof(footer), "PAG %d/%d", notebookPage + 1, totalPages);
  drawText(footer, 22, 720, 1, true);
}

static void renderWaiting() {
  display.clearScreen(0xFF);
  drawCentered("MAPLE X3", 175, 5, true);
  drawCentered("ESPERANDO MAPLE", 290, 2, true);
  drawCentered("WIFI: MAPLE-X3", 370, 2, true);
  drawCentered("192.168.4.1", 425, 2, true);
  if (!storageReady) drawCentered("SD NO DISPONIBLE", 520, 2, true);
  else drawCentered("MANTEN LA ESTRELLA EN MAPLE", 520, 1, true);
  refreshDisplay(true);
}

static void render() {
  if (!stateLoaded) { renderWaiting(); return; }
  display.clearScreen(0xFF);
  drawHeader();
  if (detail == Detail::GymFolder) renderGymFolder();
  else if (detail == Detail::GymExercise) renderGymExercise();
  else if (detail == Detail::Project) renderProjectDetail();
  else if (detail == Detail::Notebook) renderNotebook();
  else {
    switch (page) {
      case Page::Today: renderToday(); break;
      case Page::Habits: renderHabits(); break;
      case Page::Tasks: renderTasks(); break;
      case Page::Gym: renderGymList(); break;
      case Page::Projects: renderProjectsList(); break;
      case Page::Write: renderWriteList(); break;
    }
  }
  refreshDisplay(false);
}

// -----------------------------------------------------------------------------
// Local actions -> state + changes queue
// -----------------------------------------------------------------------------

static void toggleTask(JsonObject task) {
  if (task.isNull()) return;
  const bool next = !(task["done"] | false);
  task["done"] = next;
  queueChange("task.done", task["id"] | "", next);
  persistState();
}

static void toggleHabit(JsonObject habit) {
  if (habit.isNull()) return;
  const String date = currentDateKey();
  const bool old = habit["marks"][date.c_str()] | false;
  habit["marks"][date.c_str()] = !old;
  queueChange("habit.mark", habit["id"] | "", !old, date.c_str());
  persistState();
}

static void toggleExerciseSet(JsonObject exercise, int setIndex) {
  if (exercise.isNull()) return;
  const int sets = max(0, exercise["sets"] | 0);
  if (setIndex < 0 || setIndex >= sets) return;
  JsonArray done = exercise["done"].as<JsonArray>();
  while (static_cast<int>(done.size()) < sets) done.add(false);
  const bool next = !(done[setIndex] | false);
  done[setIndex] = next;
  queueChange("exercise.set", exercise["id"] | "", next, nullptr, setIndex);
  persistState();
}

static void adjustExerciseKg(JsonObject exercise, float delta) {
  if (exercise.isNull()) return;
  float kg = exercise["kg"] | 0.0f;
  kg = max(0.0f, kg + delta);
  // Current Maple Stepper uses integer kg changes; keep one decimal protocol-safe.
  kg = roundf(kg * 10.0f) / 10.0f;
  exercise["kg"] = kg;
  queueKgChange(exercise["id"] | "", kg);
  persistState();
}

static void toggleProjectStep(StepRef s) {
  if (!s.valid) return;
  const bool next = !(s.step["done"] | false);
  s.step["done"] = next;
  queueChange("project.step", s.step["id"] | "", next, nullptr, -1, s.projectId, s.folderId);
  persistState();
}

// -----------------------------------------------------------------------------
// Input handling
// -----------------------------------------------------------------------------

static void goRootPage(Page p) {
  page = p;
  detail = Detail::None;
  cursor = 0;
  subCursor = 0;
}

static void handleRootConfirm() {
  switch (page) {
    case Page::Today: {
      const int taskN = todayPendingTaskCount();
      const int habitN = habitsCount();
      if (cursor < taskN) toggleTask(todayPendingTaskAt(cursor));
      else if (cursor < taskN + habitN) toggleHabit(todayHabitAt(cursor - taskN));
      else toggleProjectStep(todayProjectAt(cursor - taskN - habitN));
      break;
    }
    case Page::Habits: {
      JsonArray arr = stateDoc["habits"].as<JsonArray>();
      if (cursor < static_cast<int>(arr.size())) toggleHabit(arr[cursor].as<JsonObject>());
      break;
    }
    case Page::Tasks: {
      JsonArray arr = stateDoc["tasks"].as<JsonArray>();
      if (cursor < static_cast<int>(arr.size())) toggleTask(arr[cursor].as<JsonObject>());
      break;
    }
    case Page::Gym:
      if (gymFoldersCount() > 0) {
        openFolderIndex = cursor;
        subCursor = 0;
        detail = Detail::GymFolder;
      }
      break;
    case Page::Projects:
      if (projectsCount() > 0) {
        openProjectIndex = cursor;
        subCursor = 0;
        detail = Detail::Project;
      }
      break;
    case Page::Write:
      if (notebooksCount() > 0) {
        openNotebookIndex = cursor;
        notebookPage = 0;
        detail = Detail::Notebook;
      }
      break;
  }
}

static void handleButton(uint8_t b) {
  if (!stateLoaded) {
    if (b == InputManager::BTN_CONFIRM) render();
    return;
  }

  if (detail == Detail::GymExercise) {
    JsonObject e = getExercise(openFolderIndex, openExerciseIndex);
    const int sets = max(0, e["sets"] | 0);
    if (b == InputManager::BTN_BACK) {
      detail = Detail::GymFolder;
      subCursor = openExerciseIndex;
    } else if (b == InputManager::BTN_UP) {
      subCursor = clampCursor(subCursor - 1, sets + 1);
    } else if (b == InputManager::BTN_DOWN) {
      subCursor = clampCursor(subCursor + 1, sets + 1);
    } else if (subCursor == 0 && b == InputManager::BTN_LEFT) {
      adjustExerciseKg(e, -1.0f);
    } else if (subCursor == 0 && b == InputManager::BTN_RIGHT) {
      adjustExerciseKg(e, 1.0f);
    } else if (subCursor > 0 && b == InputManager::BTN_CONFIRM) {
      toggleExerciseSet(e, subCursor - 1);
    }
    render();
    return;
  }

  if (detail == Detail::GymFolder) {
    JsonObject f = getGymFolder(openFolderIndex);
    const int count = f["exercises"].as<JsonArray>().size();
    if (b == InputManager::BTN_BACK) {
      detail = Detail::None;
      cursor = openFolderIndex;
    } else if (b == InputManager::BTN_UP) subCursor = clampCursor(subCursor - 1, count);
    else if (b == InputManager::BTN_DOWN) subCursor = clampCursor(subCursor + 1, count);
    else if (b == InputManager::BTN_CONFIRM && count > 0) {
      openExerciseIndex = subCursor;
      subCursor = 0;
      detail = Detail::GymExercise;
    }
    render();
    return;
  }

  if (detail == Detail::Project) {
    JsonObject p = getProject(openProjectIndex);
    const int count = projectStepCount(p);
    if (b == InputManager::BTN_BACK) {
      detail = Detail::None;
      cursor = openProjectIndex;
    } else if (b == InputManager::BTN_UP) subCursor = clampCursor(subCursor - 1, count);
    else if (b == InputManager::BTN_DOWN) subCursor = clampCursor(subCursor + 1, count);
    else if (b == InputManager::BTN_CONFIRM && count > 0) toggleProjectStep(projectStepAt(p, subCursor));
    render();
    return;
  }

  if (detail == Detail::Notebook) {
    JsonArray arr = stateDoc["notebooks"].as<JsonArray>();
    if (openNotebookIndex >= 0 && openNotebookIndex < static_cast<int>(arr.size())) {
      String text = safeText(arr[openNotebookIndex].as<JsonObject>(), "text");
      const int charsPerPage = 40 * 22;
      const int totalPages = max(1, static_cast<int>((text.length() + charsPerPage - 1) / charsPerPage));
      if (b == InputManager::BTN_BACK) {
        detail = Detail::None;
        cursor = openNotebookIndex;
      } else if (b == InputManager::BTN_LEFT || b == InputManager::BTN_UP) {
        notebookPage = max(0, notebookPage - 1);
      } else if (b == InputManager::BTN_RIGHT || b == InputManager::BTN_DOWN) {
        notebookPage = min(totalPages - 1, notebookPage + 1);
      }
    }
    render();
    return;
  }

  // Root navigation. Left/right mirrors Maple's tab strip.
  const int pageIndex = static_cast<int>(page);
  if (b == InputManager::BTN_LEFT) {
    goRootPage(static_cast<Page>((pageIndex + 5) % 6));
  } else if (b == InputManager::BTN_RIGHT) {
    goRootPage(static_cast<Page>((pageIndex + 1) % 6));
  } else if (b == InputManager::BTN_UP) {
    cursor = clampCursor(cursor - 1, rootItemCount());
  } else if (b == InputManager::BTN_DOWN) {
    cursor = clampCursor(cursor + 1, rootItemCount());
  } else if (b == InputManager::BTN_CONFIRM) {
    handleRootConfirm();
  } else if (b == InputManager::BTN_BACK) {
    goRootPage(Page::Today);
  }
  render();
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.printf("Maple X3 Receiver v%s booting...\n", FW_VERSION);

  BoardConfig::holdPowerRails();
  BoardConfig::releaseSdRail();

  display.setDisplayX3();
  freeink::applyXteinkDisplayController();
  display.begin();

  storageReady = SdMan.begin();
  if (storageReady) {
    SdMan.ensureDirectoryExists(MAPLE_DIR);
    loadState();
    loadChanges();
  } else {
    initChangesDoc();
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const bool apOk = WiFi.softAP(AP_SSID);
  Serial.printf("AP %s, IP=%s\n", apOk ? "ready" : "FAILED", WiFi.softAPIP().toString().c_str());
  if (MDNS.begin(MDNS_HOST)) MDNS.addService("http", "tcp", 80);
  setupHttp();

  input.begin();
  input.beginAsync(2, 15, 32);

  render();
  Serial.printf("Maple X3 ready. storage=%d state=%d revision=%lu\n",
                storageReady, stateLoaded, static_cast<unsigned long>(currentRevision));
}

void loop() {
  server.handleClient();

  if (reloadStatePending) {
    reloadStatePending = false;
    if (loadState()) {
      // A fresh state from Maple resets navigation but never removes unacked deltas.
      goRootPage(Page::Today);
      render();
    }
  }

  uint8_t button = 0;
  while (input.popPress(button)) handleButton(button);

  delay(2);
}
