#include <Arduino.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <InputManager.h>
#include <SDCardManager.h>
#include <XteinkDetect.h>
#include <esp_system.h>
#include <time.h>

// Maple X3 v8 standalone
// Data-compatible with Maple v8 JSON packages.
// No Wi-Fi dependency: state, import and export live on microSD.

static constexpr const char* FW_VERSION = "8.0.1";
static constexpr const char* MAPLE_DIR = "/Maple";
static constexpr const char* STATE_PATH = "/Maple/maple-x3-state.json";
static constexpr const char* STATE_TMP_PATH = "/Maple/maple-x3-state.tmp";
static constexpr const char* EXPORT_DIR = "/Maple/Exports";
static constexpr const char* IMPORT_DIR = "/Maple/Import";
static constexpr int MAX_IMPORT_FILES = 40;
static constexpr int TZ_OFFSET_SECONDS = -6 * 3600; // Guadalajara / central Mexico

// XTEINK X3 display SPI pinout (same proven mapping as Maple X3 v0.1).
static constexpr int EPD_SCLK = 8;
static constexpr int EPD_MOSI = 10;
static constexpr int EPD_CS   = 21;
static constexpr int EPD_DC   = 4;
static constexpr int EPD_RST  = 5;
static constexpr int EPD_BUSY = 6;

EInkDisplay display(EPD_SCLK, EPD_MOSI, EPD_CS, EPD_DC, EPD_RST, EPD_BUSY);
InputManager input;
JsonDocument stateDoc;
bool storageReady = false;
bool stateLoaded = false;
uint32_t redrawCount = 0;
bool darkMode = false;
uint64_t baseEpochMs = 0;
uint32_t baseMillis = 0;
String lastPurgeDay;


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
// Theme + persistence
// -----------------------------------------------------------------------------
static bool ink() { return !darkMode; }
static bool paper() { return darkMode; }

static void clearCanvas() { display.clearScreen(darkMode ? 0x00 : 0xFF); }

static void textT(const char* s, int x, int y, int scale = 2) { drawText(s, x, y, scale, ink()); }
static void textClipT(const char* s, int x, int y, int scale, int maxW) { drawTextClipped(s, x, y, scale, maxW, ink()); }
static void lineT(int x, int y, int w, int h = 1) { fillRect(x, y, w, h, ink()); }
static void borderT(int x, int y, int w, int h, int t = 2) { drawRoundedRect(x, y, w, h, t, ink()); }

static bool writeJsonAtomic(const char* finalPath, const char* tempPath, JsonDocument& doc) {
  if (!storageReady) return false;
  if (SdMan.exists(tempPath)) SdMan.remove(tempPath);
  FsFile f = SdMan.open(tempPath, O_WRITE | O_CREAT | O_TRUNC);
  if (!f) return false;
  const size_t n = serializeJson(doc, f);
  f.flush();
  f.close();
  if (!n) { SdMan.remove(tempPath); return false; }
  if (SdMan.exists(finalPath)) SdMan.remove(finalPath);
  return SdMan.rename(tempPath, finalPath);
}

static bool loadJsonFile(const char* path, JsonDocument& doc) {
  if (!storageReady || !SdMan.exists(path)) return false;
  FsFile f = SdMan.open(path, O_RDONLY);
  if (!f) return false;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  return !err;
}

static String nextId() {
  char b[36];
  snprintf(b, sizeof(b), "x3-%08lx-%08lx", (unsigned long)esp_random(), (unsigned long)millis());
  return String(b);
}

static const char* labelDefault(int pageIndex) {
  static const char* labels[] = {"HOY", "HABITOS", "TAREAS", "EJERCICIOS", "PROYECTOS", "ESCRIBIR"};
  return labels[constrain(pageIndex, 0, 5)];
}

static void ensureStore() {
  if (!stateDoc["labels"].is<JsonObject>()) stateDoc["labels"].to<JsonObject>();
  JsonObject labels = stateDoc["labels"].as<JsonObject>();
  static const char* keys[] = {"today","habits","tasks","exercises","projects","write"};
  for (int i = 0; i < 6; ++i) if (!labels[keys[i]].is<const char*>()) labels[keys[i]] = labelDefault(i);
  if (!stateDoc["tasks"].is<JsonArray>()) stateDoc["tasks"].to<JsonArray>();
  if (!stateDoc["habits"].is<JsonArray>()) stateDoc["habits"].to<JsonArray>();
  if (!stateDoc["folders"].is<JsonArray>()) stateDoc["folders"].to<JsonArray>();
  if (!stateDoc["projects"].is<JsonArray>()) stateDoc["projects"].to<JsonArray>();
  if (!stateDoc["notebooks"].is<JsonArray>()) stateDoc["notebooks"].to<JsonArray>();
  if (!stateDoc["recurring"].is<JsonArray>()) stateDoc["recurring"].to<JsonArray>();
  if (!stateDoc["_x3"].is<JsonObject>()) stateDoc["_x3"].to<JsonObject>();
  JsonObject x3 = stateDoc["_x3"].as<JsonObject>();
  darkMode = x3["dark"] | false;
  uint64_t savedEpoch = x3["epochMs"] | 0ULL;
  if (savedEpoch > 1577836800000ULL) baseEpochMs = savedEpoch;
  lastPurgeDay = String((const char*)(x3["lastPurgeDay"] | ""));
}

static uint64_t compileEpochMs() {
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  const char* p = strstr(months, mon);
  int month = p ? ((p - months) / 3 + 1) : 1;
  int day = (__DATE__[4] == ' ' ? 0 : (__DATE__[4]-'0')*10) + (__DATE__[5]-'0');
  int year = atoi(__DATE__ + 7);
  struct tm tmv{};
  tmv.tm_year = year - 1900; tmv.tm_mon = month - 1; tmv.tm_mday = day;
  tmv.tm_hour = 12; tmv.tm_isdst = 0;
  // Treat compile date as UTC-ish; only the date matters until a package gives us exportedAt.
  time_t t = mktime(&tmv);
  return t > 0 ? (uint64_t)t * 1000ULL : 1786464000000ULL;
}

static uint64_t nowEpochMs() {
  return baseEpochMs + (uint32_t)(millis() - baseMillis);
}

static void localTm(struct tm& out) {
  time_t t = (time_t)(nowEpochMs() / 1000ULL) + TZ_OFFSET_SECONDS;
  gmtime_r(&t, &out);
}

static String currentDateKey() {
  struct tm t{}; localTm(t);
  char b[16]; snprintf(b, sizeof(b), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  return String(b);
}

static String currentMmDd() {
  struct tm t{}; localTm(t);
  char b[8]; snprintf(b, sizeof(b), "%02d-%02d", t.tm_mon + 1, t.tm_mday);
  return String(b);
}

static int currentWeekdayMonday0() {
  struct tm t{}; localTm(t);
  return (t.tm_wday + 6) % 7;
}

static void saveClockIntoState() {
  JsonObject x3 = stateDoc["_x3"].as<JsonObject>();
  x3["dark"] = darkMode;
  x3["epochMs"] = nowEpochMs();
  x3["lastPurgeDay"] = lastPurgeDay;
  baseEpochMs = x3["epochMs"].as<uint64_t>();
  baseMillis = millis();
}

static bool persistState() {
  if (!storageReady) return false;
  saveClockIntoState();
  return writeJsonAtomic(STATE_PATH, STATE_TMP_PATH, stateDoc);
}

static void newStore() {
  stateDoc.clear();
  ensureStore();
  stateLoaded = true;
}

static bool loadState() {
  if (!loadJsonFile(STATE_PATH, stateDoc)) { newStore(); return false; }
  ensureStore();
  stateLoaded = true;
  return true;
}

// -----------------------------------------------------------------------------
// Data helpers and Maple v8 behavior
// -----------------------------------------------------------------------------
enum class Page : uint8_t { Today=0, Habits=1, Tasks=2, Gym=3, Projects=4, Write=5 };
enum class View : uint8_t {
  Root, HabitYear, GymFolder, GymExercise, Project, Notebook, NotebookTags, Grimorio,
  Settings, EditMenu, TextInput, NumberInput,
  Transfer, ImportPicker, ExportSelect,
  RecurFreq, RecurWeek, RecurMonth, RecurYear,
  Message
};

enum class EditKind : uint8_t { None, Task, Habit, GymFolder, Exercise, Project, ProjectFolder, ProjectStep, Notebook };
enum class TextTarget : uint8_t {
  None, AddTask, AddHabit, AddGymFolder, AddExercise, AddProject, AddProjectFolder, AddProjectStep,
  AddNotebook, RenameTask, RenameHabit, RenameGymFolder, RenameExercise, RenameProject, RenameProjectFolder,
  RenameProjectStep, RenameNotebook, RenameTab, NotebookText, AddNotebookTag, RecurringText
};
enum class NumberTarget : uint8_t { None, ExerciseSets, ExerciseReps, ExerciseKg };

Page page = Page::Today;
View view = View::Root;
View returnView = View::Root;
int cursor = 0;
int subCursor = 0;
int openA = -1, openB = -1, openC = -1;
bool editing = false;

EditKind editKind = EditKind::None;
int editA = -1, editB = -1, editC = -1;
TextTarget textTarget = TextTarget::None;
String inputBuffer;
String inputTitle;
int textA = -1, textB = -1, textC = -1;
int keyboardIndex = 0;
NumberTarget numberTarget = NumberTarget::None;
int numberValue = 0;
int numberDigit = 0;
String messageText;
View messageReturn = View::Root;

String recurText;
int recurFreq = 0; // 0 daily, 1 weekly, 2 monthly, 3 yearly
bool recurWeek[7] = {false,false,false,false,false,false,false};
bool recurMonth[31] = {false};
bool recurYear[12][31] = {{false}};
int recurPickMonth = 0;

bool exportSel[5] = {true,true,true,true,true};
String importFiles[MAX_IMPORT_FILES];
int importFileCount = 0;

static int clampCursor(int v, int n) { if (n <= 0) return 0; return constrain(v, 0, n-1); }
static const char* safeText(JsonVariantConst v, const char* key, const char* fallback="") {
  const char* s = v[key] | fallback; return s ? s : fallback;
}

static JsonArray tasks() { return stateDoc["tasks"].as<JsonArray>(); }
static JsonArray habits() { return stateDoc["habits"].as<JsonArray>(); }
static JsonArray folders() { return stateDoc["folders"].as<JsonArray>(); }
static JsonArray projects() { return stateDoc["projects"].as<JsonArray>(); }
static JsonArray notebooks() { return stateDoc["notebooks"].as<JsonArray>(); }
static JsonArray recurring() { return stateDoc["recurring"].as<JsonArray>(); }

static int pendingTaskCount() { int n=0; for (JsonObject t: tasks()) if (!(t["done"]|false)) ++n; return n; }
static JsonObject pendingTaskAt(int idx) { int n=0; for (JsonObject t: tasks()) if (!(t["done"]|false) && n++==idx) return t; return JsonObject(); }

struct StepRef { bool valid=false; JsonObject step; int pi=-1, fi=-1, si=-1; const char* projectName=""; const char* folderName=""; };
static StepRef nextPendingStep(int pi) {
  JsonArray ps = projects(); if (pi<0 || pi>=(int)ps.size()) return {};
  JsonObject p = ps[pi].as<JsonObject>(); int fi=0;
  for (JsonObject f: p["folders"].as<JsonArray>()) { int si=0; for (JsonObject st: f["steps"].as<JsonArray>()) {
    if (!(st["done"]|false)) return {true, st, pi, fi, si, p["name"]|"", f["name"]|""}; ++si; } ++fi; }
  return {};
}
static int pendingProjectCount() { int n=0; for(int i=0;i<(int)projects().size();++i) if(nextPendingStep(i).valid) ++n; return n; }
static StepRef pendingProjectAt(int idx) { int n=0; for(int i=0;i<(int)projects().size();++i){auto r=nextPendingStep(i); if(r.valid && n++==idx) return r;} return {}; }

static int projectPct(int pi) {
  JsonArray ps=projects(); if(pi<0||pi>=(int)ps.size()) return 0; JsonObject p=ps[pi].as<JsonObject>();
  int total=0, done=0; for(JsonObject f:p["folders"].as<JsonArray>()) for(JsonObject st:f["steps"].as<JsonArray>()){++total;if(st["done"]|false)++done;}
  return total? (done*100/total):0;
}

static void normalizeExerciseDone(JsonObject e) {
  int sets=max(0,e["sets"]|0); if(!e["done"].is<JsonArray>()) e["done"].to<JsonArray>(); JsonArray d=e["done"].as<JsonArray>();
  while((int)d.size()<sets) d.add(false); while((int)d.size()>sets) d.remove(d.size()-1);
}

static void toggleTask(JsonObject t) { if(t.isNull())return; t["done"]=!(t["done"]|false); persistState(); }
static void toggleHabit(JsonObject h) {
  if(h.isNull())return; String d=currentDateKey(); if(!h["marks"].is<JsonObject>())h["marks"].to<JsonObject>();
  bool v=h["marks"][d.c_str()]|false; h["marks"][d.c_str()]=!v; persistState();
}
static void toggleStep(StepRef r) { if(!r.valid)return; r.step["done"]=!(r.step["done"]|false); persistState(); }

static JsonObject findTaskByRid(const char* rid) {
  if(!rid||!*rid)return JsonObject(); for(JsonObject t:tasks()) if(String((const char*)(t["rid"]|""))==rid) return t; return JsonObject();
}

static bool recurringMatches(JsonObject r, const String& day) {
  const char* f=r["freq"]|"";
  if(!strcmp(f,"daily")) return true;
  if(!strcmp(f,"weekly")) { int wd=currentWeekdayMonday0(); for(int x:r["weekdays"].as<JsonArray>()) if(x==wd)return true; return false; }
  struct tm t{}; localTm(t);
  if(!strcmp(f,"monthly")) { for(int x:r["monthdays"].as<JsonArray>()) if(x==t.tm_mday)return true; return false; }
  if(!strcmp(f,"yearly")) { String md=currentMmDd(); for(const char* x:r["yeardays"].as<JsonArray>()) if(md==x)return true; return false; }
  return false;
}

static void applyRecurring() {
  String day=currentDateKey(); bool changed=false;
  for(JsonObject r:recurring()) {
    if(!recurringMatches(r,day) || String((const char*)(r["lastApplied"]|""))==day) continue;
    JsonObject existing=findTaskByRid(r["id"]|"");
    if(existing.isNull()) { JsonObject t=tasks().add<JsonObject>(); t["id"]=nextId(); t["text"]=r["text"]|""; t["done"]=false; t["rid"]=r["id"]|""; }
    else if(existing["done"]|false) existing["done"]=false;
    r["lastApplied"]=day; changed=true;
  }
  if(changed) persistState();
}

static void purgeCompletedAfter2am() {
  struct tm t{}; localTm(t); if(t.tm_hour<2)return; String day=currentDateKey(); if(lastPurgeDay==day)return;
  JsonArray a=tasks(); for(int i=(int)a.size()-1;i>=0;--i) if(a[i]["done"]|false) a.remove(i);
  lastPurgeDay=day; persistState();
}

static void swapArrayItems(JsonArray arr, int i, int j) {
  if(i<0||j<0||i>=(int)arr.size()||j>=(int)arr.size()||i==j)return;
  JsonDocument tmp; tmp.set(arr[i]); JsonDocument tmp2; tmp2.set(arr[j]); arr[i].set(tmp2); arr[j].set(tmp);
}

// -----------------------------------------------------------------------------
// Maple v8 package import/export (microSD)
// -----------------------------------------------------------------------------
static bool endsJson(const String& s) { String x=s; x.toLowerCase(); return x.endsWith(".json"); }
static void addImportFile(const String& path) {
  if(importFileCount>=MAX_IMPORT_FILES || !endsJson(path) || path==STATE_PATH) return;
  for(int i=0;i<importFileCount;++i) if(importFiles[i]==path)return;
  importFiles[importFileCount++]=path;
}

static void scanDirForJson(const char* dirPath) {
  if(!storageReady || !SdMan.exists(dirPath)) return;
  FsFile dir=SdMan.open(dirPath,O_RDONLY); if(!dir || !dir.isDir()){if(dir)dir.close();return;}
  FsFile entry;
  while(entry.openNext(&dir,O_RDONLY)) {
    if(!entry.isDir()) {
      char name[160]={0}; entry.getName(name,sizeof(name)); String n(name);
      if(endsJson(n)) { String p=String(dirPath); if(p!="/")p+="/"; p+=n; addImportFile(p); }
    }
    entry.close();
  }
  dir.close();
}

static void scanImportFiles() {
  importFileCount=0; scanDirForJson("/"); scanDirForJson(MAPLE_DIR); scanDirForJson(IMPORT_DIR); scanDirForJson(EXPORT_DIR);
}

static JsonObject findByText(JsonArray arr, const char* key, const char* value) {
  String want=String(value?value:""); want.trim();
  for(JsonObject o:arr){String got=String((const char*)(o[key]|""));got.trim();if(got==want)return o;} return JsonObject();
}

static void copyIntoArray(JsonArray dest, JsonVariantConst src) { JsonObject o=dest.add<JsonObject>(); o.set(src); }

static void mergePackage(JsonDocument& in) {
  // Tasks: add by text; imported done=true can mark an existing task done.
  if(in["tasks"].is<JsonArray>()) for(JsonObjectConst t:in["tasks"].as<JsonArrayConst>()) {
    JsonObject cur=findByText(tasks(),"text",t["text"]|"");
    if(cur.isNull()) copyIntoArray(tasks(),t); else if(t["done"]|false) cur["done"]=true;
  }
  // Habits: add by name; preserve local marks when dates overlap.
  if(in["habits"].is<JsonArray>()) for(JsonObjectConst h:in["habits"].as<JsonArrayConst>()) {
    JsonObject cur=findByText(habits(),"name",h["name"]|"");
    if(cur.isNull()) copyIntoArray(habits(),h); else {
      if(!cur["marks"].is<JsonObject>())cur["marks"].to<JsonObject>(); JsonObject m=cur["marks"].as<JsonObject>();
      for(JsonPairConst kv:h["marks"].as<JsonObjectConst>()) if(!m.containsKey(kv.key().c_str())) m[kv.key().c_str()]=kv.value();
    }
  }
  // Exercises: add folders by name and missing exercises by name, matching Maple v8 merge semantics.
  if(in["folders"].is<JsonArray>()) for(JsonObjectConst f:in["folders"].as<JsonArrayConst>()) {
    JsonObject cur=findByText(folders(),"name",f["name"]|"");
    if(cur.isNull()) copyIntoArray(folders(),f); else {
      if(!cur["exercises"].is<JsonArray>())cur["exercises"].to<JsonArray>(); JsonArray exs=cur["exercises"].as<JsonArray>();
      for(JsonObjectConst e:f["exercises"].as<JsonArrayConst>()) if(findByText(exs,"name",e["name"]|"").isNull()) copyIntoArray(exs,e);
    }
  }
  // Projects: add missing project/folder/step; imported done=true marks existing steps.
  if(in["projects"].is<JsonArray>()) for(JsonObjectConst p:in["projects"].as<JsonArrayConst>()) {
    JsonObject cur=findByText(projects(),"name",p["name"]|"");
    if(cur.isNull()) copyIntoArray(projects(),p); else {
      if(!cur["folders"].is<JsonArray>())cur["folders"].to<JsonArray>(); JsonArray pfs=cur["folders"].as<JsonArray>();
      for(JsonObjectConst f:p["folders"].as<JsonArrayConst>()) {
        JsonObject cf=findByText(pfs,"name",f["name"]|"");
        if(cf.isNull()) copyIntoArray(pfs,f); else {
          if(!cf["steps"].is<JsonArray>())cf["steps"].to<JsonArray>(); JsonArray sts=cf["steps"].as<JsonArray>();
          for(JsonObjectConst st:f["steps"].as<JsonArrayConst>()) { JsonObject cs=findByText(sts,"text",st["text"]|""); if(cs.isNull())copyIntoArray(sts,st); else if(st["done"]|false)cs["done"]=true; }
        }
      }
    }
  }
  // Notebooks: newer updatedAt wins.
  if(in["notebooks"].is<JsonArray>()) for(JsonObjectConst n:in["notebooks"].as<JsonArrayConst>()) {
    JsonObject cur=findByText(notebooks(),"name",n["name"]|"");
    if(cur.isNull()) copyIntoArray(notebooks(),n); else if((uint64_t)(n["updatedAt"]|0ULL)>(uint64_t)(cur["updatedAt"]|0ULL)) { cur["text"]=n["text"]|""; cur["updatedAt"]=n["updatedAt"]|0ULL; if(n["tags"].is<JsonArray>())cur["tags"].set(n["tags"]); }
  }
  if(in["recurring"].is<JsonArray>()) for(JsonObjectConst r:in["recurring"].as<JsonArrayConst>()) if(findByText(recurring(),"text",r["text"]|"").isNull()) copyIntoArray(recurring(),r);
  uint64_t exported=in["exportedAt"]|0ULL; if(exported>1577836800000ULL){baseEpochMs=exported;baseMillis=millis();}
  ensureStore(); applyRecurring(); persistState();
}

static bool importPackageFile(const String& path) {
  JsonDocument in; if(!loadJsonFile(path.c_str(),in))return false; mergePackage(in); return true;
}

static bool exportPackage() {
  if(!storageReady)return false; SdMan.ensureDirectoryExists(EXPORT_DIR);
  JsonDocument out; out["app"]="maple"; out["version"]=1; out["exportedAt"]=nowEpochMs();
  if(exportSel[0])out["habits"].set(stateDoc["habits"]);
  if(exportSel[1]){out["tasks"].set(stateDoc["tasks"]);out["recurring"].set(stateDoc["recurring"]);}
  if(exportSel[2])out["folders"].set(stateDoc["folders"]);
  if(exportSel[3])out["projects"].set(stateDoc["projects"]);
  if(exportSel[4])out["notebooks"].set(stateDoc["notebooks"]);
  String path;
  for(int n=1;n<1000;++n){char b[80];snprintf(b,sizeof(b),"%s/maple-x3-%03d.json",EXPORT_DIR,n);if(!SdMan.exists(b)){path=b;break;}}
  if(!path.length())return false; FsFile f=SdMan.open(path.c_str(),O_WRITE|O_CREAT|O_TRUNC);if(!f)return false;
  size_t n=serializeJsonPretty(out,f);f.flush();f.close(); if(!n)return false; messageText=String("GUARDADO: ")+path; return true;
}

// -----------------------------------------------------------------------------
// Text / edit helpers
// -----------------------------------------------------------------------------
static void showMessage(const String& msg, View back=View::Root) { messageText=msg; messageReturn=back; view=View::Message; cursor=0; }

static void startText(TextTarget target, const String& initial, const char* title, View back, int a=-1,int b=-1,int c=-1) {
  textTarget=target; inputBuffer=initial; inputTitle=title; returnView=back; textA=a;textB=b;textC=c;keyboardIndex=0;view=View::TextInput;
}

static void startNumber(NumberTarget target, int value, View back) { numberTarget=target;numberValue=constrain(value,0,99);numberDigit=0;returnView=back;view=View::NumberInput; }

static JsonObject objectAt(EditKind kind,int a,int b,int c) {
  if(kind==EditKind::Task){JsonArray x=tasks();return a>=0&&a<(int)x.size()?x[a].as<JsonObject>():JsonObject();}
  if(kind==EditKind::Habit){JsonArray x=habits();return a>=0&&a<(int)x.size()?x[a].as<JsonObject>():JsonObject();}
  if(kind==EditKind::GymFolder){JsonArray x=folders();return a>=0&&a<(int)x.size()?x[a].as<JsonObject>():JsonObject();}
  if(kind==EditKind::Exercise){JsonArray fs=folders();if(a<0||a>=(int)fs.size())return {};JsonArray es=fs[a]["exercises"].as<JsonArray>();return b>=0&&b<(int)es.size()?es[b].as<JsonObject>():JsonObject();}
  if(kind==EditKind::Project){JsonArray x=projects();return a>=0&&a<(int)x.size()?x[a].as<JsonObject>():JsonObject();}
  if(kind==EditKind::ProjectFolder){JsonArray ps=projects();if(a<0||a>=(int)ps.size())return {};JsonArray fs=ps[a]["folders"].as<JsonArray>();return b>=0&&b<(int)fs.size()?fs[b].as<JsonObject>():JsonObject();}
  if(kind==EditKind::ProjectStep){JsonArray ps=projects();if(a<0||a>=(int)ps.size())return {};JsonArray fs=ps[a]["folders"].as<JsonArray>();if(b<0||b>=(int)fs.size())return {};JsonArray ss=fs[b]["steps"].as<JsonArray>();return c>=0&&c<(int)ss.size()?ss[c].as<JsonObject>():JsonObject();}
  if(kind==EditKind::Notebook){JsonArray x=notebooks();return a>=0&&a<(int)x.size()?x[a].as<JsonObject>():JsonObject();}
  return {};
}

static JsonArray arrayForEdit(EditKind kind,int a,int b) {
  if(kind==EditKind::Task)return tasks(); if(kind==EditKind::Habit)return habits(); if(kind==EditKind::GymFolder)return folders(); if(kind==EditKind::Project)return projects(); if(kind==EditKind::Notebook)return notebooks();
  if(kind==EditKind::Exercise){JsonArray fs=folders();return (a>=0&&a<(int)fs.size())?fs[a]["exercises"].as<JsonArray>():JsonArray();}
  if(kind==EditKind::ProjectFolder){JsonArray ps=projects();return (a>=0&&a<(int)ps.size())?ps[a]["folders"].as<JsonArray>():JsonArray();}
  if(kind==EditKind::ProjectStep){JsonArray ps=projects();if(a<0||a>=(int)ps.size())return {};JsonArray fs=ps[a]["folders"].as<JsonArray>();return (b>=0&&b<(int)fs.size())?fs[b]["steps"].as<JsonArray>():JsonArray();}
  return {};
}

static void openEdit(EditKind k,int a,int b=-1,int c=-1,View back=View::Root){editKind=k;editA=a;editB=b;editC=c;returnView=back;cursor=0;view=View::EditMenu;}

static void commitText() {
  String s=inputBuffer; s.trim(); if(textTarget!=TextTarget::NotebookText && !s.length()){showMessage("TEXTO VACIO",returnView);return;}
  JsonObject o;
  switch(textTarget){
    case TextTarget::AddTask:{o=tasks().add<JsonObject>();o["id"]=nextId();o["text"]=s;o["done"]=false;break;}
    case TextTarget::AddHabit:{o=habits().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["marks"].to<JsonObject>();break;}
    case TextTarget::AddGymFolder:{o=folders().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["exercises"].to<JsonArray>();break;}
    case TextTarget::AddExercise:{JsonArray fs=folders();if(textA>=0&&textA<(int)fs.size()){o=fs[textA]["exercises"].as<JsonArray>().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["sets"]=4;o["reps"]=12;o["kg"]=10;o["done"].to<JsonArray>();}break;}
    case TextTarget::AddProject:{o=projects().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["folders"].to<JsonArray>();break;}
    case TextTarget::AddProjectFolder:{JsonArray ps=projects();if(textA>=0&&textA<(int)ps.size()){o=ps[textA]["folders"].as<JsonArray>().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["collapsed"]=false;o["steps"].to<JsonArray>();}break;}
    case TextTarget::AddProjectStep:{JsonArray ps=projects();if(textA>=0&&textA<(int)ps.size()){JsonArray fs=ps[textA]["folders"].as<JsonArray>();if(textB>=0&&textB<(int)fs.size()){o=fs[textB]["steps"].as<JsonArray>().add<JsonObject>();o["id"]=nextId();o["text"]=s;o["done"]=false;}}break;}
    case TextTarget::AddNotebook:{o=notebooks().add<JsonObject>();o["id"]=nextId();o["name"]=s;o["text"]="";o["updatedAt"]=nowEpochMs();o["tags"].to<JsonArray>();break;}
    case TextTarget::RenameTask:o=objectAt(EditKind::Task,textA,-1,-1);if(!o.isNull())o["text"]=s;break;
    case TextTarget::RenameHabit:o=objectAt(EditKind::Habit,textA,-1,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameGymFolder:o=objectAt(EditKind::GymFolder,textA,-1,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameExercise:o=objectAt(EditKind::Exercise,textA,textB,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameProject:o=objectAt(EditKind::Project,textA,-1,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameProjectFolder:o=objectAt(EditKind::ProjectFolder,textA,textB,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameProjectStep:o=objectAt(EditKind::ProjectStep,textA,textB,textC);if(!o.isNull())o["text"]=s;break;
    case TextTarget::RenameNotebook:o=objectAt(EditKind::Notebook,textA,-1,-1);if(!o.isNull())o["name"]=s;break;
    case TextTarget::RenameTab:{static const char* keys[]={"today","habits","tasks","exercises","projects","write"};if(textA>=0&&textA<6)stateDoc["labels"][keys[textA]]=s;break;}
    case TextTarget::NotebookText:{o=objectAt(EditKind::Notebook,textA,-1,-1);if(!o.isNull()){o["text"]=inputBuffer;o["updatedAt"]=nowEpochMs();}break;}
    case TextTarget::AddNotebookTag:{o=objectAt(EditKind::Notebook,textA,-1,-1);if(!o.isNull()){if(!o["tags"].is<JsonArray>())o["tags"].to<JsonArray>();JsonArray ta=o["tags"].as<JsonArray>();bool exists=false;for(const char* x:ta)if(s==x)exists=true;if(!exists)ta.add(s);}break;}
    case TextTarget::RecurringText:{recurText=s;cursor=0;view=View::RecurFreq;return;}
    default:break;
  }
  persistState(); view=returnView; cursor=0;
}

static void commitNumber() {
  JsonArray fs=folders();if(openA<0||openA>=(int)fs.size()){view=returnView;return;}JsonArray es=fs[openA]["exercises"].as<JsonArray>();if(openB<0||openB>=(int)es.size()){view=returnView;return;}JsonObject e=es[openB].as<JsonObject>();
  if(numberTarget==NumberTarget::ExerciseSets){e["sets"]=numberValue;normalizeExerciseDone(e);} else if(numberTarget==NumberTarget::ExerciseReps)e["reps"]=numberValue; else if(numberTarget==NumberTarget::ExerciseKg)e["kg"]=numberValue;
  persistState();view=returnView;
}

static void deleteEditItem() {
  JsonArray arr=arrayForEdit(editKind,editA,editB);int idx=(editKind==EditKind::Exercise||editKind==EditKind::ProjectFolder)?editB:(editKind==EditKind::ProjectStep?editC:editA);
  if(editKind==EditKind::Task){JsonObject t=objectAt(editKind,editA,-1,-1);String rid=t["rid"]|"";if(rid.length()){JsonArray rr=recurring();for(int i=(int)rr.size()-1;i>=0;--i)if(String((const char*)(rr[i]["id"]|""))==rid)rr.remove(i);}}
  if(!arr.isNull()&&idx>=0&&idx<(int)arr.size())arr.remove(idx);persistState();view=returnView;cursor=0;
}

static TextTarget renameTargetForEdit() {
  switch(editKind){case EditKind::Task:return TextTarget::RenameTask;case EditKind::Habit:return TextTarget::RenameHabit;case EditKind::GymFolder:return TextTarget::RenameGymFolder;case EditKind::Exercise:return TextTarget::RenameExercise;case EditKind::Project:return TextTarget::RenameProject;case EditKind::ProjectFolder:return TextTarget::RenameProjectFolder;case EditKind::ProjectStep:return TextTarget::RenameProjectStep;case EditKind::Notebook:return TextTarget::RenameNotebook;default:return TextTarget::None;}
}
static const char* editTextKey(){return (editKind==EditKind::Task||editKind==EditKind::ProjectStep)?"text":"name";}

// -----------------------------------------------------------------------------
// Rendering
// -----------------------------------------------------------------------------
static const char* pageKey(Page p){static const char* k[]={"today","habits","tasks","exercises","projects","write"};return k[(int)p];}
static String pageTitle(){const char* s=stateDoc["labels"][pageKey(page)]|labelDefault((int)page);return asciiUpper(s);}

static void drawSparkle(int cx,int cy,bool selected){
  if(selected)fillRect(cx-27,cy-27,54,54,ink());else borderT(cx-27,cy-27,54,54,2);
  bool c=selected?paper():ink(); fillRect(cx-2,cy-17,4,34,c);fillRect(cx-17,cy-2,34,4,c);
  for(int i=0;i<9;++i){pixel(cx-9+i,cy-9+i,c);pixel(cx+9-i,cy-9+i,c);} }

static void drawHeader() {
  drawSparkle(logicalWidth()/2,38,page==Page::Today&&view==View::Root);
  const char* tabs[]={"HAB","TAR","GYM","PROY","ESC"}; Page ps[]={Page::Habits,Page::Tasks,Page::Gym,Page::Projects,Page::Write};
  int gap=5,left=12,totalW=logicalWidth()-24,boxW=(totalW-gap*4)/5,y=78;
  for(int i=0;i<5;++i){int x=left+i*(boxW+gap);bool sel=page==ps[i]&&view==View::Root;if(sel)fillRect(x,y,boxW,42,ink());else borderT(x,y,boxW,42,2);String s=tabs[i];int sc=i==3?1:2;int tx=x+(boxW-textWidth(s,sc))/2;drawTextRaw(s,tx,y+(42-7*sc)/2,sc,sel?paper():ink());}
  String t=pageTitle();drawTextRaw(t,17,139,2,ink());lineT(17,164,logicalWidth()-34,2);
}

static void drawEditButton(bool selected){int y=174;if(selected)fillRect(17,y,logicalWidth()-34,34,ink());else borderT(17,y,logicalWidth()-34,34,1);String s=editing?"LISTO":"EDITAR";drawTextRaw(s,logicalWidth()/2-textWidth(s,1)/2,y+12,1,selected?paper():ink());}

static void drawRow(int row,const String& text,bool selected,bool check=false,bool checked=false,const String& right="") {
  int y=220+row*48; if(selected)fillRect(17,y,logicalWidth()-34,40,ink()); bool c=selected?paper():ink(); int x=25;
  if(check){drawRoundedRect(25,y+8,22,22,2,c);if(checked){fillRect(29,y+12,14,14,c); /* simple inner mark */}x=58;}
  drawTextClipped(text.c_str(),x,y+12,2,right.length()?300:440,c); if(right.length())drawTextClipped(right.c_str(),385,y+15,1,120,c);
}

static void drawActionRow(int row,const char* text,bool selected){drawRow(row,String(text),selected,false,false,"");}

static int rootDataCount(){
  if(page==Page::Today)return pendingTaskCount()+(int)habits().size()+pendingProjectCount();
  if(page==Page::Habits)return habits().size(); if(page==Page::Tasks)return tasks().size(); if(page==Page::Gym)return folders().size(); if(page==Page::Projects)return projects().size(); if(page==Page::Write)return notebooks().size()+1; return 0;
}
static int rootActionCount(){if(page==Page::Today)return editing?1:0;if(page==Page::Tasks)return editing?3:2;return editing?2:1;}
static int rootCount(){return rootDataCount()+rootActionCount();}

static void renderRoot(){
  clearCanvas();drawHeader();drawEditButton(cursor==-1);
  int count=rootCount();if(cursor>=count)cursor=max(0,count-1);if(cursor< -1)cursor=-1;
  int pageSize=11;int start=cursor<0?0:(cursor/pageSize)*pageSize;int row=0;
  if(page==Page::Today){
    int taskN=pendingTaskCount(),habitN=habits().size(),projN=pendingProjectCount();
    for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i<taskN){JsonObject t=pendingTaskAt(i);drawRow(row,safeText(t,"text"),sel,true,false,"TAREA");}
      else if(i<taskN+habitN){JsonObject h=habits()[i-taskN].as<JsonObject>();String d=currentDateKey();drawRow(row,safeText(h,"name"),sel,true,h["marks"][d.c_str()]|false,"HAB");}
      else if(i<taskN+habitN+projN){auto r=pendingProjectAt(i-taskN-habitN);drawRow(row,safeText(r.step,"text"),sel,false,false,r.projectName);}
      else drawActionRow(row,"AJUSTES",sel);
    }
  } else if(page==Page::Habits){int n=habits().size();for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i<n){JsonObject h=habits()[i].as<JsonObject>();String d=currentDateKey();drawRow(row,safeText(h,"name"),sel,true,h["marks"][d.c_str()]|false,"");}else if(i==n)drawActionRow(row,"+ AGREGAR HABITO",sel);else drawActionRow(row,"AJUSTES",sel);}}
  else if(page==Page::Tasks){int n=tasks().size();for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i<n){JsonObject t=tasks()[i].as<JsonObject>();drawRow(row,safeText(t,"text"),sel,true,t["done"]|false,(t["rid"].is<const char*>()?"REP":""));}else if(i==n)drawActionRow(row,"+ AGREGAR TAREA",sel);else if(i==n+1)drawActionRow(row,"REPETIR...",sel);else drawActionRow(row,"AJUSTES",sel);}}
  else if(page==Page::Gym){int n=folders().size();for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i<n){JsonObject f=folders()[i].as<JsonObject>();String r=String(f["exercises"].as<JsonArray>().size())+" EJ";drawRow(row,safeText(f,"name"),sel,false,false,r);}else if(i==n)drawActionRow(row,"+ AGREGAR CARPETA",sel);else drawActionRow(row,"AJUSTES",sel);}}
  else if(page==Page::Projects){int n=projects().size();for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i<n){JsonObject p=projects()[i].as<JsonObject>();drawRow(row,safeText(p,"name"),sel,false,false,String(projectPct(i))+"%");}else if(i==n)drawActionRow(row,"+ AGREGAR PROYECTO",sel);else drawActionRow(row,"AJUSTES",sel);}}
  else if(page==Page::Write){int n=notebooks().size();for(int i=start;i<count&&row<pageSize;++i,++row){bool sel=cursor==i;if(i==0)drawActionRow(row,"GRIMORIO",sel);else if(i<=n){JsonObject no=notebooks()[i-1].as<JsonObject>();drawRow(row,safeText(no,"name"),sel,false,false,"LIBRETA");}else if(i==n+1)drawActionRow(row,"+ AGREGAR LIBRETA",sel);else drawActionRow(row,"AJUSTES",sel);}}
  if(!count)drawText("SIN DATOS",24,250,2,ink());
  drawText("L/R PESTANA  OK ABRIR  MANTEN OK TRANSFERIR",18,756,1,ink());refreshDisplay(false);
}

static void renderHabitYear(){
  clearCanvas();drawHeader();JsonArray hs=habits();if(openA<0||openA>=(int)hs.size()){view=View::Root;renderRoot();return;}JsonObject h=hs[openA].as<JsonObject>();
  textClipT(safeText(h,"name"),18,184,2,470);struct tm now{};localTm(now);int year=now.tm_year+1900;drawText(String(year).c_str(),18,215,1,ink());
  struct tm jan{};jan.tm_year=year-1900;jan.tm_mon=0;jan.tm_mday=1;jan.tm_hour=12;time_t jt=mktime(&jan);struct tm jtm{};localtime_r(&jt,&jtm);int offset=(jtm.tm_wday+6)%7;
  static const int mdays[]={31,28,31,30,31,30,31,31,30,31,30,31};auto leap=[&](){return (year%4==0&&year%100!=0)||year%400==0;};
  int y0=255,cell=7,gap=2;for(int day=0;day<(leap()?366:365);++day){int w=(offset+day)/7,d=(offset+day)%7;drawRect(18+w*(cell+gap),y0+d*(cell+gap),cell,cell,1,ink());}
  int marked=0;for(JsonPair kv:h["marks"].as<JsonObject>()){if(!(kv.value()|false))continue;String k=kv.key().c_str();if(k.length()<10||k.substring(0,4).toInt()!=year)continue;int mo=k.substring(5,7).toInt(),da=k.substring(8,10).toInt();if(mo<1||mo>12||da<1)continue;int doy=da-1;for(int m=1;m<mo;++m)doy+=mdays[m-1]+(m==2&&leap()?1:0);int w=(offset+doy)/7,d=(offset+doy)%7;fillRect(18+w*(cell+gap),y0+d*(cell+gap),cell,cell,ink());++marked;}
  drawText((String("DIAS MARCADOS: ")+String(marked)).c_str(),18,340,1,ink());drawText("OK MARCA HOY  BACK VOLVER",18,756,1,ink());refreshDisplay(false);
}
static void renderGymFolder(){
  clearCanvas();drawHeader();JsonArray fs=folders();if(openA<0||openA>=(int)fs.size()){view=View::Root;renderRoot();return;}JsonObject f=fs[openA].as<JsonObject>();
  textClipT(safeText(f,"name"),18,184,2,470);JsonArray es=f["exercises"].as<JsonArray>();int n=es.size();int count=n+1+(editing?1:0);subCursor=clampCursor(subCursor,count);
  int start=(subCursor/10)*10,row=0;for(int i=start;i<count&&row<10;++i,++row){bool sel=subCursor==i;if(i<n){JsonObject e=es[i].as<JsonObject>();String r=String(e["sets"]|0)+"X"+String(e["reps"]|0)+" "+String(e["kg"]|0)+"KG";drawRow(row,safeText(e,"name"),sel,false,false,r);}else if(i==n)drawActionRow(row,"+ AGREGAR EJERCICIO",sel);else drawActionRow(row,"AJUSTES",sel);}
  drawText("OK EJERCICIO  BACK VOLVER",18,756,1,ink());refreshDisplay(false);
}

static void renderGymExercise(){
  clearCanvas();drawHeader();JsonArray fs=folders();if(openA<0||openA>=(int)fs.size())return;JsonArray es=fs[openA]["exercises"].as<JsonArray>();if(openB<0||openB>=(int)es.size())return;JsonObject e=es[openB].as<JsonObject>();normalizeExerciseDone(e);
  textClipT(safeText(e,"name"),18,184,2,470);String fields[3]={String("SETS  ")+String(e["sets"]|0),String("REPS  ")+String(e["reps"]|0),String("KG    ")+String(e["kg"]|0)};
  int sets=max(0,e["sets"]|0);int count=3+sets;subCursor=clampCursor(subCursor,count);for(int i=0;i<3;++i)drawRow(i,fields[i],subCursor==i,false,false,subCursor==i?"OK RUEDA":"");
  JsonArray d=e["done"].as<JsonArray>();int base=3;for(int s=0;s<sets&&s<8;++s)drawRow(base+s,String("SET ")+String(s+1),subCursor==base+s,true,d[s]|false,"");
  drawText("UP/DN FOCO  L/R +/-  OK  BACK",18,756,1,ink());refreshDisplay(false);
}

struct ProjectRow {int kind=0;int fi=-1;int si=-1;}; // 1 folder,2 step,3 addstep,4 addfolder
static int projectRows(int pi){JsonArray ps=projects();if(pi<0||pi>=(int)ps.size())return 0;int n=0;for(JsonObject f:ps[pi]["folders"].as<JsonArray>()){++n;if(!(f["collapsed"]|false)){n+=f["steps"].as<JsonArray>().size();++n;}}return n+1+(editing?1:0);}
static ProjectRow projectRowAt(int pi,int idx){JsonArray ps=projects();if(pi<0||pi>=(int)ps.size())return {};int n=0,fi=0;for(JsonObject f:ps[pi]["folders"].as<JsonArray>()){if(n++==idx)return{1,fi,-1};if(!(f["collapsed"]|false)){int si=0;for(JsonObject st:f["steps"].as<JsonArray>()){(void)st;if(n++==idx)return{2,fi,si};++si;}if(n++==idx)return{3,fi,-1};}++fi;}if(n++==idx)return{4,-1,-1};return{5,-1,-1};}

static void renderProject(){
  clearCanvas();drawHeader();JsonArray ps=projects();if(openA<0||openA>=(int)ps.size())return;JsonObject p=ps[openA].as<JsonObject>();textClipT(safeText(p,"name"),18,184,2,470);
  int count=projectRows(openA);subCursor=clampCursor(subCursor,count);int start=(subCursor/10)*10,row=0;for(int i=start;i<count&&row<10;++i,++row){ProjectRow r=projectRowAt(openA,i);bool sel=subCursor==i;if(r.kind==1){JsonObject f=p["folders"][r.fi].as<JsonObject>();drawRow(row,String(f["collapsed"]|false?"[+] ":"[-] ")+safeText(f,"name"),sel,false,false,"CARP");}
    else if(r.kind==2){JsonObject st=p["folders"][r.fi]["steps"][r.si].as<JsonObject>();drawRow(row,String("  ")+safeText(st,"text"),sel,true,st["done"]|false,"");}
    else if(r.kind==3)drawActionRow(row,"  + AGREGAR PASO",sel);else if(r.kind==4)drawActionRow(row,"+ AGREGAR CARPETA",sel);else drawActionRow(row,"AJUSTES",sel);}
  drawText("OK MARCAR/ABRIR  BACK",18,756,1,ink());refreshDisplay(false);
}

static bool exportNotebookTxt(int ni){
  if(!storageReady)return false;JsonArray ns=notebooks();if(ni<0||ni>=(int)ns.size())return false;SdMan.ensureDirectoryExists(EXPORT_DIR);JsonObject n=ns[ni].as<JsonObject>();String name=safeText(n,"name","libreta");String safe;for(size_t i=0;i<name.length();++i){char c=name[i];if((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_')safe+=c;else if(c==' ')safe+='_';}if(!safe.length())safe="libreta";String path=String(EXPORT_DIR)+"/"+safe+".txt";FsFile f=SdMan.open(path.c_str(),O_WRITE|O_CREAT|O_TRUNC);if(!f)return false;String txt=safeText(n,"text");size_t w=f.write((const uint8_t*)txt.c_str(),txt.length());f.flush();f.close();messageText=String("GUARDADO: ")+path;return w==txt.length();
}

static void renderNotebook(){
  clearCanvas();drawHeader();JsonArray ns=notebooks();if(openA<0||openA>=(int)ns.size())return;JsonObject n=ns[openA].as<JsonObject>();textClipT(safeText(n,"name"),18,184,2,470);
  drawRow(0,"EDITAR TEXTO",subCursor==0);drawRow(1,"TAGS",subCursor==1,false,false,String(n["tags"].as<JsonArray>().size()));drawRow(2,"EXPORTAR TXT",subCursor==2);
  String txt=asciiUpper(safeText(n,"text"));txt.replace("\r","");int charsPer=42*14;int pages=max(1,(int)((txt.length()+charsPer-1)/charsPer));int pg=constrain(openB,0,pages-1);openB=pg;int pos=pg*charsPer,y=365;for(int l=0;l<14&&pos<(int)txt.length();++l){String part=txt.substring(pos,min(pos+42,(int)txt.length()));drawTextRaw(part,20,y,1,ink());y+=22;pos+=part.length();}
  drawText((String("PAG ")+String(pg+1)+"/"+String(pages)).c_str(),400,756,1,ink());drawText("L/R PAG  OK ACCION  BACK",18,756,1,ink());refreshDisplay(false);
}

static void renderNotebookTags(){
  clearCanvas();drawHeader();JsonArray ns=notebooks();if(openA<0||openA>=(int)ns.size())return;JsonObject n=ns[openA].as<JsonObject>();textT("TAGS",18,184,2);if(!n["tags"].is<JsonArray>())n["tags"].to<JsonArray>();JsonArray ta=n["tags"].as<JsonArray>();int count=ta.size()+1;subCursor=clampCursor(subCursor,count);int start=(subCursor/10)*10;for(int r=0;r<10&&start+r<count;++r){int i=start+r;if(i<(int)ta.size())drawRow(r,String((const char*)(ta[i]|"")),subCursor==i,false,false,"OK BORRA");else drawRow(r,"+ AGREGAR TAG",subCursor==i);}drawText("OK BORRAR/AGREGAR  BACK",18,756,1,ink());refreshDisplay(false);
}
static void renderGrimorio(){
  clearCanvas();drawHeader();textT("GRIMORIO",18,184,2); // Unique tags + untagged notebooks.
  String tags[30];int tc=0;for(JsonObject n:notebooks())for(const char* t:n["tags"].as<JsonArray>()){bool ex=false;for(int i=0;i<tc;++i)if(tags[i]==t)ex=true;if(!ex&&tc<30)tags[tc++]=t;}
  int count=tc+1;subCursor=clampCursor(subCursor,count);int start=(subCursor/10)*10;for(int r=0;r<10&&start+r<count;++r){int i=start+r;if(i<tc)drawRow(r,tags[i],subCursor==i,false,false,"TAG");else drawRow(r,"SIN ETIQUETA",subCursor==i,false,false,"");}
  drawText("OK VER  BACK VOLVER",18,756,1,ink());refreshDisplay(false);
}

static void renderSettings(){
  clearCanvas();drawHeader();textT("AJUSTES / EDICION",18,184,2);int count=8+(page==Page::Gym?1:0);cursor=clampCursor(cursor,count);
  static const char* names[]={"NOMBRE HOY","NOMBRE HABITOS","NOMBRE TAREAS","NOMBRE EJERCICIOS","NOMBRE PROYECTOS","NOMBRE ESCRIBIR"};
  for(int i=0;i<6;++i)drawRow(i,names[i],cursor==i,false,false,asciiUpper(stateDoc["labels"][pageKey((Page)i)]|labelDefault(i)));
  drawRow(6,"MODO OSCURO",cursor==6,true,darkMode,"");drawRow(7,"TERMINAR EDICION",cursor==7,false,false,editing?"SI":"NO");if(page==Page::Gym)drawRow(8,"DESMARCAR TODOS LOS SETS",cursor==8,false,false,"");
  drawText("OK CAMBIAR  BACK VOLVER",18,756,1,ink());refreshDisplay(false);
}

static void renderEditMenu(){
  clearCanvas();drawHeader();JsonObject o=objectAt(editKind,editA,editB,editC);String title=asciiUpper(safeText(o,editTextKey(),"ELEMENTO"));textClipT(title.c_str(),18,184,2,470);
  const char* ops[]={"RENOMBRAR","SUBIR","BAJAR","ELIMINAR","CANCELAR"};cursor=clampCursor(cursor,5);for(int i=0;i<5;++i)drawRow(i,ops[i],cursor==i);drawText("OK ELEGIR  BACK CANCELAR",18,756,1,ink());refreshDisplay(false);
}

static const char* keyboardKeys[] = {
  "A","B","C","D","E","F",
  "G","H","I","J","K","L",
  "M","N","O","P","Q","R",
  "S","T","U","V","W","X",
  "Y","Z","0","1","2","3",
  "4","5","6","7","8","9",
  "ESP","DEL",".",",","NL","OK"
};
static constexpr int KEY_COUNT=48, KEY_COLS=6;

static void renderTextInput(){
  clearCanvas();drawHeader();textClipT(inputTitle.c_str(),18,184,2,470);String shown=asciiUpper(inputBuffer.c_str());if(shown.length()>120)shown=String("...")+shown.substring(shown.length()-117);drawTextClipped(shown.c_str(),18,220,1,490,ink());lineT(18,250,492,1);
  for(int i=0;i<KEY_COUNT;++i){int col=i%KEY_COLS,row=i/KEY_COLS;int x=18+col*82,y=280+row*50;bool sel=keyboardIndex==i;if(sel)fillRect(x,y,76,40,ink());else borderT(x,y,76,40,1);String k=keyboardKeys[i];drawTextRaw(k,x+(76-textWidth(k,1))/2,y+14,1,sel?paper():ink());}
  drawText("FLECHAS TECLADO  OK LETRA  BACK BORRA",18,740,1,ink());refreshDisplay(false);
}

static void renderNumberInput(){
  clearCanvas();drawHeader();textT("RUEDA NUMERICA",18,184,2);int tens=numberValue/10,units=numberValue%10;int vals[2]={tens,units};for(int d=0;d<2;++d){int x=145+d*140;for(int off=-2;off<=2;++off){int v=(vals[d]+off+10)%10;String s=String(v);int y=270+(off+2)*60;drawTextRaw(s,x,y,off==0?4:2,ink());}if(numberDigit==d)borderT(x-30,370,90,70,2);}drawText((String("VALOR: ")+String(numberValue)).c_str(),190,600,2,ink());drawText("L/R DIGITO  UP/DN GIRA  OK GUARDAR",18,756,1,ink());refreshDisplay(false);
}

static void renderTransfer(){clearCanvas();drawSparkle(logicalWidth()/2,90,true);textT("TRANSFERIR MAPLE",150,155,2);const char* a[]={"EXPORTAR A SD","IMPORTAR DESDE SD","CERRAR"};cursor=clampCursor(cursor,3);for(int i=0;i<3;++i)drawRow(i,a[i],cursor==i);drawText("OK ELEGIR  BACK CERRAR",18,756,1,ink());refreshDisplay(false);}
static void renderImportPicker(){clearCanvas();textT("IMPORTAR JSON DESDE SD",18,50,2);cursor=clampCursor(cursor,max(1,importFileCount));if(!importFileCount){drawText("NO HAY ARCHIVOS .JSON",18,140,2,ink());drawText("COPIALO A RAIZ O /MAPLE/IMPORT",18,190,1,ink());}else{int start=(cursor/11)*11;for(int r=0;r<11&&start+r<importFileCount;++r){int i=start+r;drawRow(r,importFiles[i],cursor==i);}}drawText("OK IMPORTAR  BACK VOLVER",18,756,1,ink());refreshDisplay(false);}
static void renderExportSelect(){clearCanvas();textT("EXPORTAR",18,50,2);const char* n[]={"HABITOS","TAREAS","EJERCICIOS","PROYECTOS","ESCRIBIR","GUARDAR JSON EN SD"};cursor=clampCursor(cursor,6);for(int i=0;i<6;++i)drawRow(i,n[i],cursor==i,i<5,i<5?exportSel[i]:false,i==5?"/MAPLE/EXPORTS":"");drawText("OK MARCAR/GUARDAR  BACK",18,756,1,ink());refreshDisplay(false);}

static void renderRecurFreq(){clearCanvas();textT("TAREA RECURRENTE",18,50,2);textClipT(recurText.c_str(),18,90,2,470);const char* f[]={"DIARIO","SEMANAL","MENSUAL","ANUAL"};cursor=clampCursor(cursor,4);for(int i=0;i<4;++i)drawRow(i,f[i],cursor==i);drawText("OK ELEGIR  BACK",18,756,1,ink());refreshDisplay(false);}
static void renderRecurWeek(){clearCanvas();textT("DIAS DE LA SEMANA",18,50,2);const char* d[]={"LUNES","MARTES","MIERCOLES","JUEVES","VIERNES","SABADO","DOMINGO","GUARDAR"};cursor=clampCursor(cursor,8);for(int i=0;i<8;++i)drawRow(i,d[i],cursor==i,i<7,i<7?recurWeek[i]:false);drawText("OK MARCAR  GUARDAR AL FINAL",18,756,1,ink());refreshDisplay(false);}
static void renderRecurMonth(){clearCanvas();textT("DIAS DEL MES",18,50,2);cursor=clampCursor(cursor,32);int start=(cursor/11)*11;for(int r=0;r<11&&start+r<32;++r){int i=start+r;if(i<31)drawRow(r,String("DIA ")+String(i+1),cursor==i,true,recurMonth[i],"");else drawRow(r,"GUARDAR",cursor==i);}drawText("OK MARCAR  BACK",18,756,1,ink());refreshDisplay(false);}
static void renderRecurYear(){clearCanvas();textT("FECHAS ANUALES",18,50,2);String months[]={"ENE","FEB","MAR","ABR","MAY","JUN","JUL","AGO","SEP","OCT","NOV","DIC"};int days=31;cursor=clampCursor(cursor,days+2);drawText((String("MES: ")+months[recurPickMonth]+"  L/R CAMBIA MES").c_str(),18,90,2,ink());int start=(cursor/10)*10;for(int r=0;r<10&&start+r<days+2;++r){int i=start+r;if(i<31)drawRow(r,String("DIA ")+String(i+1),cursor==i,true,recurYear[recurPickMonth][i],"");else if(i==31)drawRow(r,"CAMBIAR MES",cursor==i);else drawRow(r,"GUARDAR",cursor==i);}drawText("OK MARCAR  BACK",18,756,1,ink());refreshDisplay(false);}
static void renderMessage(){clearCanvas();textT("MAPLE X3",18,80,3);drawTextClipped(messageText.c_str(),18,220,2,490,ink());drawText("OK / BACK CONTINUAR",18,756,1,ink());refreshDisplay(true);}

static void render(){
  switch(view){case View::Root:renderRoot();break;case View::HabitYear:renderHabitYear();break;case View::GymFolder:renderGymFolder();break;case View::GymExercise:renderGymExercise();break;case View::Project:renderProject();break;case View::Notebook:renderNotebook();break;case View::NotebookTags:renderNotebookTags();break;case View::Grimorio:renderGrimorio();break;case View::Settings:renderSettings();break;case View::EditMenu:renderEditMenu();break;case View::TextInput:renderTextInput();break;case View::NumberInput:renderNumberInput();break;case View::Transfer:renderTransfer();break;case View::ImportPicker:renderImportPicker();break;case View::ExportSelect:renderExportSelect();break;case View::RecurFreq:renderRecurFreq();break;case View::RecurWeek:renderRecurWeek();break;case View::RecurMonth:renderRecurMonth();break;case View::RecurYear:renderRecurYear();break;case View::Message:renderMessage();break;}
}

// -----------------------------------------------------------------------------
// Input actions
// -----------------------------------------------------------------------------
static void goPage(Page p){page=p;view=View::Root;cursor=0;subCursor=0;openA=openB=openC=-1;}

static void createRecurringRule() {
  JsonObject r=recurring().add<JsonObject>();r["id"]=nextId();r["text"]=recurText;r["lastApplied"]="";
  if(recurFreq==0)r["freq"]="daily";else if(recurFreq==1)r["freq"]="weekly";else if(recurFreq==2)r["freq"]="monthly";else r["freq"]="yearly";
  JsonArray w=r["weekdays"].to<JsonArray>();if(recurFreq==1)for(int i=0;i<7;++i)if(recurWeek[i])w.add(i);
  JsonArray m=r["monthdays"].to<JsonArray>();if(recurFreq==2)for(int i=0;i<31;++i)if(recurMonth[i])m.add(i+1);
  JsonArray y=r["yeardays"].to<JsonArray>();if(recurFreq==3)for(int mo=0;mo<12;++mo)for(int d=0;d<31;++d)if(recurYear[mo][d]){char b[8];snprintf(b,sizeof(b),"%02d-%02d",mo+1,d+1);y.add(b);}
  applyRecurring();persistState();goPage(Page::Tasks);
}

static void handleRootConfirm(){
  if(cursor==-1){editing=!editing;persistState();render();return;}
  int n;
  if(page==Page::Today){int tn=pendingTaskCount(),hn=habits().size(),pn=pendingProjectCount();if(cursor<tn)toggleTask(pendingTaskAt(cursor));else if(cursor<tn+hn)toggleHabit(habits()[cursor-tn].as<JsonObject>());else if(cursor<tn+hn+pn){ /* Maple v8 muestra el siguiente paso en Hoy sin modificarlo */ }else{view=View::Settings;cursor=0;}render();return;}
  if(page==Page::Habits){n=habits().size();if(cursor<n){if(editing)openEdit(EditKind::Habit,cursor,-1,-1,View::Root);else{openA=cursor;view=View::HabitYear;}}else if(cursor==n)startText(TextTarget::AddHabit,"","NUEVO HABITO",View::Root);else{view=View::Settings;cursor=0;}render();return;}
  if(page==Page::Tasks){n=tasks().size();if(cursor<n){if(editing)openEdit(EditKind::Task,cursor,-1,-1,View::Root);else toggleTask(tasks()[cursor].as<JsonObject>());}else if(cursor==n)startText(TextTarget::AddTask,"","NUEVA TAREA",View::Root);else if(cursor==n+1){recurText="";for(bool&x:recurWeek)x=false;for(bool&x:recurMonth)x=false;memset(recurYear,0,sizeof(recurYear));startText(TextTarget::RecurringText,"","TAREA RECURRENTE",View::Root);}else{view=View::Settings;cursor=0;}render();return;}
  if(page==Page::Gym){n=folders().size();if(cursor<n){if(editing)openEdit(EditKind::GymFolder,cursor,-1,-1,View::Root);else{openA=cursor;subCursor=0;view=View::GymFolder;}}else if(cursor==n)startText(TextTarget::AddGymFolder,"","NUEVA CARPETA",View::Root);else{view=View::Settings;cursor=0;}render();return;}
  if(page==Page::Projects){n=projects().size();if(cursor<n){if(editing)openEdit(EditKind::Project,cursor,-1,-1,View::Root);else{openA=cursor;subCursor=0;view=View::Project;}}else if(cursor==n)startText(TextTarget::AddProject,"","NUEVO PROYECTO",View::Root);else{view=View::Settings;cursor=0;}render();return;}
  if(page==Page::Write){n=notebooks().size();if(cursor==0){subCursor=0;view=View::Grimorio;}else if(cursor<=n){int ni=cursor-1;if(editing)openEdit(EditKind::Notebook,ni,-1,-1,View::Root);else{openA=ni;openB=0;subCursor=0;view=View::Notebook;}}else if(cursor==n+1)startText(TextTarget::AddNotebook,"","NUEVA LIBRETA",View::Root);else{view=View::Settings;cursor=0;}render();return;}
}

static void handleEditMenuConfirm(){
  if(cursor==4){view=returnView;render();return;}JsonArray arr=arrayForEdit(editKind,editA,editB);int idx=(editKind==EditKind::Exercise||editKind==EditKind::ProjectFolder)?editB:(editKind==EditKind::ProjectStep?editC:editA);
  if(cursor==0){JsonObject o=objectAt(editKind,editA,editB,editC);TextTarget rt=renameTargetForEdit();String init=safeText(o,editTextKey());startText(rt,init,"RENOMBRAR",returnView,editA,editB,editC);render();return;}
  if(cursor==1||cursor==2){int j=idx+(cursor==1?-1:1);if(!arr.isNull()&&j>=0&&j<(int)arr.size()){swapArrayItems(arr,idx,j);persistState();if(editKind==EditKind::Exercise)editB=j;else if(editKind==EditKind::ProjectFolder)editB=j;else if(editKind==EditKind::ProjectStep)editC=j;else editA=j;}view=returnView;render();return;}
  if(cursor==3){deleteEditItem();render();return;}
}

static void handleTextShortConfirm(){
  const char* k=keyboardKeys[keyboardIndex];
  if(!strcmp(k,"ESP"))inputBuffer+=" ";else if(!strcmp(k,"DEL")){if(inputBuffer.length())inputBuffer.remove(inputBuffer.length()-1);}else if(!strcmp(k,"NL"))inputBuffer+="\n";else if(!strcmp(k,"OK")){commitText();render();return;}else inputBuffer+=k;
  render();
}

static void handleShortConfirm(){
  if(view==View::Root){handleRootConfirm();return;}
  if(view==View::HabitYear){toggleHabit(habits()[openA].as<JsonObject>());render();return;}
  if(view==View::GymFolder){JsonArray es=folders()[openA]["exercises"].as<JsonArray>();int n=es.size();if(subCursor<n){if(editing)openEdit(EditKind::Exercise,openA,subCursor,-1,View::GymFolder);else{openB=subCursor;subCursor=0;view=View::GymExercise;}}else if(subCursor==n)startText(TextTarget::AddExercise,"","NUEVO EJERCICIO",View::GymFolder,openA);else{view=View::Settings;cursor=0;}render();return;}
  if(view==View::GymExercise){JsonObject e=folders()[openA]["exercises"][openB].as<JsonObject>();if(subCursor==0)startNumber(NumberTarget::ExerciseSets,e["sets"]|0,View::GymExercise);else if(subCursor==1)startNumber(NumberTarget::ExerciseReps,e["reps"]|0,View::GymExercise);else if(subCursor==2)startNumber(NumberTarget::ExerciseKg,e["kg"]|0,View::GymExercise);else{normalizeExerciseDone(e);JsonArray d=e["done"].as<JsonArray>();int i=subCursor-3;if(i>=0&&i<(int)d.size())d[i]=!(d[i]|false);persistState();}render();return;}
  if(view==View::Project){ProjectRow r=projectRowAt(openA,subCursor);JsonObject p=projects()[openA].as<JsonObject>();if(r.kind==1){if(editing)openEdit(EditKind::ProjectFolder,openA,r.fi,-1,View::Project);else p["folders"][r.fi]["collapsed"]=!(p["folders"][r.fi]["collapsed"]|false),persistState();}else if(r.kind==2){if(editing)openEdit(EditKind::ProjectStep,openA,r.fi,r.si,View::Project);else{JsonObject st=p["folders"][r.fi]["steps"][r.si].as<JsonObject>();st["done"]=!(st["done"]|false);persistState();}}else if(r.kind==3)startText(TextTarget::AddProjectStep,"","NUEVO PASO",View::Project,openA,r.fi);else if(r.kind==4)startText(TextTarget::AddProjectFolder,"","NUEVA CARPETA",View::Project,openA);else{view=View::Settings;cursor=0;}render();return;}
  if(view==View::Notebook){if(subCursor==0){JsonObject n=notebooks()[openA].as<JsonObject>();startText(TextTarget::NotebookText,String((const char*)(n["text"]|"")),"EDITAR LIBRETA",View::Notebook,openA);}else if(subCursor==1){subCursor=0;view=View::NotebookTags;}else{bool ok=exportNotebookTxt(openA);showMessage(ok?messageText:"NO SE PUDO EXPORTAR",View::Notebook);}render();return;}
  if(view==View::NotebookTags){JsonObject n=notebooks()[openA].as<JsonObject>();JsonArray ta=n["tags"].as<JsonArray>();if(subCursor<(int)ta.size()){ta.remove(subCursor);persistState();subCursor=clampCursor(subCursor,ta.size()+1);}else startText(TextTarget::AddNotebookTag,"","AGREGAR TAG",View::NotebookTags,openA);render();return;}
  if(view==View::Grimorio){ // Selecting a tag opens the first matching notebook; keeps navigation simple on physical buttons.
    String tags[30];int tc=0;for(JsonObject n:notebooks())for(const char* t:n["tags"].as<JsonArray>()){bool ex=false;for(int i=0;i<tc;++i)if(tags[i]==t)ex=true;if(!ex&&tc<30)tags[tc++]=t;}
    if(subCursor<tc){for(int i=0;i<(int)notebooks().size();++i){for(const char* t:notebooks()[i]["tags"].as<JsonArray>())if(tags[subCursor]==t){openA=i;openB=0;subCursor=0;view=View::Notebook;render();return;}}}else{for(int i=0;i<(int)notebooks().size();++i)if(notebooks()[i]["tags"].as<JsonArray>().size()==0){openA=i;openB=0;subCursor=0;view=View::Notebook;render();return;}}
    render();return;}
  if(view==View::Settings){if(cursor<6){String init=stateDoc["labels"][pageKey((Page)cursor)]|labelDefault(cursor);startText(TextTarget::RenameTab,init,"NOMBRE DE PESTANA",View::Settings,cursor);}else if(cursor==6){darkMode=!darkMode;persistState();}else if(cursor==7){editing=!editing;persistState();view=View::Root;cursor=0;}else{for(JsonObject f:folders())for(JsonObject e:f["exercises"].as<JsonArray>())e["done"].to<JsonArray>();persistState();view=View::Root;cursor=0;}render();return;}
  if(view==View::EditMenu){handleEditMenuConfirm();return;}
  if(view==View::TextInput){handleTextShortConfirm();return;}
  if(view==View::NumberInput){commitNumber();render();return;}
  if(view==View::Transfer){if(cursor==0){cursor=0;view=View::ExportSelect;}else if(cursor==1){scanImportFiles();cursor=0;view=View::ImportPicker;}else{view=View::Root;cursor=0;}render();return;}
  if(view==View::ImportPicker){if(importFileCount&&cursor<importFileCount){String p=importFiles[cursor];bool ok=importPackageFile(p);showMessage(ok?String("IMPORTADO: ")+p:String("ERROR LEYENDO: ")+p,View::Root);}render();return;}
  if(view==View::ExportSelect){if(cursor<5)exportSel[cursor]=!exportSel[cursor];else{bool ok=exportPackage();showMessage(ok?messageText:"NO SE PUDO EXPORTAR",View::Root);}render();return;}
  if(view==View::RecurFreq){recurFreq=cursor;if(recurFreq==0){createRecurringRule();}else if(recurFreq==1){cursor=0;view=View::RecurWeek;}else if(recurFreq==2){cursor=0;view=View::RecurMonth;}else{cursor=0;recurPickMonth=0;view=View::RecurYear;}render();return;}
  if(view==View::RecurWeek){if(cursor<7)recurWeek[cursor]=!recurWeek[cursor];else createRecurringRule();render();return;}
  if(view==View::RecurMonth){if(cursor<31)recurMonth[cursor]=!recurMonth[cursor];else createRecurringRule();render();return;}
  if(view==View::RecurYear){if(cursor<31)recurYear[recurPickMonth][cursor]=!recurYear[recurPickMonth][cursor];else if(cursor==31)recurPickMonth=(recurPickMonth+1)%12;else createRecurringRule();render();return;}
  if(view==View::Message){view=messageReturn;cursor=0;render();return;}
}

static void handleLongConfirm(){
  if(view==View::Root){view=View::Transfer;cursor=0;render();return;}
  if(view==View::TextInput){commitText();render();return;}
}

static void handleBack(){
  if(view==View::Root){goPage(Page::Today);render();return;}if(view==View::NotebookTags){view=View::Notebook;subCursor=1;render();return;}if(view==View::GymExercise){view=View::GymFolder;subCursor=openB;render();return;}if(view==View::GymFolder||view==View::HabitYear||view==View::Project||view==View::Grimorio){view=View::Root;cursor=max(0,openA);render();return;}if(view==View::Notebook){view=View::Root;cursor=openA+1;render();return;}
  if(view==View::TextInput){if(inputBuffer.length())inputBuffer.remove(inputBuffer.length()-1);else view=returnView;render();return;}if(view==View::NumberInput){view=returnView;render();return;}if(view==View::EditMenu||view==View::Settings){view=returnView;render();return;}if(view==View::Transfer){view=View::Root;cursor=0;render();return;}if(view==View::ImportPicker||view==View::ExportSelect){view=View::Transfer;cursor=0;render();return;}if(view==View::RecurFreq||view==View::RecurWeek||view==View::RecurMonth||view==View::RecurYear){goPage(Page::Tasks);render();return;}if(view==View::Message){view=messageReturn;render();return;}
}

static void adjustCurrent(int delta){
  if(view==View::Root){int n=rootCount();cursor=constrain(cursor+delta,-1,max(-1,n-1));}
  else if(view==View::GymFolder){int n=folders()[openA]["exercises"].as<JsonArray>().size()+1+(editing?1:0);subCursor=clampCursor(subCursor+delta,n);}
  else if(view==View::GymExercise){JsonObject e=folders()[openA]["exercises"][openB].as<JsonObject>();subCursor=clampCursor(subCursor+delta,3+max(0,e["sets"]|0));}
  else if(view==View::Project)subCursor=clampCursor(subCursor+delta,projectRows(openA));
  else if(view==View::Notebook)subCursor=clampCursor(subCursor+delta,3);else if(view==View::NotebookTags){JsonArray ta=notebooks()[openA]["tags"].as<JsonArray>();subCursor=clampCursor(subCursor+delta,ta.size()+1);}
  else if(view==View::Grimorio){String tags[30];int tc=0;for(JsonObject n:notebooks())for(const char* t:n["tags"].as<JsonArray>()){bool ex=false;for(int i=0;i<tc;++i)if(tags[i]==t)ex=true;if(!ex&&tc<30)tags[tc++]=t;}subCursor=clampCursor(subCursor+delta,tc+1);}
  else if(view==View::Settings)cursor=clampCursor(cursor+delta,8+(page==Page::Gym?1:0));else if(view==View::EditMenu)cursor=clampCursor(cursor+delta,5);else if(view==View::Transfer)cursor=clampCursor(cursor+delta,3);else if(view==View::ImportPicker)cursor=clampCursor(cursor+delta,max(1,importFileCount));else if(view==View::ExportSelect)cursor=clampCursor(cursor+delta,6);else if(view==View::RecurFreq)cursor=clampCursor(cursor+delta,4);else if(view==View::RecurWeek)cursor=clampCursor(cursor+delta,8);else if(view==View::RecurMonth)cursor=clampCursor(cursor+delta,32);else if(view==View::RecurYear)cursor=clampCursor(cursor+delta,33);
  else if(view==View::TextInput){int row=keyboardIndex/KEY_COLS,col=keyboardIndex%KEY_COLS;row=constrain(row+delta,0,(KEY_COUNT-1)/KEY_COLS);keyboardIndex=min(KEY_COUNT-1,row*KEY_COLS+col);}
  else if(view==View::NumberInput){int tens=numberValue/10,units=numberValue%10;if(numberDigit==0)tens=(tens+delta+10)%10;else units=(units+delta+10)%10;numberValue=tens*10+units;}
  render();
}

static void handleLeftRight(int delta){
  if(view==View::Root){int p=(int)page;goPage((Page)((p+delta+6)%6));render();return;}
  if(view==View::GymExercise){JsonObject e=folders()[openA]["exercises"][openB].as<JsonObject>();if(subCursor<3){const char* key=subCursor==0?"sets":(subCursor==1?"reps":"kg");int v=e[key]|0;e[key]=max(0,v+delta);if(subCursor==0)normalizeExerciseDone(e);persistState();render();return;}}
  if(view==View::Notebook){String txt=safeText(notebooks()[openA],"text");int pages=max(1,(int)((txt.length()+42*17-1)/(42*17)));openB=constrain(openB+delta,0,pages-1);render();return;}
  if(view==View::TextInput){int row=keyboardIndex/KEY_COLS,col=keyboardIndex%KEY_COLS;col=(col+delta+KEY_COLS)%KEY_COLS;keyboardIndex=min(KEY_COUNT-1,row*KEY_COLS+col);render();return;}
  if(view==View::NumberInput){numberDigit=constrain(numberDigit+delta,0,1);render();return;}
  if(view==View::RecurYear){recurPickMonth=(recurPickMonth+delta+12)%12;render();return;}
}

bool confirmPending=false;bool confirmLongFired=false;uint32_t confirmStarted=0;
static void queueConfirm(){if(confirmPending)return;confirmPending=true;confirmLongFired=false;confirmStarted=millis();}
static void serviceConfirmHold(){
  if(!confirmPending)return;bool held=input.isPressed(InputManager::BTN_CONFIRM);uint32_t age=millis()-confirmStarted;
  if(held&&age>=800&&!confirmLongFired){confirmLongFired=true;confirmPending=false;handleLongConfirm();return;}
  if(!held){confirmPending=false;if(!confirmLongFired)handleShortConfirm();}
}

static void handleButton(uint8_t b){
  if(b==InputManager::BTN_CONFIRM){if(view==View::Root||view==View::TextInput)queueConfirm();else handleShortConfirm();return;}
  if(b==InputManager::BTN_BACK){handleBack();return;}if(b==InputManager::BTN_UP){adjustCurrent(-1);return;}if(b==InputManager::BTN_DOWN){adjustCurrent(1);return;}if(b==InputManager::BTN_LEFT){handleLeftRight(-1);return;}if(b==InputManager::BTN_RIGHT){handleLeftRight(1);return;}
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);delay(250);Serial.printf("Maple X3 v%s standalone booting...\\n",FW_VERSION);

  // El X3 comparte el bus SPI entre la pantalla y la microSD.
  // Inicializamos la SD ANTES de display.begin() y mantenemos el CS
  // de la pantalla en HIGH para que no interfiera durante la detección.
  BoardConfig::holdPowerRails();
  display.setDisplayX3();
  BoardConfig::releaseSdRail();

  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);
  delay(100);

  storageReady=SdMan.begin();

  // Una vez montada la SD, inicializamos la pantalla.
  freeink::applyXteinkDisplayController();
  display.begin();

  if(storageReady){
    SdMan.ensureDirectoryExists(MAPLE_DIR);
    SdMan.ensureDirectoryExists(EXPORT_DIR);
    SdMan.ensureDirectoryExists(IMPORT_DIR);
    loadState();
  }else{
    newStore();
  }

  if(baseEpochMs<1577836800000ULL)baseEpochMs=compileEpochMs();baseMillis=millis();ensureStore();applyRecurring();purgeCompletedAfter2am();
  input.begin();input.beginAsync(2,15,32);
  if(!storageReady){showMessage("SD NO DISPONIBLE. MAPLE FUNCIONA EN RAM; IMPORTAR/EXPORTAR REQUIERE SD.",View::Root);}render();
}

void loop(){
  uint8_t b=0;while(input.popPress(b))handleButton(b);serviceConfirmHold();
  static uint32_t lastMinute=0;if(millis()-lastMinute>60000){lastMinute=millis();applyRecurring();purgeCompletedAfter2am();}
  delay(2);
}
