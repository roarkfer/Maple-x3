#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <InputManager.h>
#include <SDCardManager.h>
#include <XteinkDetect.h>
#include <esp_system.h>

// Maple X3 — SD diagnostic
// Purpose: isolate SD power / SPI / FAT issues without changing Maple v8.
// XTEINK X3 verified profile from pinned FreeInk SDK:
//   SD: SCLK=8, MOSI=10, MISO=7, CS=12, POWER=13 active-high
//   EPD CS=21; the SD and display share the SPI bus.

static constexpr const char* DIAG_VERSION = "1.0.0";

static constexpr int EPD_SCLK = 8;
static constexpr int EPD_MOSI = 10;
static constexpr int EPD_CS   = 21;
static constexpr int EPD_DC   = 4;
static constexpr int EPD_RST  = 5;
static constexpr int EPD_BUSY = 6;

static constexpr int SD_SCLK  = 8;
static constexpr int SD_MOSI  = 10;
static constexpr int SD_MISO  = 7;
static constexpr int SD_CS    = 12;
static constexpr int SD_PWR   = 13;

EInkDisplay display(EPD_SCLK, EPD_MOSI, EPD_CS, EPD_DC, EPD_RST, EPD_BUSY);
InputManager input;

static bool managerOk = false;
static bool rawOk = false;
static bool writeOk = false;
static bool readOk = false;
static uint32_t rawHz = 0;
static uint8_t lastErrCode = 0;
static uint8_t lastErrData = 0;
static uint64_t totalBytes = 0;
static String filesFound[8];
static int filesFoundCount = 0;
static SdFat* rawSd = nullptr;

// -----------------------------------------------------------------------------
// Minimal portrait drawing helpers (same framebuffer orientation proven on X3)
// -----------------------------------------------------------------------------

static void pixelPhysical(int x, int y, bool black) {
  if (x < 0 || y < 0 || x >= display.getDisplayWidth() || y >= display.getDisplayHeight()) return;
  uint8_t* fb = display.getFrameBuffer();
  const uint16_t rowBytes = display.getDisplayWidthBytes();
  const size_t index = static_cast<size_t>(y) * rowBytes + (x >> 3);
  const uint8_t mask = 0x80 >> (x & 7);
  if (black) fb[index] &= ~mask;
  else fb[index] |= mask;
}

static void pixel(int x, int y, bool black = true) {
  const int px = y;
  const int py = display.getDisplayHeight() - 1 - x;
  pixelPhysical(px, py, black);
}

static void fillRect(int x, int y, int w, int h, bool black = true) {
  for (int yy = y; yy < y + h; ++yy) {
    for (int xx = x; xx < x + w; ++xx) pixel(xx, yy, black);
  }
}

static uint8_t glyphRow(char c, int row) {
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
    case ':': { static const uint8_t r[7]={0,0x06,0x06,0,0x06,0x06,0}; return r[row]; }
    case '-': { static const uint8_t r[7]={0,0,0,0x1F,0,0,0}; return r[row]; }
    case '/': { static const uint8_t r[7]={0x01,0x02,0x02,0x04,0x08,0x08,0x10}; return r[row]; }
    case '.': { static const uint8_t r[7]={0,0,0,0,0,0x06,0x06}; return r[row]; }
    case ' ': return 0;
    default: { static const uint8_t r[7]={0x0E,0x11,0x01,0x02,0x04,0,0x04}; return r[row]; }
  }
}

static void drawChar(char c, int x, int y, int scale) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 32);
  for (int row = 0; row < 7; ++row) {
    const uint8_t bits = glyphRow(c, row);
    for (int col = 0; col < 5; ++col) {
      if (bits & (1 << (4 - col))) {
        fillRect(x + col * scale, y + row * scale, scale, scale, true);
      }
    }
  }
}

static String asciiUpper(const String& in) {
  String out;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(in.c_str());
  while (*p) {
    if (*p < 0x80) {
      char c = static_cast<char>(*p++);
      if (c >= 'a' && c <= 'z') c -= 32;
      if (c >= 32 && c <= 126) out += c;
      continue;
    }
    if (*p == 0xC3 && p[1]) {
      switch (p[1]) {
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

static void drawText(const String& src, int x, int y, int scale = 2) {
  const String text = asciiUpper(src);
  int cx = x;
  for (size_t i = 0; i < text.length(); ++i) {
    drawChar(text[i], cx, y, scale);
    cx += 6 * scale;
    if (cx > 520) break;
  }
}

static String yesNo(bool v) { return v ? "OK" : "ERROR"; }

static String gbText(uint64_t bytes) {
  if (!bytes) return "0";
  const double gb = static_cast<double>(bytes) / 1000000000.0;
  return String(gb, 2) + " GB";
}

// -----------------------------------------------------------------------------
// SD diagnostics
// -----------------------------------------------------------------------------

static void powerCycleSd() {
  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(SD_PWR, OUTPUT);
  digitalWrite(SD_PWR, LOW);
  delay(150);
  digitalWrite(SD_PWR, HIGH);
  delay(300);
}

static void captureRawFiles(SdFat& sd) {
  filesFoundCount = 0;

  auto scan = [&](const char* path, const char* prefix) {
    FsFile dir = sd.open(path);
    if (!dir || !dir.isDirectory()) {
      if (dir) dir.close();
      return;
    }

    char name[96];
    for (FsFile f = dir.openNextFile(); f && filesFoundCount < 8; f = dir.openNextFile()) {
      if (!f.isDirectory()) {
        f.getName(name, sizeof(name));
        filesFound[filesFoundCount++] = String(prefix) + name;
      }
      f.close();
    }
    dir.close();
  };

  scan("/", "/");
  if (filesFoundCount < 8) scan("/Maple/Import", "/Maple/Import/");
}

static void runWriteReadRaw(SdFat& sd) {
  const char* path = "/MAPLE_SD_TEST.TXT";
  sd.remove(path);

  FsFile f = sd.open(path, O_RDWR | O_CREAT | O_TRUNC);
  if (f) {
    const char* msg = "MAPLE SD OK";
    const size_t n = f.print(msg);
    f.close();
    writeOk = (n == strlen(msg));
  }

  FsFile r = sd.open(path, O_RDONLY);
  if (r) {
    char buf[32] = {};
    const int n = r.read(buf, sizeof(buf) - 1);
    r.close();
    if (n > 0) {
      buf[n] = '\0';
      readOk = String(buf).startsWith("MAPLE SD OK");
    }
  }

  sd.remove(path);
}

static bool tryRaw(uint32_t hz) {
  powerCycleSd();
  SPI.end();
  delay(20);
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  SdFat* candidate = new SdFat();
  const bool ok = candidate->begin(SD_CS, hz);

  lastErrCode = candidate->sdErrorCode();
  lastErrData = candidate->sdErrorData();

  Serial.printf("[RAW] %lu Hz -> %s err=0x%02X data=0x%02X\n",
                static_cast<unsigned long>(hz),
                ok ? "OK" : "FAIL",
                lastErrCode,
                lastErrData);

  if (ok) {
    rawSd = candidate;
    rawHz = hz;
    rawOk = true;
    totalBytes =
        static_cast<uint64_t>(candidate->clusterCount()) * candidate->bytesPerCluster();
    captureRawFiles(*candidate);
    runWriteReadRaw(*candidate);
    return true;
  }

  delete candidate;
  return false;
}

static void runDiagnostics() {
  // Select the real X3 profile BEFORE touching SD. The pinned SDK then knows:
  // SCLK=8, MOSI=10, MISO=7, CS=12, PWR=13 active-high.
  display.setDisplayX3();
  BoardConfig::holdPowerRails();
  BoardConfig::releaseSdRail();

  Serial.printf("Active profile: %s\n", BoardConfig::ACTIVE.name);
  Serial.printf("SD pins: cs=%d sclk=%d miso=%d mosi=%d pwr=%d activeHigh=%d\n",
                BoardConfig::ACTIVE.sd.cs,
                BoardConfig::ACTIVE.sd.sclk >= 0 ? BoardConfig::ACTIVE.sd.sclk : BoardConfig::ACTIVE.display.sclk,
                BoardConfig::ACTIVE.sd.miso,
                BoardConfig::ACTIVE.sd.mosi >= 0 ? BoardConfig::ACTIVE.sd.mosi : BoardConfig::ACTIVE.display.mosi,
                BoardConfig::ACTIVE.sd.powerEnable,
                BoardConfig::ACTIVE.sd.powerActiveHigh);

  // Test 1: exactly what Maple uses.
  managerOk = SdMan.begin();
  Serial.printf("[MANAGER] begin -> %s\n", managerOk ? "OK" : "FAIL");

  if (managerOk) {
    totalBytes = SdMan.sdTotalBytes();

    const auto root = SdMan.listFiles("/", 5);
    for (const auto& f : root) {
      if (filesFoundCount < 8) filesFound[filesFoundCount++] = "/" + f;
    }
    if (filesFoundCount < 8 && SdMan.exists("/Maple/Import")) {
      const auto imports = SdMan.listFiles("/Maple/Import", 5);
      for (const auto& f : imports) {
        if (filesFoundCount < 8) filesFound[filesFoundCount++] = "/Maple/Import/" + f;
      }
    }

    const char* testPath = "/MAPLE_SD_TEST.TXT";
    SdMan.remove(testPath);
    writeOk = SdMan.writeFile(testPath, "MAPLE SD OK");
    const String back = SdMan.readFile(testPath);
    readOk = back.startsWith("MAPLE SD OK");
    SdMan.remove(testPath);
  } else {
    // Test 2: direct SdFat with explicit X3 pins and slower clocks.
    // If one succeeds, the hardware/card is fine and the problem is in the
    // manager timing/configuration rather than the SD slot itself.
    const uint32_t rates[] = {4000000, 10000000, 20000000, 40000000};
    for (uint32_t hz : rates) {
      if (tryRaw(hz)) break;
    }
  }

  // Critical shared-SPI handoff: deselect the SD and release Arduino SPI before
  // the E-Ink driver reconfigures the same bus. Keep SD power ON because the
  // pinned BoardConfig warns an unpowered X3 card can clamp SCLK/MOSI.
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);
  SPI.end();
  delay(50);
}

static void renderDiagnostic() {
  display.clearScreen(0xFF);

  drawText("MAPLE X3 SD TEST", 22, 25, 3);
  drawText(String("DIAG ") + DIAG_VERSION, 22, 65, 1);

  drawText("PERFIL: XTEINK X3", 22, 105, 2);
  drawText("PWR 13  CS 12  MISO 7", 22, 135, 1);
  drawText("SCLK 8  MOSI 10", 22, 155, 1);

  drawText(String("SD MANAGER: ") + yesNo(managerOk), 22, 205, 2);

  if (!managerOk) {
    drawText(String("SD RAW: ") + yesNo(rawOk), 22, 240, 2);
    if (rawOk) {
      drawText(String("RAW CLK: ") + String(rawHz / 1000000) + " MHZ", 22, 270, 1);
    } else {
      char err[64];
      snprintf(err, sizeof(err), "ERR: 0X%02X  DATA: 0X%02X", lastErrCode, lastErrData);
      drawText(err, 22, 270, 1);
    }
  }

  if (managerOk || rawOk) {
    drawText(String("CAPACIDAD: ") + gbText(totalBytes), 22, 315, 1);
    drawText(String("ESCRITURA: ") + yesNo(writeOk), 22, 340, 1);
    drawText(String("LECTURA: ") + yesNo(readOk), 22, 365, 1);

    drawText("ARCHIVOS:", 22, 410, 2);
    if (!filesFoundCount) {
      drawText("(NINGUNO)", 22, 445, 1);
    } else {
      int y = 445;
      for (int i = 0; i < filesFoundCount && i < 8; ++i) {
        String line = filesFound[i];
        if (line.length() > 38) line = line.substring(0, 38);
        drawText(line, 22, y, 1);
        y += 24;
      }
    }
  } else {
    drawText("NO SE PUDO MONTAR LA SD", 22, 330, 2);
    drawText("PRUEBA OTRA MICROSD FAT32", 22, 370, 1);
    drawText("Y REVISA QUE ESTE BIEN INSERTADA", 22, 395, 1);
  }

  drawText("OK = REINICIAR PRUEBA", 22, 745, 1);
  display.displayBuffer(EInkDisplay::FULL_REFRESH);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nMaple X3 SD Diagnostic v%s\n", DIAG_VERSION);

  runDiagnostics();

  // Reinitialize the display only AFTER all SD tests are complete.
  freeink::applyXteinkDisplayController();
  display.begin();

  renderDiagnostic();

  input.begin();
  input.beginAsync(2, 15, 32);
}

void loop() {
  uint8_t button = 0;
  while (input.popPress(button)) {
    if (button == InputManager::BTN_CONFIRM || button == InputManager::BTN_BACK) {
      delay(50);
      esp_restart();
    }
  }
  delay(5);
}
