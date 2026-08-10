#include <Arduino.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <XteinkDetect.h>

// XTEINK X3 / X4 display SPI pinout.
// X3 is the only device compiled in this project.
static constexpr int EPD_SCLK = 8;
static constexpr int EPD_MOSI = 10;
static constexpr int EPD_CS   = 21;
static constexpr int EPD_DC   = 4;
static constexpr int EPD_RST  = 5;
static constexpr int EPD_BUSY = 6;

EInkDisplay display(EPD_SCLK, EPD_MOSI, EPD_CS, EPD_DC, EPD_RST, EPD_BUSY);

// We render in portrait coordinates (528 x 792), then rotate into the
// FreeInk X3 framebuffer (792 x 528).
static uint16_t logicalWidth()  { return display.getDisplayHeight(); }
static uint16_t logicalHeight() { return display.getDisplayWidth(); }

static void pixelPhysical(int x, int y, bool black) {
  if (x < 0 || y < 0 || x >= display.getDisplayWidth() || y >= display.getDisplayHeight()) return;

  uint8_t* fb = display.getFrameBuffer();
  const uint16_t rowBytes = display.getDisplayWidthBytes();
  const size_t index = static_cast<size_t>(y) * rowBytes + (x >> 3);
  const uint8_t mask = 0x80 >> (x & 7);

  // FreeInk's cleared framebuffer is 0xFF = white.
  if (black) fb[index] &= ~mask;
  else       fb[index] |= mask;
}

static void pixel(int x, int y, bool black = true) {
  // Portrait -> physical landscape. If your unit appears upside down, this is
  // the only mapping we will flip in the next build.
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
  // 5x7 uppercase font. Each return value uses the low 5 bits.
  switch (c) {
    case 'A': { static const uint8_t r[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return r[row]; }
    case 'B': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return r[row]; }
    case 'D': { static const uint8_t r[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return r[row]; }
    case 'E': { static const uint8_t r[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return r[row]; }
    case 'G': { static const uint8_t r[7]={0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}; return r[row]; }
    case 'H': { static const uint8_t r[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return r[row]; }
    case 'I': { static const uint8_t r[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; return r[row]; }
    case 'L': { static const uint8_t r[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return r[row]; }
    case 'M': { static const uint8_t r[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return r[row]; }
    case 'P': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return r[row]; }
    case 'R': { static const uint8_t r[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return r[row]; }
    case 'T': { static const uint8_t r[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return r[row]; }
    case 'U': { static const uint8_t r[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return r[row]; }
    case 'X': { static const uint8_t r[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return r[row]; }
    case 'Y': { static const uint8_t r[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return r[row]; }
    case '0': { static const uint8_t r[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return r[row]; }
    case '1': { static const uint8_t r[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return r[row]; }
    case '3': { static const uint8_t r[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return r[row]; }
    case '.': { static const uint8_t r[7]={0x00,0x00,0x00,0x00,0x00,0x06,0x06}; return r[row]; }
    case '-': { static const uint8_t r[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return r[row]; }
    case ' ': return 0;
    default:  return 0;
  }
}

static void drawChar(char c, int x, int y, int scale) {
  for (int row = 0; row < 7; ++row) {
    const uint8_t bits = glyphRow(c, row);
    for (int col = 0; col < 5; ++col) {
      if (bits & (1 << (4 - col))) {
        fillRect(x + col * scale, y + row * scale, scale, scale, true);
      }
    }
  }
}

static int textWidth(const char* text, int scale) {
  int n = 0;
  while (text[n]) ++n;
  return n ? n * 6 * scale - scale : 0;
}

static void drawText(const char* text, int x, int y, int scale) {
  while (*text) {
    drawChar(*text++, x, y, scale);
    x += 6 * scale;
  }
}

static void drawCentered(const char* text, int y, int scale) {
  drawText(text, (static_cast<int>(logicalWidth()) - textWidth(text, scale)) / 2, y, scale);
}

static void drawTestScreen() {
  display.clearScreen(0xFF);

  // Border confirms full geometry and rotation.
  fillRect(18, 18, logicalWidth() - 36, 3);
  fillRect(18, logicalHeight() - 21, logicalWidth() - 36, 3);
  fillRect(18, 18, 3, logicalHeight() - 36);
  fillRect(logicalWidth() - 21, 18, 3, logicalHeight() - 36);

  drawCentered("MAPLE X3", 245, 7);
  drawCentered("GITHUB BUILD", 350, 3);
  drawCentered("READY", 410, 4);

  // Three bars make orientation obvious.
  fillRect(70, 520, logicalWidth() - 140, 12);
  fillRect(110, 550, logicalWidth() - 220, 12);
  fillRect(150, 580, logicalWidth() - 300, 12);
}

void setup() {
  Serial.begin(115200);
  delay(250);

  // Safe board bring-up helpers from FreeInk.
  BoardConfig::holdPowerRails();
  BoardConfig::releaseSdRail();

  // X3-only build: select X3 geometry before display init.
  display.setDisplayX3();

  // Detect newer X3 batches that use the UC8279 sibling controller.
  freeink::applyXteinkDisplayController();

  display.begin();
  drawTestScreen();
  display.displayBuffer(EInkDisplay::FULL_REFRESH);

  Serial.printf("Maple X3 v0.1 booted. Display=%ux%u\n",
                display.getDisplayWidth(), display.getDisplayHeight());
}

void loop() {
  delay(1000);
}
