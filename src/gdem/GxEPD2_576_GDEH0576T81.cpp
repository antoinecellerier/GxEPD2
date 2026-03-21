// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display, available here: https://www.good-display.com/comp/xcompanyFile/downloadNew.do?appId=24&fid=2389&id=1782
// Panel: GDEH0576T81 : https://www.good-display.com/product/702.html
// Controller : SSD2677 (UC-compatible mode) : https://v4.cecdn.yun300.cn/100001_1909185148/SSD2677%28Rev1.1%29N.pdf
//
// Author: Jean-Marc Zingg
// Contributor: Antoine Cellerier
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2
//
// ============================================================================
// Implementation Notes
// ============================================================================
//
// This panel uses the SSD2677 in UC-compatible mode, which is fundamentally
// different from the SSD1680-compatible mode used by the GDEM0397T81 driver.
// The command set (0x00 PSR, 0x04 Power On, 0x10 DTM, 0x12 DRF, etc.) and
// data format (2bpp) are UC8179-family, not SSD1680-family.
//
// 2bpp Data Format:
//   The controller uses 2 bits per pixel written to DTM1 (command 0x10).
//   The encoding differs between full and partial refresh:
//
//   Full refresh (absolute pixel values):
//     0b00 = black, 0b11 = white
//     (0b01 also appears to work as white but 0b11 matches the sample code)
//
//   Partial refresh (transition codes, interleaved old+new):
//     Each 2bpp pair encodes (old_bit << 1) | new_bit:
//     0b00 = was black,  stays black  (no change)
//     0b01 = was black,  becomes white
//     0b10 = was white,  becomes black
//     0b11 = was white,  stays white  (no change)
//     This requires knowing the previous frame content, stored in _previous_buffer.
//     See sample code function PIC_display_Part_ALL / bitInterleave for reference.
//
// Horizontal Mirror (SHL=0):
//   PSR register value 0x27 has SHL=0, meaning the source driver scans
//   right-to-left. Changing to SHL=1 (PSR 0x37) was tested and does NOT work
//   (display produces no output). Instead, we fix the mirror in software by
//   reversing byte order within each row AND reversing bit order within each
//   byte (_reverseByte). Both reversals are required together; either alone
//   produces 8-pixel zigzag artifacts on diagonal lines.
//
// Partial RAM Area (CMD 0x83):
//   Command 0x83 was tested for sub-region writes (as used by the GDEY116F51
//   driver on the same SSD2677 controller). It does NOT work on this panel —
//   data always writes starting from RAM position 0 regardless of the window
//   coordinates. The sample code also never uses CMD 0x83. Therefore, all
//   writes send the full 920x680 screen (156,400 bytes at 2bpp). This means:
//   - Paging (splitting writes across multiple pages) does not work
//   - The GxEPD2_BW buffer must be large enough for the full screen
//   - On ESP32: set MAX_DISPLAY_BUFFER_SIZE >= 78200 (920*680/8)
//
// Previous Frame Buffer:
//   Partial refresh requires both old and new pixel data. A ~78KB buffer
//   (_previous_buffer) is lazily allocated via malloc on first use. If
//   allocation fails (e.g., on memory-constrained boards), partial update
//   is unavailable and all refreshes use the full refresh path.
//
// LUT Selection:
//   Full refresh: forced temperature LUT (0xE0=0x02, 0xE6=temp, 0xA5)
//     Temperature is read from the controller's internal sensor (CMD 0x40)
//     and mapped to compensation values per the sample code.
//   Partial refresh: OTP LUT (0xE0=0x00, 0xA5)
//     Using the same LUT (0xE0=0x00) for both full and partial produces
//     noise bands during full refresh because the LUT interprets absolute
//     pixel values as transition codes.
//
// Busy Signal:
//   The DESPI-C02 adapter board inverts the BUSY signal relative to the
//   datasheet. The datasheet says HIGH=busy, but through the DESPI-C02,
//   LOW=busy and HIGH=ready. The busy_level parameter is set to LOW.
//   The sample code's lcd_chkstatus() waits for BUSY==1 (HIGH=ready),
//   which is consistent with this.
//
// ============================================================================

#include "GxEPD2_576_GDEH0576T81.h"

// ============================================================================
// Lookup Tables
// ============================================================================

// Reverse bit order in a byte.
// Required because SHL=0 in PSR causes right-to-left source scanning.
// Combined with reversed byte order per row, this corrects the horizontal
// mirror. Both byte reversal AND bit reversal are needed; either alone
// produces 8-pixel zigzag artifacts on diagonal lines.
static uint8_t _reverseByte(uint8_t b)
{
  b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
  b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
  b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
  return b;
}

// Expand a 4-bit nibble to 8-bit 2bpp for FULL REFRESH.
// Each input bit becomes 2 output bits: 1 -> 0b11 (white), 0 -> 0b00 (black).
// Matches sample code EPD_W21_WriteDATA_1To2 which maps bit=1 to 0x03.
// Note: using 0b01 for white instead of 0b11 was tested and produces grayish
// output — the full refresh LUT expects 0b11 for white.
static const uint8_t nibble_to_2bpp[] PROGMEM =
{
  0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
  0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};

// Interleave old and new nibbles for PARTIAL REFRESH.
// Index: (old_nibble << 4) | new_nibble
// Output byte: (old_bit3 new_bit3 old_bit2 new_bit2 old_bit1 new_bit1 old_bit0 new_bit0)
// Matches sample code bitInterleave() function.
static const uint8_t interleave_2bpp[256] PROGMEM =
{
#define IL(o, n) (uint8_t)( \
  (((o)>>3 & 1)<<7) | (((n)>>3 & 1)<<6) | \
  (((o)>>2 & 1)<<5) | (((n)>>2 & 1)<<4) | \
  (((o)>>1 & 1)<<3) | (((n)>>1 & 1)<<2) | \
  (((o)>>0 & 1)<<1) | (((n)>>0 & 1)<<0) )
#define IL16(o) IL(o,0),IL(o,1),IL(o,2),IL(o,3),IL(o,4),IL(o,5),IL(o,6),IL(o,7), \
               IL(o,8),IL(o,9),IL(o,10),IL(o,11),IL(o,12),IL(o,13),IL(o,14),IL(o,15)
  IL16(0), IL16(1), IL16(2), IL16(3), IL16(4), IL16(5), IL16(6), IL16(7),
  IL16(8), IL16(9), IL16(10), IL16(11), IL16(12), IL16(13), IL16(14), IL16(15)
#undef IL16
#undef IL
};

// ============================================================================
// Constructor
// ============================================================================

GxEPD2_576_GDEH0576T81::GxEPD2_576_GDEH0576T81(int16_t cs, int16_t dc, int16_t rst, int16_t busy) :
  // busy_level = LOW: through DESPI-C02, LOW=busy, HIGH=ready
  // (datasheet says HIGH=busy, but the adapter inverts the signal)
  GxEPD2_EPD(cs, dc, rst, busy, LOW, 10000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate)
{
  _previous_buffer = nullptr;
}

// ============================================================================
// Previous Frame Buffer Management
// ============================================================================

// Lazily allocate the previous frame buffer for partial update support.
// Returns true if buffer is available (already allocated or newly allocated).
// On memory-constrained boards, malloc may fail and partial update will be
// unavailable — writeImage falls back to _writeImageAbsolute (full refresh).
bool GxEPD2_576_GDEH0576T81::_allocPreviousBuffer()
{
  if (_previous_buffer) return true;
  _previous_buffer = (uint8_t*)malloc(uint32_t(WIDTH) * uint32_t(HEIGHT) / 8);
  if (_previous_buffer)
  {
    memset(_previous_buffer, 0xFF, uint32_t(WIDTH) * uint32_t(HEIGHT) / 8); // assume white
  }
  return _previous_buffer != nullptr;
}

// Copy a region of bitmap data into _previous_buffer at the given coordinates.
// Maintains a 1bpp snapshot of the current screen content (in GxEPD2's native
// byte order, without SHL=0 reversal) for constructing interleaved old+new
// data during the next partial refresh.
void GxEPD2_576_GDEH0576T81::_storeToPrevious(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h,
                                               bool invert, bool mirror_y, bool pgm)
{
  if (!_previous_buffer) return;
  uint16_t wb = (w + 7) / 8;
  x -= x % 8;
  w = wb * 8;
  int16_t x1 = x < 0 ? 0 : x;
  int16_t y1 = y < 0 ? 0 : y;
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x;
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y;
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  uint16_t wb_full = WIDTH / 8;
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint32_t idx = mirror_y ? j + dx / 8 + uint32_t((h - 1 - (i + dy))) * wb : j + dx / 8 + uint32_t(i + dy) * wb;
      uint8_t data;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      _previous_buffer[(y1 + i) * wb_full + (x1 / 8 + j)] = data;
    }
  }
}

// ============================================================================
// Public API: Screen Buffer Operations
// ============================================================================

void GxEPD2_576_GDEH0576T81::clearScreen(uint8_t value)
{
  writeScreenBuffer(value);
  refresh(false);
}

void GxEPD2_576_GDEH0576T81::writeScreenBuffer(uint8_t value)
{
  if (!_init_display_done) _Init_Full();
  _writeScreenBuffer(value);
  _initial_write = false;
}

// Fill the entire display RAM with a uniform color.
// Also updates _previous_buffer to match.
void GxEPD2_576_GDEH0576T81::_writeScreenBuffer(uint8_t value)
{
  if (!_init_display_done) _Init_Full();
  // Sample code waits for busy after CMD 0x10 before writing data
  _writeCommand(0x10); // DTM1 — start data transmission
  _waitWhileBusy("_writeScreenBuffer DTM1", power_on_time);
  _startTransfer();
  // Full refresh encoding: white = 0xFF (all 0b11 pairs), black = 0x00 (all 0b00 pairs)
  uint8_t val_2bpp = (value == 0xFF) ? 0xFF : 0x00;
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 4; i++)
  {
    _transfer(val_2bpp);
  }
  _endTransfer();
  // Keep previous buffer in sync
  if (_allocPreviousBuffer())
  {
    memset(_previous_buffer, value, uint32_t(WIDTH) * uint32_t(HEIGHT) / 8);
  }
}

// ============================================================================
// Public API: Image Write Operations
// ============================================================================

void GxEPD2_576_GDEH0576T81::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_previous_buffer)
  {
    // Partial update path: write full screen with interleaved old+new encoding
    _writeFullScreenInterleaved(bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
  else
  {
    // No previous buffer available — fall back to absolute encoding (full refresh only)
    _writeImageAbsolute(bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_576_GDEH0576T81::writeImageForFullRefresh(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _allocPreviousBuffer(); // try to allocate for future partial updates
  _writeImageAbsolute(bitmap, x, y, w, h, invert, mirror_y, pgm);
  _storeToPrevious(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::writeImageToPrevious(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _storeToPrevious(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

// Called by GxEPD2_BW after refresh to equalize old/new buffers for next partial update.
void GxEPD2_576_GDEH0576T81::writeImageAgain(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _storeToPrevious(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::writeImagePartAgain(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  // TODO: implement if needed for paged partial updates
}

// ============================================================================
// Internal: Full Refresh Data Write (absolute 2bpp encoding)
// ============================================================================

// Write image data using absolute 2bpp encoding for full refresh.
// Always writes the entire 920x680 screen — CMD 0x83 (partial RAM area)
// does not work on this panel for sub-region writes.
// Pixels outside the bitmap region are filled with white (0xFF).
//
// The sample code flow is: EPD_init() -> PIC_display() -> EPD_update()
// where PIC_display writes CMD 0x10 then all pixel data. No CMD 0x83 is used.
void GxEPD2_576_GDEH0576T81::_writeImageAbsolute(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // first-time full screen clear
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32

  // Clamp bitmap region to display bounds
  uint16_t wb = (w + 7) / 8; // width bytes, bitmaps are padded
  x -= x % 8; // byte boundary
  w = wb * 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x;
  int16_t y1 = y < 0 ? 0 : y;
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x;
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y;
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;

  if (!_init_display_done) _Init_Full();

  // Begin full-screen data transmission
  // Note: no CMD 0x83 — sample code does not use it for full refresh
  _writeCommand(0x10); // DTM1
  _waitWhileBusy("_writeImageAbsolute DTM1", power_on_time); // sample waits after 0x10
  _startTransfer();

  uint16_t wb_full = WIDTH / 8; // 920/8 = 115 bytes per row
  for (int16_t row = 0; row < int16_t(HEIGHT); row++)
  {
    // SHL=0 fix: write bytes in reverse order (rightmost byte first)
    for (int16_t col = wb_full - 1; col >= 0; col--)
    {
      uint8_t data;
      if (row >= y1 && row < y1 + h1 && col >= x1 / 8 && col < (x1 + w1) / 8)
      {
        // Inside the bitmap region — read from source
        int16_t j = col - x1 / 8;
        uint32_t idx = mirror_y ? j + dx / 8 + uint32_t((h - 1 - (row - y1 + dy))) * wb : j + dx / 8 + uint32_t(row - y1 + dy) * wb;
        if (pgm)
        {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
          data = pgm_read_byte(&bitmap[idx]);
#else
          data = bitmap[idx];
#endif
        }
        else
        {
          data = bitmap[idx];
        }
        if (invert) data = ~data;
      }
      else
      {
        data = 0xFF; // outside bitmap region — white
      }
      // SHL=0 fix: reverse bit order within each byte
      data = _reverseByte(data);
      // Expand 1bpp to 2bpp: 1->0b11 (white), 0->0b00 (black)
      _transfer(pgm_read_byte(&nibble_to_2bpp[(data >> 4) & 0x0F]));
      _transfer(pgm_read_byte(&nibble_to_2bpp[data & 0x0F]));
    }
  }

  _endTransfer();
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

// ============================================================================
// Internal: Partial Refresh Data Write (interleaved old+new encoding)
// ============================================================================

// Write full screen with interleaved old+new encoding for partial refresh.
// For each pixel, outputs 2bpp = (old_bit << 1) | new_bit.
// Requires _previous_buffer to contain the current screen content.
// Pixels outside the changed region are encoded as old=new (no change).
//
// This matches the sample code's PIC_display_Part_ALL function which also
// writes the full screen for every partial update — there is no sub-region
// partial write capability on this controller.
void GxEPD2_576_GDEH0576T81::_writeFullScreenInterleaved(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // first-time full screen clear
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32

  // Clamp bitmap region to display bounds
  uint16_t wb = (w + 7) / 8;
  x -= x % 8;
  w = wb * 8;
  int16_t x1 = x < 0 ? 0 : x;
  int16_t y1 = y < 0 ? 0 : y;
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x;
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y;
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;

  if (!_using_partial_mode) _Init_Part();

  _writeCommand(0x10); // DTM1
  _waitWhileBusy("_writeFullScreenInterleaved DTM1", power_on_time);
  _startTransfer();

  uint16_t wb_full = WIDTH / 8;
  for (int16_t row = 0; row < int16_t(HEIGHT); row++)
  {
    // SHL=0 fix: write bytes in reverse order
    for (int16_t col = wb_full - 1; col >= 0; col--)
    {
      // Read old pixel data from previous frame buffer
      uint8_t old_data = _previous_buffer[row * wb_full + col];
      uint8_t new_data;

      if (row >= y1 && row < y1 + h1 && col >= x1 / 8 && col < (x1 + w1) / 8)
      {
        // Inside the changed region — read new data from bitmap
        int16_t j = col - x1 / 8;
        uint32_t idx = mirror_y ? j + dx / 8 + uint32_t((h - 1 - (row - y1 + dy))) * wb : j + dx / 8 + uint32_t(row - y1 + dy) * wb;
        if (pgm)
        {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
          new_data = pgm_read_byte(&bitmap[idx]);
#else
          new_data = bitmap[idx];
#endif
        }
        else
        {
          new_data = bitmap[idx];
        }
        if (invert) new_data = ~new_data;
      }
      else
      {
        // Outside the changed region — no change (new = old)
        new_data = old_data;
      }

      // SHL=0 fix: reverse bit order within each byte
      old_data = _reverseByte(old_data);
      new_data = _reverseByte(new_data);

      // Interleave old+new nibbles: output (old_bit<<1)|new_bit per pixel
      _transfer(pgm_read_byte(&interleave_2bpp[((old_data >> 4) << 4) | ((new_data >> 4) & 0x0F)]));
      _transfer(pgm_read_byte(&interleave_2bpp[((old_data & 0x0F) << 4) | (new_data & 0x0F)]));
    }
  }

  _endTransfer();
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

// ============================================================================
// Public API: Image Part Write (delegates to full-screen write)
// ============================================================================

void GxEPD2_576_GDEH0576T81::writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  // CMD 0x83 sub-region writes don't work; delegate to full-screen write
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::writeImagePartToPrevious(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  // TODO: implement if needed
}

// ============================================================================
// Public API: Color Overloads (B/W display — ignore color channel)
// ============================================================================

void GxEPD2_576_GDEH0576T81::writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black) writeImage(black, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black) writeImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1) writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
}

// ============================================================================
// Public API: Draw Operations (write + refresh + sync previous buffer)
// ============================================================================

void GxEPD2_576_GDEH0576T81::drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImageAgain(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_576_GDEH0576T81::drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black) drawImage(black, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black) drawImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_576_GDEH0576T81::drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1) drawImage(data1, x, y, w, h, invert, mirror_y, pgm);
}

// ============================================================================
// Public API: Refresh
// ============================================================================

void GxEPD2_576_GDEH0576T81::refresh(bool partial_update_mode)
{
  if (partial_update_mode) refresh(0, 0, WIDTH, HEIGHT);
  else
  {
    // Full refresh — data should already be written via _Init_Full path
    _Update_Full();
    _initial_refresh = false;
  }
}

void GxEPD2_576_GDEH0576T81::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (_initial_refresh) return refresh(false); // first update must be full refresh
  if (!_using_partial_mode) _Init_Part();
  _Update_Part();
}

// ============================================================================
// Public API: Power Management
// ============================================================================

void GxEPD2_576_GDEH0576T81::powerOff(void)
{
  _PowerOff();
}

void GxEPD2_576_GDEH0576T81::hibernate()
{
  _PowerOff();
  if (_rst >= 0)
  {
    _writeCommand(0x07); // deep sleep
    _writeData(0xA5);    // check code
    _hibernating = true;
    _init_display_done = false;
  }
}

// ============================================================================
// Internal: Partial RAM Area (CMD 0x83)
// ============================================================================

// NOTE: CMD 0x83 was tested and does NOT work for sub-region writes on this
// panel. Data always writes from RAM position 0 regardless of coordinates.
// This function is retained but currently unused — all writes send the full
// screen via CMD 0x10 without prior CMD 0x83.
void GxEPD2_576_GDEH0576T81::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool partial_mode)
{
  _writeCommand(0x83);
  _writeData(x / 256);
  _writeData(x % 256);
  _writeData((x + w - 1) / 256);
  _writeData((x + w - 1) % 256);
  _writeData(y / 256);
  _writeData(y % 256);
  _writeData((y + h - 1) / 256);
  _writeData((y + h - 1) % 256);
  _writeData(partial_mode ? 0x01 : 0x00);
}

// ============================================================================
// Internal: Power Control
// ============================================================================

void GxEPD2_576_GDEH0576T81::_PowerOn()
{
  if (!_power_is_on)
  {
    _writeCommand(0x04); // Power On
    _waitWhileBusy("_PowerOn", power_on_time);
  }
  _power_is_on = true;
}

void GxEPD2_576_GDEH0576T81::_PowerOff()
{
  if (_power_is_on)
  {
    _writeCommand(0x02); // Power Off
    _writeData(0x00);
    _waitWhileBusy("_PowerOff", power_off_time);
  }
  _power_is_on = false;
  _using_partial_mode = false;
}

// ============================================================================
// Internal: Display Initialization
// ============================================================================

// Configure all controller registers. From sample code EPD_init().
// Hardware reset is only performed on first use or after hibernate.
void GxEPD2_576_GDEH0576T81::_InitDisplay()
{
  // Hardware reset — only needed after hibernate or on first init
  if ((_rst >= 0) && (_hibernating || _initial_write))
  {
    digitalWrite(_rst, HIGH);
    delay(20);
    digitalWrite(_rst, LOW);  // assert reset
    delay(10);                // 10ms reset pulse (from sample code)
    digitalWrite(_rst, HIGH);
    delay(10);                // 10ms recovery (from sample code)
    _waitWhileBusy("_InitDisplay reset", power_on_time);
    _hibernating = false;
    _power_is_on = false;
  }

  // Panel Setting Register — from sample code
  // 0x27: REG=0(OTP), KWR=0, UD=1(scan up), SHL=0(shift right), SHD_N=0
  // SHL=0 causes horizontal mirror — corrected in software via _reverseByte
  // Note: PSR 0x37 (SHL=1) was tested and does NOT work (no display output)
  _writeCommand(0x00); // PSR
  _writeData(0x27);
  _writeData(0x0E);
  _waitWhileBusy("_InitDisplay PSR", power_on_time);

  _writeCommand(0x06); // BTST — Booster Soft Start
  _writeData(0x0F);
  _writeData(0x8B);
  _writeData(0x9C);
  _writeData(0xC1);

  _writeCommand(0xE7); // PST
  _writeData(0xC1);

  _writeCommand(0x30); // PLL — clock frequency
  _writeData(0x08);

  _writeCommand(0x50); // CDI — VCOM and Data Interval Setting
  _writeData(0x77);    // VBD=01(black border), DDX=11, CDI=0111

  _writeCommand(0x61); // TRES — Resolution Setting
  _writeData(WIDTH / 256);   // source high byte (920 = 0x0398)
  _writeData(WIDTH % 256);   // source low byte
  _writeData(HEIGHT / 256);  // gate high byte (680 = 0x02A8)
  _writeData(HEIGHT % 256);  // gate low byte

  _writeCommand(0x62); // HTOTAL — panel-specific timing parameters
  _writeData(0x98);
  _writeData(0x98);
  _writeData(0x98);
  _writeData(0x75);
  _writeData(0xCA);
  _writeData(0xB2);
  _writeData(0x98);
  _writeData(0x7E);

  _writeCommand(0x65); // GSST — Gate Start Setting
  _writeData(0x00);
  _writeData(0x00);
  _writeData(0x00);
  _writeData(0x00);

  _writeCommand(0xE9); // PST
  _writeData(0x01);

  _init_display_done = true;
}

// Initialize for full refresh with forced temperature LUT.
// Reads temperature from controller's internal sensor and selects the
// matching LUT compensation value, per sample code Write_LUT_All / Read_temp.
// Using OTP LUT (0xE0=0x00) for full refresh was tested and produces noise
// bands — the full refresh LUT expects different waveform parameters.
void GxEPD2_576_GDEH0576T81::_Init_Full()
{
  _InitDisplay();

  // Read temperature from controller's internal sensor (CMD 0x40)
  _writeCommand(0x40);
  _waitWhileBusy("_Init_Full ReadTemp", power_on_time);
  uint8_t temp = _readData();

  // Map temperature to LUT compensation value (from sample code)
  uint8_t tempvalue;
  if (temp <= 5)        tempvalue = 232;  // 0xE8
  else if (temp <= 10)  tempvalue = 235;  // 0xEB
  else if (temp <= 20)  tempvalue = 238;  // 0xEE
  else if (temp <= 30)  tempvalue = 241;  // 0xF1
  else if (temp <= 127) tempvalue = 244;  // 0xF4
  else                  tempvalue = 232;  // 0xE8 (fallback)

  // Activate LUT with temperature compensation
  _writeCommand(0xE0); // Cascade Setting
  _writeData(0x02);    // TSFIX — use forced temperature
  _writeCommand(0xE6); // Force Temperature
  _writeData(tempvalue);
  _writeCommand(0xA5); // Activate LUT
  _waitWhileBusy("_Init_Full LUT", power_on_time);
  delay(10);           // at least 10ms delay after LUT activation (from sample code)

  _PowerOn();
  _using_partial_mode = false;
}

// Initialize for partial refresh with OTP LUT.
// Uses 0xE0=0x00 (no forced temperature) and 0xA5 to load the partial
// waveform from OTP. The partial LUT interprets 2bpp data as transition
// codes (old_bit<<1 | new_bit), not absolute pixel values.
void GxEPD2_576_GDEH0576T81::_Init_Part()
{
  _InitDisplay();

  _writeCommand(0xE0); // Cascade Setting
  _writeData(0x00);    // No TSFIX — use OTP partial waveform
  _writeCommand(0xA5); // Activate LUT
  _waitWhileBusy("_Init_Part LUT", power_on_time);

  _PowerOn();
  _using_partial_mode = true;
}

// ============================================================================
// Internal: Display Refresh
// ============================================================================

void GxEPD2_576_GDEH0576T81::_Update_Full()
{
  _writeCommand(0x12); // Display Refresh (DRF)
  _writeData(0x00);
  _waitWhileBusy("_Update_Full", full_refresh_time);
  _init_display_done = false; // controller needs re-init after refresh
}

void GxEPD2_576_GDEH0576T81::_Update_Part()
{
  _writeCommand(0x12); // Display Refresh (DRF)
  _writeData(0x00);
  _waitWhileBusy("_Update_Part", partial_refresh_time);
}
