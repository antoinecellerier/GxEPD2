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
// Key characteristics:
// - 2bpp data format with different encodings for full vs partial refresh.
//   Full refresh: 0b00=black, 0b11=white.
//   Partial refresh: 2bpp encodes (old_bit<<1)|new_bit transition codes.
// - SHL=0 in PSR causes right-to-left source scan; corrected in software
//   via reversed byte order + reversed bit order per row.
// - CMD 0x83 (partial RAM area) does not work for sub-region writes;
//   all writes send the full 920x680 screen.
// - Partial update requires a previous frame buffer (~78KB), lazily allocated.
//   If allocation fails (memory-constrained boards), only full refresh works.
// - Requires MAX_DISPLAY_BUFFER_SIZE >= 78200 (920*680/8) since paging does
//   not work without CMD 0x83 sub-region support.
//
// See GxEPD2_576_GDEH0576T81.cpp for detailed implementation notes.

#ifndef _GxEPD2_576_GDEH0576T81_H_
#define _GxEPD2_576_GDEH0576T81_H_

#include "../GxEPD2_EPD.h"

class GxEPD2_576_GDEH0576T81 : public GxEPD2_EPD
{
  public:
    // ---- Display attributes ----
    static const uint16_t WIDTH = 920;          // source lines (horizontal in native orientation)
    static const uint16_t WIDTH_VISIBLE = WIDTH;
    static const uint16_t HEIGHT = 680;         // gate lines (vertical in native orientation)
    static const GxEPD2::Panel panel = GxEPD2::GDEH0576T81;
    static const bool hasColor = false;
    static const bool hasPartialUpdate = true;
    static const bool hasFastPartialUpdate = true;
    static const uint16_t power_on_time = 100;          // ms, ~81ms measured
    static const uint16_t power_off_time = 150;         // ms, ~139ms measured
    static const uint16_t full_refresh_time = 750;      // ms, ~398ms measured
    static const uint16_t partial_refresh_time = 300;   // ms, per spec sheet

    // ---- Constructor ----
    GxEPD2_576_GDEH0576T81(int16_t cs, int16_t dc, int16_t rst, int16_t busy);

    // ---- Public API: screen buffer ----
    void clearScreen(uint8_t value = 0xFF);
    void writeScreenBuffer(uint8_t value = 0xFF);

    // ---- Public API: image write (no refresh) ----
    //  x and w should be multiple of 8
    void writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImageForFullRefresh(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImageToPrevious(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImageAgain(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                        int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImagePartToPrevious(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                  int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImagePartAgain(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                             int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    // Color overloads (B/W display — color channel is ignored)
    void writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                        int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);

    // ---- Public API: draw (write + refresh + sync) ----
    void drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                       int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                       int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);
    void drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false);

    // ---- Public API: refresh & power ----
    void refresh(bool partial_update_mode = false);
    void refresh(int16_t x, int16_t y, int16_t w, int16_t h);
    void powerOff();
    void hibernate();

  private:
    // ---- Previous frame buffer (for partial update interleaved encoding) ----
    bool _allocPreviousBuffer();
    void _storeToPrevious(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm);

    // ---- Display data write methods ----
    void _writeScreenBuffer(uint8_t value);
    void _writeImageAbsolute(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm);
    void _writeFullScreenInterleaved(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm);

    // ---- Controller commands ----
    void _setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool partial_mode = false); // NOTE: does not work for sub-regions on this panel
    void _PowerOn();
    void _PowerOff();
    void _InitDisplay();
    void _Init_Full();  // forced temperature LUT for full refresh
    void _Init_Part();  // OTP LUT for partial refresh
    void _Update_Full();
    void _Update_Part();

    // ---- State ----
    uint8_t* _previous_buffer; // lazily allocated, WIDTH * HEIGHT / 8 = 78200 bytes
};

#endif
