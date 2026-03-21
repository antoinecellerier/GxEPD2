// GxEPD2_GridDiagnostic.ino
//
// Display diagnostic for SPI e-paper panels. Draws a labeled coordinate grid,
// corner markers, diagonals, and a center crosshair to verify:
// - Correct pixel orientation and mapping (no mirroring or rotation issues)
// - Diagonal line rendering (detects byte/bit alignment problems)
// - Full screen coverage (detects paging or partial RAM area issues)
// - Coordinate system (native rotation=0)
//
// The grid adapts to any display size. Grid lines are drawn every gridSpacing
// pixels, with coordinate labels at every other intersection to avoid clutter.
// Two diagonals cross the screen: one solid, one dashed, to test line rendering
// at non-axis-aligned angles.
//
// Usage: select your display in GxEPD2_display_selection_new_style.h (or one of
// the old-style selection files), then upload. Hold the display in its native
// orientation and compare the on-screen labels with expected coordinates.
//
// Author: Antoine Cellerier
//
// Library: https://github.com/ZinggJM/GxEPD2

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_4C.h>
#include <GxEPD2_7C.h>

// select the display class and display driver class in the following file (new style):
#include "GxEPD2_display_selection_new_style.h"

// or select the display constructor line in one of the following files (old style):
#include "GxEPD2_display_selection.h"
#include "GxEPD2_display_selection_added.h"

void setup()
{
  Serial.begin(115200);
  display.init(115200);
  display.setRotation(0); // native orientation — no rotation

  uint16_t W = display.width();
  uint16_t H = display.height();
  Serial.print("Display: "); Serial.print(W); Serial.print("x"); Serial.println(H);

  // Adaptive grid spacing: target ~8-12 cells per axis
  int gridSpacing = 40;
  if (W >= 400 || H >= 400) gridSpacing = 80;
  if (W >= 800 || H >= 800) gridSpacing = 100;
  // Label every Nth intersection to avoid clutter
  int labelEvery = (gridSpacing >= 80) ? 2 : 4;
  int labelSpacing = gridSpacing * labelEvery;

  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    // ---- Grid lines ----
    for (int x = 0; x < W; x += gridSpacing)
      display.drawFastVLine(x, 0, H, GxEPD_BLACK);
    for (int y = 0; y < H; y += gridSpacing)
      display.drawFastHLine(0, y, W, GxEPD_BLACK);

    // ---- Coordinate labels at intersections ----
    display.setFont(); // default 5x7 built-in font
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(W >= 400 ? 2 : 1);
    for (int x = 0; x < W; x += labelSpacing)
    {
      for (int y = 0; y < H; y += labelSpacing)
      {
        char buf[8];
        int textH = (W >= 400) ? 14 : 7;
        snprintf(buf, sizeof(buf), "x%d", x);
        display.setCursor(x + 3, y + 3);
        display.print(buf);
        snprintf(buf, sizeof(buf), "y%d", y);
        display.setCursor(x + 3, y + 3 + textH + 2);
        display.print(buf);
      }
    }

    // ---- Diagonal lines ----
    // Solid diagonal: (0,0) to (W-1,H-1)
    // Useful for detecting byte-level pixel alignment issues (zigzag = broken)
    display.drawLine(0, 0, W - 1, H - 1, GxEPD_BLACK);
    // Dashed diagonal: (W-1,0) to (0,H-1)
    for (int i = 0; i < 100; i += 2)
    {
      int x1 = (W - 1) - (W - 1) * i / 100;
      int y1 = (H - 1) * i / 100;
      int x2 = (W - 1) - (W - 1) * (i + 1) / 100;
      int y2 = (H - 1) * (i + 1) / 100;
      display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);
    }

    // ---- Corner labels ----
    display.setTextSize(W >= 400 ? 2 : 1);
    display.setCursor(4, 4);
    display.print("0,0");
    display.setCursor(W - (W >= 400 ? 80 : 40), 4);
    display.print(W - 1); display.print(",0");
    display.setCursor(4, H - (W >= 400 ? 20 : 10));
    display.print("0,"); display.print(H - 1);
    display.setCursor(W - (W >= 400 ? 120 : 60), H - (W >= 400 ? 20 : 10));
    display.print(W - 1); display.print(","); display.print(H - 1);

    // ---- Center crosshair ----
    display.drawFastHLine(W / 2 - 20, H / 2, 40, GxEPD_BLACK);
    display.drawFastVLine(W / 2, H / 2 - 20, 40, GxEPD_BLACK);
    display.setTextSize(1);
    display.setCursor(W / 2 + 5, H / 2 + 5);
    display.print("CTR");
  }
  while (display.nextPage());

  display.hibernate();
  Serial.println("Done.");
}

void loop() {}
