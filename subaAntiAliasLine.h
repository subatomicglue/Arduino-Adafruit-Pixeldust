#ifndef SUBA_LINE_DRAWING_H
#define SUBA_LINE_DRAWING_H

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subatomic's Anti Aliased Line Drawing - suba::AntiAliasLine::drawAALine()
// - draws to Adafruit_Protomatter Matrix   (e.g. Matrix Portal M4)
//
// Uses Xiaolin Wu's line algorithm
// https://en.wikipedia.org/wiki/Xiaolin_Wu%27s_line_algorithm
// Abrash, Michael (June 1992). "Fast Antialiasing (Column)". Dr. Dobb's Journal. 17 (6): 139(7).
// Wu, Xiaolin (July 1991). "An efficient antialiasing technique". Computer Graphics. 25 (4): 143–152. doi:10.1145/127719.122734. ISBN 0-89791-436-8.
// Wu, Xiaolin (1991). "Fast Anti-Aliased Circle Generation". In James Arvo (ed.). Graphics Gems II. San Francisco: Morgan Kaufmann. pp. 446–450. ISBN 0-12-064480-0.
////////////////////////////////////////////////////////////////////////////////////////////////////////

// subatomic namespace
namespace suba {
namespace AntiAliasLine {

// integer part of x
double ipart(double x) {
    return floor(x);
}

// round the number
double roundToNearestInt(double x) {
    return ipart(x + 0.5);
}

// fractional part of x
double fpart(double x) {
    return x - floor(x);
}

double rfpart(double x) {
    return 1 - fpart(x);
}

// swaps two numbers
void swap(double* a , double*b)
{
    double temp = *a;
    *a = *b;
    *b = temp;
}

// draws a pixel on screen of given brightness
// 0<=brightness<=1.
// assumes MatrixPortalM4 code is #included before this header, and the matrix obj is instantiated
void plot( int x , int y , double brightness, const suba::Vec3& c1, const suba::Vec3& c2, double t /* 0 to 1 position on the line being drawn */ )
{
  // dont draw if pixel is too dim
  if (brightness < 0.062745098039216) return; // results in < 16 when (brightness * 255), which the Matrixportal M4 neopixels render as black, so let's avoid drawing black pixels.

  suba::Vec3 c = suba::mul( suba::lerp( c1, c2, t ), brightness );
  matrix.drawPixel( x, y, matrix.color565(c[0], c[1], c[2]) );

  const bool DEBUG=false;
  if (DEBUG) {
    Serial.print("  c: ");
    Serial.print(c[0], 1);   Serial.print(", ");
    Serial.print(c[1], 1);   Serial.print(", ");
    Serial.print(c[2], 1);   Serial.print(", ");
    Serial.print("  b: ");
    Serial.print(brightness, 1);   Serial.print(", ");
    Serial.print("  t: ");
    Serial.print(t, 1);   Serial.print("\n");
  }
}

/*
[drawline] handle first endpoint
  c: 15.0, 15.0, 15.0,   b: 0.5,   t: 0.0
[drawline] handle second endpoint
  c: 100.0, 30.0, 30.0,   b: 0.5,   t: 1.0
[drawline] main loop
  c: 11.3, 11.3, 11.3,   b: 0.4,   t: 0.0
  c: 18.8, 18.8, 18.8,   b: 0.6,   t: 0.0
  c: 43.7, 26.3, 26.3,   b: 0.8,   t: 0.2
  c: 14.6, 8.8, 8.8,   b: 0.3,   t: 0.2
  c: 10.8, 5.0, 5.0,   b: 0.1,   t: 0.3
  c: 75.8, 35.0, 35.0,   b: 0.9,   t: 0.3
  c: 57.5, 22.5, 22.5,   b: 0.5,   t: 0.5
  c: 57.5, 22.5, 22.5,   b: 0.5,   t: 0.5
  c: 125.4, 43.7, 43.7,   b: 0.9,   t: 0.7
  c: 17.9, 6.2, 6.2,   b: 0.1,   t: 0.7
  c: 42.9, 13.8, 13.8,   b: 0.3,   t: 0.8
  c: 128.8, 41.2, 41.2,   b: 0.8,   t: 0.8
  c: 125.0, 37.5, 37.5,   b: 0.6,   t: 1.0
  c: 75.0, 22.5, 22.5,   b: 0.4,   t: 1.0
  up: 0.0, 0.9, -0.4
 */

void drawAALine(double x0,double y0,double x1,double y1, const suba::Vec3& c1, const suba::Vec3& c2) {
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    bool swapped_xy = false;
    bool swapped_endpoints = false;
    const bool DEBUG=false;
    
    if (steep) {
      AntiAliasLine::swap(&x0, &y0);
      AntiAliasLine::swap(&x1, &y1);
      swapped_xy = true;
    }
    if (x0 > x1) {
      AntiAliasLine::swap(&x0, &x1);
      AntiAliasLine::swap(&y0, &y1);
      swapped_endpoints = true;
    }
    
    double dx = x1 - x0;
    double dy = y1 - y0;
    double gradient;
    double xend, yend;
    double xgap, xpxl1, ypxl1, xpxl2, ypxl2, intery;
    const bool aa_the_endpoints = false;

    if (dx == 0.0)
        gradient = 1.0;
    else
        gradient = dy / dx;

    // handle first endpoint
    DEBUG && Serial.print("[drawline] handle first endpoint\n");
    xend = AntiAliasLine::roundToNearestInt(x0);
    yend = y0 + gradient * (xend - x0);
    xgap = (aa_the_endpoints ? rfpart(x0 + 0.5) : 1.0); // xgap seems to be used to dim/antialias the endpoints
    xpxl1 = xend; // this will be used in the main loop
    ypxl1 = AntiAliasLine::ipart(yend);
    if (steep) {
        plot(ypxl1,   xpxl1, AntiAliasLine::rfpart(yend) * xgap, c1, c2, swapped_endpoints ? 1 : 0);
        plot(ypxl1+1, xpxl1,  AntiAliasLine::fpart(yend) * xgap, c1, c2, swapped_endpoints ? 1 : 0);
    } else {
        plot(xpxl1, ypxl1  , AntiAliasLine::rfpart(yend) * xgap, c1, c2, swapped_endpoints ? 1 : 0);
        plot(xpxl1, ypxl1+1,  AntiAliasLine::fpart(yend) * xgap, c1, c2, swapped_endpoints ? 1 : 0);
    }
    intery = yend + gradient; // first y-intersection for the main loop
    
    // handle second endpoint
    DEBUG && Serial.print("[drawline] handle second endpoint\n");
    xend = AntiAliasLine::roundToNearestInt(x1);
    yend = y1 + gradient * (xend - x1);
    xgap = (aa_the_endpoints ? fpart(x1 + 0.5) : 1.0); // xgap seems to be used to dim/antialias the endpoints
    xpxl2 = xend; //this will be used in the main loop
    ypxl2 = AntiAliasLine::ipart(yend);
    if (steep) {
        plot(ypxl2  , xpxl2, AntiAliasLine::rfpart(yend) * xgap, c1, c2, swapped_endpoints ? 0 : 1);
        plot(ypxl2+1, xpxl2,  AntiAliasLine::fpart(yend) * xgap, c1, c2, swapped_endpoints ? 0 : 1);
    } else {
        plot(xpxl2, ypxl2,  AntiAliasLine::rfpart(yend) * xgap, c1, c2, swapped_endpoints ? 0 : 1);
        plot(xpxl2, ypxl2+1, AntiAliasLine::fpart(yend) * xgap, c1, c2, swapped_endpoints ? 0 : 1);
    }

    // main loop
    DEBUG && Serial.print("[drawline] main loop\n");
    double t = 0;
    if (steep) {
      double x_start = (xpxl1 + 1.0);
      double x_end = (xpxl2 - 1.0);
      for (double x=x_start; x <= x_end; ++x) {
        t = (x - x_start) / (x_end - x_start);
        t = swapped_endpoints ? (1.0-t) : t;
        plot(AntiAliasLine::ipart(intery)  , x, AntiAliasLine::rfpart(intery), c1, c2, t);
        plot(AntiAliasLine::ipart(intery)+1, x,  AntiAliasLine::fpart(intery), c1, c2, t);
        intery = intery + gradient;
      }
    } else {
      double x_start = (xpxl1 + 1.0);
      double x_end = (xpxl2 - 1.0);
      for (double x=x_start; x <= x_end; ++x) {
        t = (x - x_start) / (x_end - x_start);
        t = swapped_endpoints ? (1.0-t) : t;
        plot(x, AntiAliasLine::ipart(intery),  AntiAliasLine::rfpart(intery), c1, c2, t);
        plot(x, AntiAliasLine::ipart(intery)+1, AntiAliasLine::fpart(intery), c1, c2, t);
        intery = intery + gradient;
      }
    }
}

} // namespace
} // namespace

#endif
