/* ----------------------------------------------------------------------
"Pixel dust" Protomatter library example (with subatomic additions).
As written, this is SPECIFICALLY FOR THE ADAFRUIT MATRIXPORTAL M4
with 64x32 pixel matrix.

Change "HEIGHT" below for 64x64 matrix. Could also be adapted to other
Protomatter-capable boards with an attached LIS3DH accelerometer.

PLEASE SEE THE "simple" EXAMPLE FOR AN INTRODUCTORY SKETCH,
or "doublebuffer" for animation basics.

Docs:
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-hallowing-m4.pdf
------------------------------------------------------------------------- */

#include <Wire.h>                 // For I2C communication
#include <Adafruit_LIS3DH.h>      // For accelerometer
#include <Adafruit_PixelDust.h>   // For sand simulation
#include <Adafruit_Protomatter.h> // For RGB matrix
#define SUBA_FILTER_DISABLE_CLAMPING
#define SUBA_CONFIG_AUTO_FORMAT_FATFS
#include "suba.h"                 // Subatomic Math / Filters
#include "subaLocalStore.h"       // Subatomic Persistent Configuration LocalStore for Adafruit SPIFlash as a FatFs

#define HEIGHT  32 // Matrix height (pixels) - SET TO 64 FOR 64x64 MATRIX!
#define WIDTH   64 // Matrix width (pixels)
#define MAX_FPS 45 // Maximum redraw rate, frames/second
#define TO_RAD (3.1459 / 180.0)
#define TO_DEG (180.0 / 3.1459)

#if HEIGHT == 64 // 64-pixel tall matrices have 5 address lines:
uint8_t addrPins[] = {17, 18, 19, 20, 21};
#else            // 32-pixel tall matrices have 4 address lines:
uint8_t addrPins[] = {17, 18, 19, 20};
#endif

// Remaining pins are the same for all matrix sizes. These values
// are for MatrixPortal M4. See "simple" example for other boards.
uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
uint8_t clockPin   = 14;
uint8_t latchPin   = 15;
uint8_t oePin      = 16;
uint8_t BUTTON_UP  =2; // MATRIXPORTAL M4's up button
uint8_t BUTTON_DOWN=3; // MATRIXPORTAL M4's down button

// LED MatrixPortal Drawing
Adafruit_Protomatter matrix(
  WIDTH, 4, 1, rgbPins, sizeof(addrPins), addrPins,
  clockPin, latchPin, oePin, true);

// Antialiased Line Drawing (which uses the matrix variable above, must include after)
#include "subaAntiAliasLine.h" // Subatomic line drawing, uses matrix and suba::Vec3 to draw lines...

// Box Drawing (which uses matrixportal)
void drawBox( int startx, int starty, int sizex, int sizey, const uint8_t* color3 ) {
  for (int x = 0; x < sizex; ++x) {
    matrix.drawPixel( startx + x, starty, matrix.color565(color3[0], color3[1], color3[2]) );
    matrix.drawPixel( startx + x, starty + (sizey-1), matrix.color565(color3[0], color3[1], color3[2]) );
  }

  for (int y = 1; y < (sizey-1); ++y) {
    matrix.drawPixel( startx, starty + y, matrix.color565(color3[0], color3[1], color3[2]) );
    matrix.drawPixel( startx + (sizex-1), starty + y, matrix.color565(color3[0], color3[1], color3[2]) );
  }
}

// accelerometer
Adafruit_LIS3DH accel = Adafruit_LIS3DH(); // https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/overview

#define N_COLORS   8
#define N_BANKS  6
#define BOX_HEIGHT 8
#define N_GRAINS (BOX_HEIGHT*N_COLORS*8)
int current_bank=0;
uint16_t colors[N_COLORS][N_BANKS];

// Sand Simulation - https://github.com/adafruit/Adafruit_PixelDust/blob/master/Adafruit_PixelDust.h
Adafruit_PixelDust sand(WIDTH, HEIGHT, N_GRAINS, 1, 128, false);

uint32_t prevTime = 0; // Used for frames-per-second throttle

// low pass filters / filter HF noise
suba::OveKarlsonResFilter x_filter; // lowpass filter for fx
suba::OveKarlsonResFilter y_filter; // lowpass filter for fy
suba::OveKarlsonResFilter z_filter; // lowpass filter for fz
suba::OveKarlsonResFilter a_filter; // lowpass filter for fa

// tap input filters
suba::DebounceFilter single_tap_df;
suba::DebounceFilter double_tap_df;
suba::EdgeFilter single_tap_ef;
suba::EdgeFilter double_tap_ef;
suba::SingleClickFilter single_click_scf;
suba::DebounceFilter button_up_df;
suba::DebounceFilter button_down_df;
suba::EdgeFilter button_up_ef;
suba::EdgeFilter button_down_ef;

// screensaver on/off filtering (edge filter makes it easy to know when SS starts and ends)
suba::EdgeFilter screensaver_filter;

// persistent configuration key/value store
suba::LocalStore conf;

// screensaver routine "flips" between states... keep track
suba::Timer flip_timer( 5.0 );
int flip_state = 0;
int flip_state2 = 0;
suba::Vec3 flip_vec( 0, 1, 0 );
double flip_gravity = 9.8;
bool show_gravity_arrow_interactive=true;
bool show_gravity_arrow_noninteractive=true;
bool show_gravity_arrow=true;

double xx, yy, zz;                  // gravity vector read from the accelerometer * 1000 (drives the sand animation)
double fx = 0, fy = -1, fz = 0;     // low pass filtered version of xx, yy, zz
double fx_prev = 0, fy_prev = -1, fz_prev = 0;  // last frame's fx, fy, fz
double fa = 0;                      // low pass filtered angle between fx,fy,fz and the previous frame's
double dt = 0.0;                    // frame time in seconds
double dt_not_moving = 0.0;         // amount of time elapsed since MatrixPortal has not moved.
int double_tap_type = 0;            // toggle the meaning of double tap
double angle = 90.0;                // screensaver gravity angle, we'll move this around, and use it to compute a fake gravity vector to feed the sand sim
const double pointer_length = 9.0;  // length of the gravity arrow on screen
suba::Vec3 goal_arrow(0,1,0);       // the goal we'd like the onscreen arrow to move to
suba::Vec3 arrow(0,1,0);            // the current graphical arrow vector (which moves incrementally each frame towards the goal_arrow, to follow it)
const bool VERBOSE=true;            // turn on debug print

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------
void err(int x) {
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);       // Using onboard LED
  for(i=1;;i++) {                     // Loop forever...
    digitalWrite(LED_BUILTIN, i & 1); // LED on/off blink to alert user
    delay(x);
  }
}

void init_sand() {
  // Set up initial sand coordinates, in 8x8 blocks
  int n = 0;
  for(int i=0; i<N_COLORS; i++) {
    int xx = i * WIDTH / N_COLORS;
    int yy =  HEIGHT - BOX_HEIGHT;
    for(int y=0; y<BOX_HEIGHT; y++) {
      for(int x=0; x < WIDTH / N_COLORS; x++) {
        //Serial.printf("#%d -> (%d, %d)\n", n,  xx + x, yy + y);
        sand.setPosition(n++, xx + x, yy + y);
      }
    }
  }
  Serial.printf("%d total pixels\n", n);
}

void setup() {
  //while (!Serial) delay(10); // wait for serial to init (1. this is needed to show the first few print()s.  2. BUT, you can't run without a serial monitor or it'll hang here waiting for it :) )
  Serial.begin(115200);

  // initialize Persistent configuration store
  conf.init();
  conf.getItem( "arrow_interactive", show_gravity_arrow_interactive, false );
  conf.getItem( "arrow_noninteractive", show_gravity_arrow_noninteractive, true );
  conf.getItem( "color_bank", current_bank, 0 );

  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);

  float samprate=MAX_FPS;
  float cutoff=0.1f;
  float res=0.0f;
  x_filter.setLowPass();
  x_filter.setSampRate( samprate );
  x_filter.setCutoff( cutoff ); // low
  x_filter.setRes( res );
  y_filter.setLowPass();
  y_filter.setSampRate( samprate );
  y_filter.setCutoff( cutoff ); // low
  y_filter.setRes( res );
  z_filter.setLowPass();
  z_filter.setSampRate( samprate );
  z_filter.setCutoff( cutoff ); // low
  z_filter.setRes( res );
  a_filter.setLowPass();
  a_filter.setSampRate( samprate );
  a_filter.setCutoff( cutoff ); // low
  a_filter.setRes( res );
  
  ProtomatterStatus status = matrix.begin();
  Serial.printf("Protomatter begin() status: %d\n", status);

  if (!sand.begin()) {
    Serial.println("Couldn't start sand");
    err(1000); // Slow blink = malloc error
  }

  if (!accel.begin(0x19)) {
    Serial.println("Couldn't find accelerometer");
    err(250);  // Fast bink = I2C error
  }

  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  //
  // Adjust this number for the sensitivity of the 'click' force
  // this strongly depend on the range! for 16G, try 5-10
  // for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
  // if (click & 0x30)
  // {
  //   if (click & 0x10) Serial.print(" single click");
  //   if (click & 0x20) Serial.print(" double click");
  // }
  accel.setRange(LIS3DH_RANGE_8_G);   // 2, 4, 8 or 16 G!
  accel.setClick(2, 40);

  //sand.randomize();   // Initialize random sand positions
  init_sand();        // initialize orderly sand positions by color

  // setup color banks:
  int bank = -1;
  // 16 is the lowest color value it seems

  // grey/orange/red
  bank++;
  colors[0][bank] = matrix.color565( 25, 25, 25);     // dark grey
  colors[1][bank] = matrix.color565( 64, 64, 64);     // med grey
  colors[2][bank] = matrix.color565( 200, 200, 200);  // white
  colors[3][bank] = matrix.color565( 34, 20,  0);     // brown
  colors[4][bank] = matrix.color565( 64, 40,  0);     // brown yellow
  colors[5][bank] = matrix.color565(255,140,  0);     // yellow
  colors[6][bank] = matrix.color565(168, 38,  0);     // orange
  colors[7][bank] = matrix.color565(228,  3,  3);     // red

  // red/white/blue
  bank++;
  colors[0][bank] = matrix.color565(32,  3,  3);     // dark red
  colors[1][bank] = matrix.color565(100,  3,  3);    // med red
  colors[2][bank] = matrix.color565(228,  3,  3);    // red
  colors[3][bank] = matrix.color565( 25, 25, 25);    // dark grey
  colors[4][bank] = matrix.color565( 64, 64, 64);    // med grey
  colors[5][bank] = matrix.color565( 0, 20,  34);    // dark blue
  colors[6][bank] = matrix.color565( 0, 40,  64);    // med blue
  colors[7][bank] = matrix.color565( 0,140,  255);   // blue

  // blues monochromatic
  bank++;
  for (int x = 0; x < N_COLORS; ++x) {
    if (x < 4)
      colors[x][bank] = matrix.color565( 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))) );
    if (x >= 4)
      colors[x][bank] = matrix.color565( 16 + ((x-4)*(255/(N_COLORS))), 16 + ((x-4)*(255/(N_COLORS))), (x+4) < 2 ? 100 : 226 );
  }

  // reds monochromatic
  bank++;
  for (int x = 0; x < N_COLORS; ++x) {
    if (x < 4)
      colors[x][bank] = matrix.color565( 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))) );
    if (x >= 4)
      colors[x][bank] = matrix.color565( (x+4) < 2 ? 100 : 226, 16 + ((x-4)*(255/(N_COLORS))), 16 + ((x-4)*(255/(N_COLORS))) );
  }

  // greens monochromatic
  bank++;
  for (int x = 0; x < N_COLORS; ++x) {
    if (x < 4)
      colors[x][bank] = matrix.color565( 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))), 16 + (x*(255/(N_COLORS))) );
    if (x >= 4)
      colors[x][bank] = matrix.color565( 16 + ((x-4)*(255/(N_COLORS))), (x+4) < 2 ? 100 : 226, 16 + ((x-4)*(255/(N_COLORS))) );
  }

  // rainbow it up! (ow my eyes)
  bank++;
  colors[0][bank] = matrix.color565(64, 64, 64);  // Dark Gray
  colors[1][bank] = matrix.color565(120, 79, 23); // Brown
  colors[2][bank] = matrix.color565(228,  3,  3); // Red
  colors[3][bank] = matrix.color565(255,140,  0); // Orange
  colors[4][bank] = matrix.color565(255,237,  0); // Yellow
  colors[5][bank] = matrix.color565(  0,128, 38); // Green
  colors[6][bank] = matrix.color565(  0, 77,255); // Blue
  colors[7][bank] = matrix.color565(117,  7,135); // Purple
}


// MAIN LOOP - RUNS ONCE PER FRAME OF ANIMATION ----------------------------

void loop() {
  // Limit the animation frame rate to MAX_FPS.  Because the subsequent sand
  // calculations are non-deterministic (don't always take the same amount
  // of time, depending on their current states), this helps ensure that
  // things like gravity appear constant in the simulation.
  uint32_t t;
  while(((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  dt = (t - prevTime) / 1000000.0;
  prevTime = t;

  // Read accelerometer...
  sensors_event_t event;
  accel.getEvent(&event);
  //Serial.printf("(%0.1f, %0.1f, %0.1f)\n", event.acceleration.x, event.acceleration.y, event.acceleration.z);

  xx = event.acceleration.x * 1000.0;
  yy = event.acceleration.y * 1000.0;
  zz = event.acceleration.z * 1000.0;

  // detect single tap   (runs through: debounce filter, edge filter)
  uint8_t click = accel.getClick();
  bool double_tap_debounce = double_tap_df.filter( (click & 0x30) && (click & 0x20), dt );
  suba::EdgeFilter::EdgeState double_tap_filtered = double_tap_ef.filter( double_tap_debounce );

  // detect double tap   (runs through: debounce filter, edge filter, filter that cancels singleclick if doubleclick detected)
  bool single_tap_debounce = single_tap_df.filter( (click & 0x30) && (click & 0x10), dt );
  suba::EdgeFilter::EdgeState single_tap_edge = single_tap_ef.filter( single_tap_debounce );
  bool single_tap_filtered = single_click_scf.filter( single_tap_edge, dt, double_tap_filtered == suba::EdgeFilter::EDGE_DOWN );

  // detect button up
  bool button_up_debounce = button_up_df.filter( digitalRead(BUTTON_UP) == LOW, dt );
  suba::EdgeFilter::EdgeState button_up_edge = button_up_ef.filter( button_up_debounce );

  // detect button down
  bool button_down_debounce = button_down_df.filter( digitalRead(BUTTON_DOWN) == LOW, dt );
  suba::EdgeFilter::EdgeState button_down_edge = button_down_ef.filter( button_down_debounce );

  // handle single tap (cycle through color banks)
  if (single_tap_filtered == true) {
    current_bank = (current_bank + 1) % N_BANKS;
    conf.setItem( "color_bank", current_bank );
    double_tap_type = 1; // next time user resets the particles, we'll line up the colors nicely
  }
  // handle double tap (reset the particles)
  if (double_tap_filtered == suba::EdgeFilter::EDGE_DOWN) {
    switch (double_tap_type) {
      case 0:
        sand.clear();
        sand.randomize();
        break;
      case 1:
        sand.begin();
        sand.clear();
        init_sand();
        break;
    }
    double_tap_type = (double_tap_type + 1) % 2; // switch to the next type
  }
  
  // Update pixel data in LED driver
  dimension_t x, y;
  matrix.fillScreen(0x0);
  for(int i=0; i<N_GRAINS ; i++) {
    sand.getPosition(i, &x, &y);
    int n = i / ((WIDTH / N_COLORS) * BOX_HEIGHT); // Color index
    uint16_t flakeColor = colors[n][current_bank];
    matrix.drawPixel(x, y, flakeColor);
    //Serial.printf("(%d, %d)\n", x, y);
  }

  // Detect Idle/Not-Moving
  // filter the data to remove noise... (yep, sensors are noisy, they always are)
  // so we can tell when the device isn't moving around
  fx_prev = fx;
  fy_prev = fy;
  fz_prev = fz;
  suba::Vec3 up_vec = suba::normalize( suba::Vec3( x_filter.filter( event.acceleration.x ), y_filter.filter( event.acceleration.y ), z_filter.filter( event.acceleration.z ) ) );
  fx = up_vec[0]; // 1 == out the right of the matrixportalM4
  fy = up_vec[1]; // 1 == out the top of the matrixportalM4
  fz = up_vec[2]; // 1 == out the front of the matrixportalM4
  double aa = suba::dot_angle( suba::Vec3( fx, fy, fz ), suba::Vec3( fx_prev, fy_prev, fz_prev ) ) * TO_DEG;
  fa = a_filter.filter( aa );
  
  // NOT MOVING: based on no taps, and no rotation
  bool not_moving = double_tap_filtered == suba::EdgeFilter::UP && single_tap_filtered == false && fa < 0.3; // found by watching the values of fa while at rest...
  if (not_moving) {
    dt_not_moving += dt;   // keep track of how long we've been not-moving
  } else {
    dt_not_moving = 0;
  }

  // detect when screensaver activates
  const double screensaver_timeout = 15.0;
  suba::EdgeFilter::EdgeState screensaver_state = screensaver_filter.filter( screensaver_timeout < dt_not_moving );

  // animate a progress meter "loading" animation right before screensaver enables
  double screensaver_timeout_end = 1.0;// show the progress meter "loading" animation during the remaining X seconds of the screen_saver timeout
  double screensaver_timeout_start = screensaver_timeout - screensaver_timeout_end;
  if (not_moving && screensaver_timeout_start < dt_not_moving && dt_not_moving <= screensaver_timeout && screensaver_state == suba::EdgeFilter::UP) {
    double progress = (dt_not_moving - screensaver_timeout_start) / screensaver_timeout_end;
    //int y_line = 0;    // top
    //int y_line = 16;   // middle
    int y_line = 31;   // bottom
    suba::AntiAliasLine::drawAALine( 0, y_line, (int)floor(progress * 63), y_line, suba::Vec3( 16, 16, 16 ), suba::Vec3( 60,  60, 255 ) );
    //suba::AntiAliasLine::drawAALine( (int)floor(progress * 63), 31, (int)floor(progress * 63), 0, suba::Vec3( 16, 16, 16 ), suba::Vec3( 60,  60, 255 ) );
  }

  // when leaving screensaver, show a visual flash to clearly indicate to the user
  if (screensaver_state == suba::EdgeFilter::EDGE_UP) {
    const uint8_t box_color[3] = { 60, 60, 60 };
    drawBox( 0, 0, 64, 32, box_color );
  }

  // non-interactive mode (screensaver init)
  if (screensaver_state == suba::EdgeFilter::EDGE_DOWN) {
    suba::Vec3 up = suba::normalize( suba::Vec3( xx, yy, zz ) );
    double dir = (suba::cross( suba::Vec3( 1,0,0 ), up )[2] < 0.0 ? -1 : 1);
    angle = suba::dot_angle( suba::Vec3( 1,0,0 ), up ) * dir * TO_DEG;
    flip_timer.expireNow();

    if (VERBOSE) {
      Serial.print("SCREENSAVER STARTING");
      Serial.print("  MatrixportalM4's 'up' vector was: ");
      Serial.print(up[0]);   Serial.print(", ");
      Serial.print(up[1]);   Serial.print(", ");
      Serial.print(up[2]);   Serial.print(", ");
      Serial.print("  computed new angle from up vec: ");
      Serial.print(angle);   Serial.print(", ");
      Serial.print("  dir: ");
      Serial.print(dir);   Serial.print(", ");
      Serial.print("\n");
    }
  }

  // non-interactive mode (screensaver update)
  if (screensaver_state == suba::EdgeFilter::DOWN || screensaver_state == suba::EdgeFilter::EDGE_DOWN) {
    bool should_flip = flip_timer.update( dt );
    if (should_flip) {
      flip_state = (flip_state + 1) % 2;
      flip_state2 = flip_state == 0 ? ((flip_state2 + 1) % 2) : flip_state2;
      double min_gravity = 3.0;
      double max_gravity = 9.8;
      double min_timeout = 3;
      double max_timeout = 4;
      //angle = suba::rangeRandom( 0.0, 360.0 );
      angle = flip_state == 1 ? (angle + 180.0) : (angle + 180.0 - (flip_state2 ? 35.0 : 35.0));
      angle = std::fmod( angle, 360.0 );
      flip_vec[0] = cos( angle * TO_RAD );
      flip_vec[1] = sin( angle * TO_RAD );
      flip_vec[2] = 0.0;
      flip_vec = suba::normalize( flip_vec );
      //flip_gravity = suba::rangeRandom( min_gravity, max_gravity ); // random sucked, too often it'd do something not dramatic.
      //flip_timer.setTimeout( suba::rangeRandom( min_timeout, max_timeout ) );
      flip_gravity = flip_state == 0 ? max_gravity : min_gravity;
      flip_timer.setTimeout( flip_state == 0 ? min_timeout : max_timeout );

      if (VERBOSE) {
        Serial.print("  state: ");
        Serial.print(flip_state);   Serial.print(", ");
        Serial.print("  angle: ");
        Serial.print(angle);   Serial.print(", ");
        Serial.print("  flipvec: ");
        Serial.print(flip_vec[0], 1);   Serial.print(", ");
        Serial.print(flip_vec[1], 1);   Serial.print(", ");
        Serial.print(flip_vec[2], 1);   Serial.print(", ");
        Serial.print("  up: ");
        Serial.print(xx, 2);   Serial.print(", ");
        Serial.print(yy, 2);   Serial.print(", ");
        Serial.print(zz, 2);   Serial.print(", ");
        Serial.print("  gravity: ");
        Serial.print(flip_gravity, 1);   Serial.print(", ");
        Serial.print("  timer: ");
        Serial.print(flip_timer.mAge, 2);   Serial.print(", ");
        Serial.print(flip_timer.mTimeout, 2);   Serial.print(", ");
        Serial.print("\n");
      }
    }

    if (button_up_edge == suba::EdgeFilter::EDGE_DOWN || button_down_edge == suba::EdgeFilter::EDGE_DOWN) {
      show_gravity_arrow_noninteractive = !show_gravity_arrow_noninteractive;
      conf.setItem( "arrow_noninteractive", show_gravity_arrow_noninteractive );
    }

    goal_arrow = flip_vec;
    show_gravity_arrow = show_gravity_arrow_noninteractive;
    sand.iterate( flip_vec[0] * flip_gravity * 1000.0, flip_vec[1] * flip_gravity * 1000.0, flip_vec[2] * flip_gravity * 1000.0 );
  }

  // interactive mode (sand update)
  else {
    if (VERBOSE) {
      //Serial.print("dt: ");
      //Serial.print(dt); Serial.print(", ");
      //Serial.print("c: ");
      //Serial.print(current_bank); Serial.print(", ");
      //Serial.print("tap: ");
      //Serial.print((click & 0x30) && (click & 0x10)); Serial.print(", ");
      //Serial.print(single_tap_debounce); Serial.print(", ");
      //Serial.print(single_tap_edge); Serial.print(", ");
      //Serial.print(single_tap_filtered); Serial.print(", ");
      //Serial.print(double_tap_filtered); Serial.print(", ");
      
      Serial.print("  buttons: ");
      Serial.print(button_up_debounce); Serial.print(", ");
      Serial.print(button_down_debounce); Serial.print(", ");
      Serial.print(button_up_edge); Serial.print(", ");
      Serial.print(button_down_edge); Serial.print(", ");
      Serial.print("  up: ");
      Serial.print(suba::truncate( fx, 3 ), 3);   Serial.print(", ");
      Serial.print(suba::truncate( fy, 3 ), 3);   Serial.print(", ");
      Serial.print(suba::truncate( fz, 3 ), 3);   Serial.print(", ");
      Serial.print("  angle: ");
      Serial.print(fa, 6);   Serial.print("\n");
    }

    if (button_up_edge == suba::EdgeFilter::EDGE_DOWN || button_down_edge == suba::EdgeFilter::EDGE_DOWN) {
      show_gravity_arrow_interactive = !show_gravity_arrow_interactive;
      conf.setItem( "arrow_interactive", show_gravity_arrow_interactive );
    }

    goal_arrow = suba::normalize( suba::Vec3( xx, yy, zz ) );
    show_gravity_arrow = show_gravity_arrow_interactive;
    sand.iterate(xx, yy, zz);
  }

  // smooth animation of the gravity arrow
  arrow = slerp( arrow, goal_arrow, dt * 5.0 );
  if (show_gravity_arrow)
    suba::AntiAliasLine::drawAALine( 32, 16, 32 + round( arrow[0] * pointer_length ), 16 + round( arrow[1] * pointer_length ), suba::Vec3( 32, 16,  16 ), suba::Vec3( 255, 80,  80 ) );
  
  // Copy data to matrix buffers
  matrix.show();
}
