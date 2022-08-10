# [Fork] of Adafruit's "Pixel dust" Protomatter library example For ADAFRUIT's MATRIXPORTAL M4.

This is a 64x32 RGB color LED matrix running an interactive sand particle simulation that responds to moving and tilting the device.

See Adafruit's original "pixeldust" project [here](https://learn.adafruit.com/adafruit-matrixportal-m4/arduino-pixel-dust-demo), and the original [source code](https://github.com/adafruit/Adafruit_Protomatter/tree/master/examples/pixeldust).

- New Features in this Fork:
  - 6 color themes
  - screensaver makes this a nice passive desk toy
  - gravity arrow visualization
  - tap and button interaction to change settings
  - persistent storage of changed settings

## Details:
  - Added new color themes
    - Added some nicer, more stylish palettes than the default "rainbow" color theme.
    - User can cycle through them with a Single Tap.
    - Setting is persistent, and restored on next reboot
  - Gravity arrow on screen with antialiased line drawing.
    - Rotates smoothly to chase a goal vector exponentially (fast then slowly easing, imagine moving 1/2 the distance each time).  Snappy-smooth response.
    - Anti Aliased line drawing using Xiaolin Wu's algorithm.  Color per vertex.  Color interpolation.
    - UP and DOWN side buttons toggle the Gravity Arrow visualization
      - Can be set to show separately in both interactive mode and non-interactive mode
      - Settings are persistent, and restored on next reboot
  - Single/Double tap (with debounce, edge, and single-tap filtering)
    - *single:*  changes color theme/bank
    - *double:*  alternate between randomize particles, or, reset the particles nicely in color ordering.
    - *Debounce Filtering* removes noise in the stream of inputs from the button.
    - *Edge filtering* makes it easy to know when something is ON, JUST ON, OFF or JUST OFF.
    - *SingleTap Filter* The hardware gives you single or double tap detection, I did not have to implement this.   but the hardware does not cancel the single tap when a double tap is detected.   So I added a filter to cancel single tap detection when a double click was detected - which introduces a delay in the single tap (rightfully so).
  - Detect inactivity on Accelerometer values
    - Low Pass Filter on accelerometer gravity[x,y,z] individual coords
    - Low Pass Filter on the angle between prev frame and current frame gravity vectors
    - When angle is < 0.3 we know the device is “not moving”...   start the screensaver timer
  - Screensaver
    - Throw the sand around periodically by manipulating the gravity vector.
    - Screensaver has a 15 second timeout.
    - With a 1s loading bar right before saver engages.
    - Gravity vector changes by 195 deg or inverts (cycle between).   Gets some good particle action while idle.
  - Ending flash when you go back to interactive mode
  - `LocalStore` key/value persistent storage for configuration
    - API somewhat behaves like Javascript's `window.localStorage.getItem(key)` and `setItem( key, value )`
    - If FAT FS is not detected, then will auto-format your M4's flash data into a FAT fs ready to be used for read/write of persistent config


## Setup
First, perform the base MatrixPortalM4 Arduino IDE setup [here](https://learn.adafruit.com/adafruit-matrixportal-m4/arduino-ide-setup):
- Open `Tools / Library Manager` and install:
  - Adafruit NeoPixel
  - Adafruit SPIFlash
  - Adafruit Protomatter
  - Adafruit LIS3DH
    - Adafruit BusIO
    - Adafruit Unified Sensor
  - Adafruit GFX Library
  - Adafruit ImageReader
  - Adafruit PixelDust
- Open `Sketch / Include Library` and install:
  - WiFiNINA  (get adafruit's fork here: `https://github.com/adafruit/WiFiNINA/archive/master.zip`, add the `WiFiNINA.zip` file)
    - THIS IS IMPORTANT: The official WiFi101 library won't work because it doesn't support the ability to change the pins.

Then, install the following to use the MatrixPortalM4's Flash for persistent settings (needed by [subaLocalStore.h](subaLocalStore.h)):
- Open `Tools / Library Manager` and install:
  - SdFat - Adafruit Fork
  - Adafruit SPIFlash

You should be able to open and build the `ino` sketch now.

## Build
- Plug in your MatrixPortalM4 via USB data cable to the computer
- Start the Arduino IDE
- Select under `Tools / Board / "Adafruit MatrixPortal M4 (SAMD51)"`
- In a terminal, find your serial port
  - In a terminal (MacOS): `ls /dev/tty.*` to find your port (e.g. `/dev/tty.usbmodem14244401`)
  - You can plug/unplug it, to notice which one it is...
- Select the `Tools / Port /  "/dev/tty.usbmodem14244401"`
- Select `Sketch / Verify/Compile`, then `Sketch / Upload`
- (enjoy?)

# Also cool
- [3D printed handles](https://learn.adafruit.com/matrix-portal-sand/assembly)


