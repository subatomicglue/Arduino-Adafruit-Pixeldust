#ifndef SUBA_PERSISTENT_LOCALSTORE_H
#define SUBA_PERSISTENT_LOCALSTORE_H

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subatomic's LocalStore - Key/Value Persistent local store
// - Persistent storage on top of the following APIs/hardware:
//   - Adafruit_SPIFlash formatted with SdFat (FAT FS)
//     - SPI, QSPI, ESP32, RP2040
//     - e.g. matrixportal M4
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

// Since SdFat doesn't fully support FAT12 such as format a new flash
// We will use Elm Cham's fatfs f_mkfs() to format
#include "ff.h"
#include "diskio.h"

// up to 11 characters
#define DISK_LABEL    "EXT FLASH"

namespace suba {

// Uncomment to run example with custom SPI and SS e.g with FRAM breakout
// #define CUSTOM_CS   A5
// #define CUSTOM_SPI  SPI

#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);

#elif defined(ARDUINO_ARCH_ESP32)
  // ESP32 use same flash device that store code.
  // Therefore there is no need to specify the SPI and SS
  Adafruit_FlashTransport_ESP32 flashTransport;

#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 use same flash device that store code.
  // Therefore there is no need to specify the SPI and SS
  // Use default (no-args) constructor to be compatible with CircuitPython partition scheme
  Adafruit_FlashTransport_RP2040 flashTransport;

  // For generic usage: Adafruit_FlashTransport_RP2040(start_address, size)
  // If start_address and size are both 0, value that match filesystem setting in
  // 'Tools->Flash Size' menu selection will be used

#else
  // On-board external flash (QSPI or SPI) macros should already
  // defined in your board variant if supported
  // - EXTERNAL_FLASH_USE_QSPI
  // - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;
  
  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
  
  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif
#endif

  Adafruit_SPIFlash flash(&flashTransport);
  
  // file system object from SdFat
  FatFileSystem fatfs;
  
  // Elm Cham's fatfs objects
  FATFS elmchamFatfs;
  uint8_t workbuf[4096]; // Working buffer for f_fdisk function.

  // Key/Value local store (on top of Adafruit_SPIFlash as a SdFat / FatFs)
  // Configuration filename is the key, value can be simple basic types and strings. (similar to Javascript browser localStorage)
  // You can build complex configurations on top of the string write interface, of course (json, .ini, etc).
  // Automatically formats your SDCARD filesystem if it isn't already (batteries included), (e.g. for QSPI, SPI, ESP32, matrixportal M4, etc).
  //
  // Arduino IDE install instructions: 
  //  - Requires "SdFat - Adafruit Fork" and "Adafruit SPIFlash" to be installed via the Library Manager
  // Documentation:
  // - SPIFlash reference:  https://github.com/adafruit/Adafruit_SPIFlash/tree/master/examples
  // - File    API reference: ~/Documents/Arduino/libraries/SdFat_-_Adafruit_Fork/src/FatLib/ArduinoFiles.h
  // - File Open flags: ~/Documents/Arduino/libraries/SdFat_-_Adafruit_Fork/src/FatLib/FatApiConstants.h   (You usually want O_RDONLY and (O_WRONLY | O_CREAT | O_TRUNC)... or (O_WRONLY | O_AT_END) or (O_WRONLY | O_APPEND) )
  //                    ~/Documents/Arduino/libraries/SdFat_-_Adafruit_Fork/src/FatLib/ArduinoFiles.h
  // - FatFile API reference: ~/Documents/Arduino/libraries/SdFat_-_Adafruit_Fork/src/FatLib/FatFile.h
  // - Adafruit_SPIFlash API reference: ~/Documents/Arduino/libraries/Adafruit_SPIFlash/src/Adafruit_SPIFlash.h
  struct LocalStore {

    // will be true after init() if the flash successfully init'd
    bool flash_init = false;

    // will be true after init() if the fatfs successfully init'd
    bool fatfs_init = false;

    // call init() in your sketch's start()
    void init() {
      // Initialize flash library and check its chip ID.
      if (!flash.begin()) {
        Serial.println("Error, failed to initialize flash chip!");
        while(1) delay(1);
      }
      flash_init = true;
      Serial.print("[FileSystemConfig][Info]  Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

      // erase(true); // uncomment for testing  (WARNING, will cause your flash to repeatedly erased and reformatted every sketch start(), causing wear and tear on your limited resource).

      if ( !fatfs.begin(&flash) ) {
        Serial.println("[FileSystemConfig][Error] filesystem does not exist. Trying SdFat_format example to make one.");
        Serial.println("[FileSystemConfig][Info]  OMG - WE ARE GOING TO FORMAT YOUR FLASH MEMORY ... NOW!");
        format( true ); // if any failure, will hang with a TTY message
      }
      fatfs_init = true;
    }

    void format( bool force = false ) {
#ifndef SUBA_CONFIG_AUTO_FORMAT_FATFS
      Serial.println("[FileSystemConfig][Error] Must define SUBA_CONFIG_AUTO_FORMAT_FATFS and rerun to format.");
      Serial.println("                          If you have code size issues, then, once formatted, un-define SUBA_CONFIG_AUTO_FORMAT_FATFS.");
      while(1) yield();
#endif
#ifdef SUBA_CONFIG_AUTO_FORMAT_FATFS
      if (!flash_init) {
        Serial.println("[FileSystemConfig][Error] Must call init() before calling format()");
        while(1) yield();
      }
      Serial.print("[FileSystemConfig][Info]  Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);
    
      // Uncomment to flash LED while writing to flash
      // flash.setIndicator(LED_BUILTIN, true);
    
      // Wait for user to send OK to continue.
      if (!force) {
        Serial.setTimeout(30000);  // Increase timeout to print message less frequently.
        do {
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          Serial.println("This sketch will ERASE ALL DATA on the flash chip and format it with a new filesystem!");
          Serial.println("Type OK (all caps) and press enter to continue.");
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } while ( !Serial.find((char*) "OK"));
      }
      
      // Call fatfs begin and passed flash object to initialize file system
      Serial.println("[FileSystemConfig][Info]  Creating and formatting FAT filesystem (this takes ~60 seconds)...");
    
      // Make filesystem.
      FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf, sizeof(workbuf));
      if (r != FR_OK) {
        Serial.print("[FileSystemConfig][Error] f_mkfs failed with error code: "); Serial.println(r, DEC);
        while(1) yield();
      }
    
      // mount to set disk label
      r = f_mount(&elmchamFatfs, "0:", 1);
      if (r != FR_OK) {
        Serial.print("[FileSystemConfig][Error] f_mount failed with error code: "); Serial.println(r, DEC);
        while(1) yield();
      }
    
      // Setting label
      Serial.println("Setting disk label to: " DISK_LABEL);
      r = f_setlabel(DISK_LABEL);
      if (r != FR_OK) {
        Serial.print("[FileSystemConfig][Error] f_setlabel failed with error code: "); Serial.println(r, DEC);
        while(1) yield();
      }
    
      // unmount
      f_unmount("0:");
    
      // sync to make sure all data is written to flash
      flash.syncBlocks();
      
      Serial.println("[FileSystemConfig][Info]  Formatted flash!");
    
      // Check new filesystem
      if (!fatfs.begin(&flash)) {
        Serial.println("[FileSystemConfig][Error] failed to mount newly formatted filesystem!");
        while(1) delay(1);
      }
    
      // Done!
      Serial.println("[FileSystemConfig][Info]  Flash chip successfully formatted with new empty filesystem!");
#endif
    }

    void erase( bool force = false ) {
#ifndef SUBA_CONFIG_AUTO_FORMAT_FATFS
      Serial.println("[FileSystemConfig][Error] Must define SUBA_CONFIG_AUTO_FORMAT_FATFS and rerun to format.");
      Serial.println("                          If you have code size issues, then, once formatted, un-define SUBA_CONFIG_AUTO_FORMAT_FATFS.");
      while(1) yield();
#endif
#ifdef SUBA_CONFIG_AUTO_FORMAT_FATFS
      if (!flash_init) {
        Serial.println("[FileSystemConfig][Error] Must call init() before calling erase()");
        while(1) yield();
      }
      Serial.print("[FileSystemConfig][Info]  Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

      // Wait for user to send OK to continue.
      if (!force) {
        Serial.setTimeout(30000);  // Increase timeout to print message less frequently.
        do {
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          Serial.println("This sketch will ERASE ALL DATA on the flash chip!");
          Serial.println("Type OK (all caps) and press enter to continue.");
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
        while (!Serial.find((char*) "OK"));
      }
 
      Serial.println("[FileSystemConfig][Info]  Erasing flash chip in 10 seconds...");
      Serial.println("[FileSystemConfig][Info]  Note you will see stat and other debug output printed repeatedly.");
      Serial.println("[FileSystemConfig][Info]  Let it run for ~30 seconds until the flash erase is finished.");
      Serial.println("[FileSystemConfig][Info]  An error or success message will be printed when complete.");
    
      if (!flash.eraseChip()) {
        Serial.println("[FileSystemConfig][Error] Failed to erase chip!");
      }
    
      flash.waitUntilReady();
      Serial.println("[FileSystemConfig][Info]  Successfully erased chip!");
#endif
    }


    // write any basic datatype (char, int, bool, etc) to a file named <key>
    template <typename T>
    void setItem( const char* key, const T& value ) {
      if (!flash_init || !fatfs_init) {
        Serial.println("[FileSystemConfig][Error] Must call init() before calling write()");
        while(1) yield();
      }
      File f = fatfs.open(key, O_WRONLY | O_CREAT | O_TRUNC);
      if (f) {
        f.write( (uint8_t*)&value, sizeof(value) );
        f.close();
        f.sync(); // close should force cached data and directory information to be written to the storage device.
        Serial.print("[FileSystemConfig][Info]  wrote \""); Serial.print(key); Serial.print("\" with data: ["); Serial.print(value); Serial.print("]\n"); 
      } else {
        Serial.print("[FileSystemConfig][Error] opening file \""); Serial.print(key); Serial.print("\" for writing FAILED. (must call init() first.  requires \"SdFat - Adafruit Fork\" and \"Adafruit SPIFlash\" to be installed via the Library Manager)\n");
      }
    }

    // read any basic datatype (char, int, bool, etc) from a file named <key>, if file isn't created, read the <default_value> instead
    template <typename T>
    void getItem( const char* key, T& value, const T& default_value ) {
      if (!flash_init || !fatfs_init) {
        Serial.println("Error must call init() before calling write()");
        while(1) yield();
      }
      File f = fatfs.open( key, O_RDONLY );
      if (f) {
        f.read( (uint8_t*)&value, sizeof(value) );
        f.close();
        f.sync(); // close should force cached data and directory information to be written to the storage device.
        Serial.print("[FileSystemConfig][Info]  while opening file \""); Serial.print(key); Serial.print("\" for reading, we read "); Serial.print(value); Serial.print("\n"); 
      } else {
        value = default_value;
        Serial.print("[FileSystemConfig][Info]  while opening file \""); Serial.print(key); Serial.print("\" for reading, using default value "); Serial.print(default_value); Serial.print("\n"); 
      }
    }

    // write a string to a file named <key>
    void setItem( const char* key, const std::string& value ) {
      File f = fatfs.open( key, O_WRONLY | O_CREAT | O_TRUNC );
      if (f) {
        f.write( (const char*)value.c_str(), value.size()+1 ); // +1 for the terminator \0
        f.close();
        f.sync(); // close should force cached data and directory information to be written to the storage device.
        Serial.print("[FileSystemConfig][Info] wrote \""); Serial.print(key); Serial.print("\" with \""); Serial.print(value.c_str()); Serial.print("\"\n"); 
      } else {
        Serial.print("[FileSystemConfig][Error] opening file \""); Serial.print(key); Serial.print("\" for writing FAILED. (must call init() first.  requires \"SdFat - Adafruit Fork\" and \"Adafruit SPIFlash\" to be installed via the Library Manager)\n");
      }
    }

    // write a string to a file named <key>
    void setItem( const char* key, const char* value ) {
      setItem( key, std::string( value ) );
    }
    
    // read string from a file named <key>, if file isn't created, read the <default_value> instead
    void getItem( const char* key, std::string& value, const std::string& default_value ) {
      File f = fatfs.open( key, O_RDONLY );
      if (f) {
        size_t l = f.fileSize();
        //f.seekEnd();
        //size_t l = f.curPosition();
        //f.seekSet( 0 );
        value.resize( l-1 ); // strings are sized without the terminator \0, internally will +1 for the \0
        if (value.size() == (l-1))
        {
          f.read( (char*)&value[0], value.size() + 1 ); // +1 for the terminator \0
        }
        f.close();
        f.sync(); // close should force cached data and directory information to be written to the storage device.
        Serial.print("[FileSystemConfig][Info] while opening file \""); Serial.print(key); Serial.print("\" for reading, we read \""); Serial.print(value.c_str()); Serial.print("\"\n"); 
      } else {
        value = default_value;
        Serial.print("[FileSystemConfig][Info] opening file \""); Serial.print(key); Serial.print("\" for reading, using default value "); Serial.print(default_value.c_str()); Serial.print("\n"); 
      }
    }
  };

} // end namespace

//--------------------------------------------------------------------+
// fatfs diskio glue
//--------------------------------------------------------------------+
extern "C"
{

DSTATUS disk_status ( BYTE pdrv )
{
  (void) pdrv;
  return 0;
}

DSTATUS disk_initialize ( BYTE pdrv )
{
  (void) pdrv;
  return 0;
}

DRESULT disk_read (
  BYTE pdrv,    /* Physical drive nmuber to identify the drive */
  BYTE *buff,   /* Data buffer to store read data */
  DWORD sector, /* Start sector in LBA */
  UINT count    /* Number of sectors to read */
)
{
  (void) pdrv;
  return suba::flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write (
  BYTE pdrv,      /* Physical drive nmuber to identify the drive */
  const BYTE *buff, /* Data to be written */
  DWORD sector,   /* Start sector in LBA */
  UINT count      /* Number of sectors to write */
)
{
  (void) pdrv;
  return suba::flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl (
  BYTE pdrv,    /* Physical drive nmuber (0..) */
  BYTE cmd,   /* Control code */
  void *buff    /* Buffer to send/receive control data */
)
{
  (void) pdrv;

  switch ( cmd )
  {
    case CTRL_SYNC:
      suba::flash.syncBlocks();
      return RES_OK;

    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = suba::flash.size()/512;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *((WORD*) buff) = 512;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 8;    // erase block size in units of sector size
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

}

#endif
