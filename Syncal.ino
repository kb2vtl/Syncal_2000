/*
  Syncal 
  Syncal 2000 display program

  Emulates original and now unobtainable Syncal 2000 LCD display allowing replacement 
  with new 122x32 graphics LCD display module which is available off the shelf.  

  This code uses wire.h to emulate the factory LCD driver chips in the radio.

  This code is in the public domain and may be freely used in the hope that it is 
  useful to someone either as-is or as part of another project.

  For application background and construction notes please see:
  http://petergottlieb.com/wordpress/?p=24

  Started 27 November 2016
  First full functionality 29 December 2016
  Changed status line to include battery voltage 30 Dec 16
  by Peter Gottlieb
*/

/**************************************************
 * Arduino Syncal 2000 display v1.0 - 2016/12/29
 * By Peter Gottlieb
 * This work is distributed under the GNU General 
 * Public License version 3 or later (GPL3+)
 * Please include this credit note if you want to 
 * re-use any part of this sketch. Respect my work 
 * as I'll do with yours.
 * Feel free to contact me: nerd@verizon.net
 * ************************************************/
#include <Wire.h>

byte segment;
bool cmdbyte = false;                       // is the incoming byte a command?
int DataPointer = 0;                        // PCF8576 data pointer
bool LCDbias;                               // LCD bias level, no purpose here
bool LCDpowerMode;                          // LCD normal or power saving mode, no purpose here
bool LCDenable;                             // LCD display enable
byte LCDmode;                               // LCD drive mode, mode 01 (single backplane) wired in radio
byte LCDdevice;                             // LCD device select
//bool LCDinBank;                             // PCF8576 RAM load bank select (now declared in receiveEvent)
//bool LCDoutBank;                            // PCF8576 RAM display bank select (now declared in receiveEvent)
bool LCDblinkMode;                          // LCD blink, normal or alternation
byte LCDblinkFreq;                          // LCD blink frequency, off and 3 choices
byte LCDdata[20];                           // LCD display data storage array
int LCDpointer = 0;                         // pointer into LCDdata
uint8_t MHz10;                              // 10 MHz digit
uint8_t MHz1;                               // 1 MHz digit
uint8_t kHz100;                             // 100 kHz digit
uint8_t kHz10;                              // 10 kHz digit
uint8_t kHz1;                               // 1 kHz digit
uint8_t Hz100;                              // 100 Hz digit
uint8_t MHz10W;                             // 10 MHz value already written to display
uint8_t MHz1W;                              // 1 MHz value already written to display
uint8_t kHz100W;                            // 100 kHz value already written to display
uint8_t kHz10W;                             // 10 kHz value already written to display
uint8_t kHz1W;                              // 1 kHz value already written to display
uint8_t Hz100W;                             // 100 Hz value already written to display
byte channel10;                             // 10's channel digit
uint8_t channel1;                           // 1's channel digit
uint8_t channel1W = 0x0F;                   // channel written to display
bool LCDdecimalpoint;                       // decimal point
bool LCDprog;                               // PROG
bool LCDRX;                                 // RX
bool LCDexternal;                           // EXTERNAL
bool LCDch;                                 // CH
bool LCDmhz;                                // MHz
bool LCDmhzW;                               // MHz has been written to display
bool LCDleftarrow;                          // left arrow
bool LCDrightarrow;                         // right arrow
bool LCDhi;                                 // HI
bool LCDm;                                  // M
bool LCDlo;                                 // LO
uint8_t LCDpowerW;                          // displayed power, 1=Lo, 2=Md, 3=Hi
bool LCDf;                                  // F
bool LCDk;                                  // K
uint8_t LCDbar;                             // 3 level bar chart for battery or transmit power
uint8_t LCDbarW;                            // 3 level bar written to display
//int db0pin = 0;                             // Digital IO bus pins to display
//int db1pin = 1;                             //
//int db2pin = 2;                             //
//int db3pin = 3;                             //
//int db4pin = 4;                             //
//int db5pin = 5;                             //
//int db6pin = 6;                             //
//int db7pin = 7;                             //
int a0pin = 13;                             // Data/Command pin to LCD, 0=command
int rwPin = 8;                              // R/W pin to LCD, 0=write
int ePin = 9;                               // E pin to LCD, see command routine
int clPin = 11;                             // Must be one of 3, 5, 6, 9, 10, or 11 for Arduino Uno (2 kHz nominal)
int cs1pin = 12;                            // LCD CS1 pin
int cs2pin = 10;                            // LCD CS2 pin
int m_sdl;
char hamBand = ' ';                         // ham band to display, space or G or X
char hamBandW;                              // ham band already written to display
bool clean = 0;                             // if true, BITE message so keep display clean
bool invMHz10 = 0;
bool invMHz1 = 0;
bool invKHz100 = 0;                         // these flags are used in programming mode where a digit is highlighted
bool invKHz10 = 0;                          // to indicate it is selected for change
bool invKHz1 = 0;
bool invHz100 = 0;
bool LCDmsgPass = 0;
bool LCDmsgTune = 0;
bool LCDmsgGood = 0;
bool LCDmsgPoor = 0;
bool LCDmsgFail = 0;
const int voltsFS = 15;                      // Full scale voltage for readout.  Corresponds to resistor divider to yield 5 volts FS to input
float volts;                                 // for battery voltage measurement and display
int volt10;
int volt1;
int voltV1;
bool TXlow;                                  // keyline sense for TX indication
bool invertV;                                // flag to invert voltage readout if out of limits


// bitmasks for commands, see PCF8576 data sheet
const byte maskC = B10000000;                // command continuation, means next byte is also a command byte
const byte maskL = B00010000;                // normal or power saving mode
const byte maskE = B00001000;                // display enabled bit
const byte maskB = B00000100;                // LCD bias select bit
const byte maskM = B00000011;                // mode set bits, 1, 2, 3 or 4 LCD backplanes
const byte maskP = B00111111;                // Data pointer set bits, range 0 through 39
const byte maskAd = B00000111;               // Chip address set bits
const byte maskI = B00000010;                // load data RAM bank select
const byte maskO = B00000001;                // display data RAM bank select
const byte maskA = B00000100;                // blink mode, normal or alternate
const byte maskF = B00000011;                // blink rate select
const byte bit7 = B10000000;
const byte bit6 = B01000000;
const byte bit5 = B00100000;
const byte bit4 = B00010000;
const byte bit3 = B00001000;
const byte bit2 = B00000100;
const byte bit1 = B00000010;
const byte bit0 = B00000001;
String disp = "01";                                            // no longer used but if removed get linker fault so leaving it in

const int ADDRESS = 57;                                        //I2C address of Syncal 2000 LCD driver

/// Chip selection
const uint8_t WGLCD_NOCHIP = 0;
const uint8_t WGLCD_CHIP1 = 1;
const uint8_t WGLCD_CHIP2 = 2;
const uint8_t WGLCD_CHIPBOTH = 3;

/// WG12232 command codes
const uint8_t WGLCD_CMD_RMW = 0xE0;                             // Start read-modify-write operation (verified against datasheet)
const uint8_t WGLCD_CMD_END_RMW = 0xEE;                         // End read-modify-right operation (verified against datasheet)
const uint8_t WGLCD_CMD_SOFT_RST = 0xE2;                        // Software reset (verified against datasheet)
const uint8_t WGLCD_CMD_DISP_ON = 0xAF;                         // Command to turn display on (verified against datasheet)
const uint8_t WGLCD_CMD_DISP_OFF = 0xAE;                        // Command to turn display off (verified against datasheet)
const uint8_t WGLCD_CMD_MAP_SEG = 0xA0;                         // Set column/segment map register.  Add in address 0-1 (verified against datasheet)
const uint8_t WGLCD_CMD_SET_SDL = 0xC0;                         // Set display start line register.  Add in address 0-31 (verified against datasheet)
const uint8_t WGLCD_CMD_SEL_PAGE = 0xB8;                        // Set page address register.  Add in address 0-3 (verified against datasheet)
const uint8_t WGLCD_CMD_SEL_COL = 0x00;                         // Set column address register.  Add in address 0-60 (verified against datasheet)

const uint8_t WGLCD_DTM_RW_BIT = 2;
const uint8_t WGLCD_DTM_CD_BIT = 1;

/// Data Transfer Mode
const uint8_t WGLCD_DTM_READDISPDATA = WGLCD_DTM_RW_BIT | WGLCD_DTM_CD_BIT;
const uint8_t WGLCD_DTM_WRITEDISPDATA = WGLCD_DTM_CD_BIT;
const uint8_t WGLCD_DTM_READSTATUS = WGLCD_DTM_RW_BIT;
const uint8_t WGLCD_DTM_WRITECMD = 0;

/// Segment mapping
const uint8_t WGLCD_SEGMAP_NORMAL = 0;
const uint8_t WGLCD_SEGMAP_INVERT = 1;

// Freq digits 0-9, 12x16 font
uint8_t freqDigits [] = 
  {
    0x00,0x00,0x07,0xF8,0x1F,0xFE,0x1E,0x06,0x33,0x03,0x31,0x83,0x30,0xC3,0x30,0x63,0x30,0x33,0x18,0x1E,0x1F,0xFE,0x07,0xF8,      // 0  12x16
    0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x0C,0x30,0x0C,0x30,0x0E,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00,      // 1  12x16
    0x00,0x00,0x30,0x1C,0x38,0x1E,0x3C,0x07,0x3E,0x03,0x37,0x03,0x33,0x83,0x31,0xC3,0x30,0xE3,0x30,0x77,0x30,0x3E,0x30,0x1C,      // 2  12x16
    0x00,0x00,0x0C,0x0C,0x1C,0x0E,0x38,0x07,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xE7,0x1F,0x7E,0x0E,0x3C,      // 3  12x16
    0x00,0x00,0x03,0xC0,0x03,0xE0,0x03,0x70,0x03,0x38,0x03,0x1C,0x03,0x0E,0x03,0x07,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x03,0x00,      // 4  12x16
    0x00,0x00,0x0C,0x3F,0x1C,0x7F,0x38,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x38,0xE3,0x1F,0xC3,0x0F,0x83,      // 5  12x16
    0x00,0x00,0x0F,0xC0,0x1F,0xF0,0x39,0xF8,0x30,0xDC,0x30,0xCE,0x30,0xC7,0x30,0xC3,0x30,0xC3,0x39,0xC3,0x1F,0x80,0x0F,0x00,      // 6  12x16
    0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x30,0x03,0x3C,0x03,0x0F,0x03,0x03,0xC3,0x00,0xF3,0x00,0x3F,0x00,0x0F,0x00,0x03,      // 7  12x16
    0x00,0x00,0x0F,0x00,0x1F,0xBC,0x39,0xFE,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00,      // 8  12x16
    0x00,0x00,0x00,0x3C,0x00,0x7E,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x38,0xC3,0x1C,0xC3,0x0E,0xC3,0x07,0xE7,0x03,0xFE,0x00,0xFC       // 9  12x16
  };
// Frequency decimal point
uint8_t fDecimal [] =
  {
    0x00,0x00,0x00,0x00,0xC0,0x00,0xC0,0x00,0x00,0x00,0x00,0x00
  };
uint8_t fMHz [] =                                                                                           // oh let's get rid of this already
  {
    0xE0,0x3F,0xE0,0x3F,0xC0,0x01,0x80,0x03,0xC0,0x01,0xE0,0x3F,0xE0,0x3F,0x00,0x00,      // M  8x12
    0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x00,0x02,0xE0,0x3F,0xE0,0x3F,0x00,0x00,0x00,0x00,      // H  8x12
    0x00,0x33,0x00,0x39,0x00,0x29,0x00,0x25,0x00,0x27,0x00,0x33,0x00,0x00,0x00,0x00       // z  8x12
  };
uint8_t smallText [] =                                                                                      // small characters we actually use
  {
    0x80,0x20,0x80,0x20,0xC0,0x3F,0xE0,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00,  // 1  8x12   1
    0xC0,0x30,0xE0,0x38,0x20,0x2C,0x20,0x26,0xE0,0x33,0xC0,0x31,0x00,0x00,0x00,0x00,  // 2  8x12   2
    0x40,0x10,0x60,0x30,0x20,0x22,0x20,0x22,0xE0,0x3F,0xC0,0x1D,0x00,0x00,0x00,0x00,  // 3  8x12   3
    0x00,0x06,0x00,0x07,0x80,0x05,0xC0,0x24,0xE0,0x3F,0xE0,0x3F,0x00,0x24,0x00,0x00,  // 4  8x12   4
    0xE0,0x13,0xE0,0x33,0x20,0x22,0x20,0x22,0x20,0x3E,0x20,0x1C,0x00,0x00,0x00,0x00,  // 5  8x12   5
    0x80,0x1F,0xC0,0x3F,0x60,0x22,0x20,0x22,0x20,0x3E,0x00,0x1C,0x00,0x00,0x00,0x00,  // 6  8x12   6
    0xE0,0x00,0xE0,0x00,0x20,0x38,0x20,0x3C,0x20,0x06,0xE0,0x03,0xE0,0x01,0x00,0x00,  // 7  8x12   7
    0xC0,0x1D,0xE0,0x3F,0x20,0x22,0x20,0x22,0xE0,0x3F,0xC0,0x1D,0x00,0x00,0x00,0x00,  // 8  8x12   8
    0xC0,0x01,0xE0,0x23,0x20,0x32,0x20,0x3E,0xE0,0x0F,0xC0,0x03,0x00,0x00,0x00,0x00,  // 9  8x12   9
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x20,0x20,0xE0,0x38,0xC0,0x18,0x00,0x00,  // C  8x12   C
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x22,0x20,0x02,0x60,0x07,0xE0,0x00,0x00,0x00,  // F  8x12   F
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x20,0x24,0xE0,0x3C,0xC0,0x3C,0x00,0x00,  // G  8x12   G
    0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x00,0x02,0xE0,0x3F,0xE0,0x3F,0x00,0x00,0x00,0x00,  // H  8x12   H
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x00,0x20,0x00,0x38,0x00,0x3C,0x00,0x00,  // L  8x12   L
    0xE0,0x3F,0xE0,0x3F,0xC0,0x01,0x80,0x03,0xC0,0x01,0xE0,0x3F,0xE0,0x3F,0x00,0x00,  // M  8x12   M
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x60,0x30,0xC0,0x1F,0x80,0x0F,0x00,0x00,  // O  8x12   O    
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x02,0x20,0x06,0xE0,0x3F,0xC0,0x39,0x00,0x00,  // R  8x12   R        
    0xE0,0x0F,0xE0,0x1F,0x00,0x30,0x00,0x30,0xE0,0x1F,0xE0,0x0F,0x00,0x00,0x00,0x00,  // V  8x12   V
    0xE0,0x38,0xE0,0x3D,0x00,0x07,0x00,0x07,0xE0,0x3D,0xE0,0x38,0x00,0x00,0x00,0x00,  // X  8x12   X
    0x00,0x18,0x00,0x3D,0x00,0x25,0x00,0x25,0x00,0x1F,0x00,0x3E,0x00,0x20,0x00,0x00,  // a  8x12   a
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x00,0x01,0x00,0x3F,0x00,0x3E,0x00,0x00,  // h  8x12   h
    0x00,0x00,0x00,0x21,0x00,0x21,0x60,0x3F,0x60,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,  // i  8x12   i
    0x00,0x3F,0x00,0x3F,0x00,0x01,0x00,0x01,0x00,0x3F,0x00,0x3E,0x00,0x00,0x00,0x00,  // n  8x12   n
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x00,0x21,0x00,0x3F,0x00,0x1E,0x00,0x00,0x00,0x00,  // o  8x12   o   
    0x00,0x21,0x00,0x33,0x00,0x1E,0x00,0x0C,0x00,0x1E,0x00,0x33,0x00,0x21,0x00,0x00,  // x  8x12   x
    0x00,0x33,0x00,0x39,0x00,0x29,0x00,0x25,0x00,0x27,0x00,0x33,0x00,0x00,0x00,0x00,  // z  8x12   z
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x20,0x21,0xE0,0x1F,0xE0,0x3F,0x00,0x20,0x00,0x00,  // d  8x12   d
    0xC0,0x1F,0xE0,0x3F,0x20,0x2C,0x20,0x27,0xA0,0x21,0xE0,0x3F,0xC0,0x1F,0x00,0x00,  // 0  8x12   0
    0x00,0x4E,0x00,0xDF,0x00,0x91,0x00,0x91,0x00,0xFE,0x00,0x7F,0x00,0x01,0x00,0x00,  // g  8x12   g
    0x60,0x00,0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x60,0x00,0x00,0x00,0x00,0x00   // T  8x12   T
  };    

/*                                                                                                            // all small characters, in case we need them later
    0xC0,0x1F,0xE0,0x3F,0x20,0x2C,0x20,0x27,0xA0,0x21,0xE0,0x3F,0xC0,0x1F,0x00,0x00,  // 0  8x12
    0x80,0x20,0x80,0x20,0xC0,0x3F,0xE0,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00,  // 1  8x12
    0xC0,0x30,0xE0,0x38,0x20,0x2C,0x20,0x26,0xE0,0x33,0xC0,0x31,0x00,0x00,0x00,0x00,  // 2  8x12
    0x40,0x10,0x60,0x30,0x20,0x22,0x20,0x22,0xE0,0x3F,0xC0,0x1D,0x00,0x00,0x00,0x00,  // 3  8x12
    0x00,0x06,0x00,0x07,0x80,0x05,0xC0,0x24,0xE0,0x3F,0xE0,0x3F,0x00,0x24,0x00,0x00,  // 4  8x12
    0xE0,0x13,0xE0,0x33,0x20,0x22,0x20,0x22,0x20,0x3E,0x20,0x1C,0x00,0x00,0x00,0x00,  // 5  8x12
    0x80,0x1F,0xC0,0x3F,0x60,0x22,0x20,0x22,0x20,0x3E,0x00,0x1C,0x00,0x00,0x00,0x00,  // 6  8x12
    0xE0,0x00,0xE0,0x00,0x20,0x38,0x20,0x3C,0x20,0x06,0xE0,0x03,0xE0,0x01,0x00,0x00,  // 7  8x12
    0xC0,0x1D,0xE0,0x3F,0x20,0x22,0x20,0x22,0xE0,0x3F,0xC0,0x1D,0x00,0x00,0x00,0x00,  // 8  8x12
    0xC0,0x01,0xE0,0x23,0x20,0x32,0x20,0x3E,0xE0,0x0F,0xC0,0x03,0x00,0x00,0x00,0x00,  // 9  8x12
    0x80,0x3F,0xC0,0x3F,0x60,0x04,0x60,0x04,0xC0,0x3F,0x80,0x3F,0x00,0x00,0x00,0x00,  // A  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x22,0x20,0x22,0xE0,0x3F,0xC0,0x1D,0x00,0x00,  // B  8x12
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x20,0x20,0xE0,0x38,0xC0,0x18,0x00,0x00,  // C  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x60,0x30,0xC0,0x1F,0x80,0x0F,0x00,0x00,  // D  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x22,0x20,0x22,0x20,0x27,0x60,0x30,0x00,0x00,  // E  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x22,0x20,0x02,0x60,0x07,0xE0,0x00,0x00,0x00,  // F  8x12
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x20,0x24,0xE0,0x3C,0xC0,0x3C,0x00,0x00,  // G  8x12
    0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x00,0x02,0xE0,0x3F,0xE0,0x3F,0x00,0x00,0x00,0x00,  // H  8x12
    0x00,0x00,0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,  // I  8x12
    0x00,0x1C,0x00,0x3C,0x00,0x20,0x20,0x20,0xE0,0x3F,0xE0,0x1F,0x20,0x00,0x00,0x00,  // J  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x80,0x0F,0xE0,0x3D,0x60,0x30,0x00,0x00,  // K  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x00,0x20,0x00,0x38,0x00,0x3C,0x00,0x00,  // L  8x12
    0xE0,0x3F,0xE0,0x3F,0xC0,0x01,0x80,0x03,0xC0,0x01,0xE0,0x3F,0xE0,0x3F,0x00,0x00,  // M  8x12
    0xE0,0x3F,0xE0,0x3F,0x80,0x03,0x00,0x07,0x00,0x0E,0xE0,0x3F,0xE0,0x3F,0x00,0x00,  // N  8x12
    0x80,0x0F,0xC0,0x1F,0x60,0x30,0x20,0x20,0x60,0x30,0xC0,0x1F,0x80,0x0F,0x00,0x00,  // O  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x22,0x20,0x02,0xE0,0x03,0xC0,0x01,0x00,0x00,  // P  8x12
    0x80,0x0F,0xC0,0x1F,0x60,0x10,0x20,0x58,0x60,0x7C,0xC0,0x7F,0x80,0x4F,0x00,0x00,  // Q  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x02,0x20,0x06,0xE0,0x3F,0xC0,0x39,0x00,0x00,  // R  8x12
    0xC0,0x19,0xE0,0x3B,0x20,0x22,0x20,0x26,0xE0,0x3C,0xC0,0x18,0x00,0x00,0x00,0x00,  // S  8x12
    0x60,0x00,0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x20,0x20,0x60,0x00,0x00,0x00,0x00,0x00,  // T  8x12
    0xE0,0x1F,0xE0,0x3F,0x00,0x20,0x00,0x20,0xE0,0x3F,0xE0,0x1F,0x00,0x00,0x00,0x00,  // U  8x12
    0xE0,0x0F,0xE0,0x1F,0x00,0x30,0x00,0x30,0xE0,0x1F,0xE0,0x0F,0x00,0x00,0x00,0x00,  // V  8x12
    0xE0,0x07,0xE0,0x3F,0x00,0x38,0x00,0x06,0x00,0x38,0xE0,0x3F,0xE0,0x07,0x00,0x00,  // W  8x12
    0xE0,0x38,0xE0,0x3D,0x00,0x07,0x00,0x07,0xE0,0x3D,0xE0,0x38,0x00,0x00,0x00,0x00,  // X  8x12
    0xE0,0x01,0xE0,0x23,0x00,0x3E,0x00,0x3E,0xE0,0x23,0xE0,0x01,0x00,0x00,0x00,0x00,  // Y  8x12
    0xE0,0x30,0x60,0x3C,0x20,0x2E,0xA0,0x23,0xE0,0x21,0x60,0x30,0x60,0x38,0x00,0x00,  // Z  8x12
    0x00,0x18,0x00,0x3D,0x00,0x25,0x00,0x25,0x00,0x1F,0x00,0x3E,0x00,0x20,0x00,0x00,  // a  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x1F,0x00,0x21,0x00,0x21,0x00,0x3F,0x00,0x1E,0x00,0x00,  // b  8x12
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x00,0x21,0x00,0x33,0x00,0x12,0x00,0x00,0x00,0x00,  // c  8x12
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x20,0x21,0xE0,0x1F,0xE0,0x3F,0x00,0x20,0x00,0x00,  // d  8x12
    0x00,0x1E,0x00,0x3F,0x00,0x25,0x00,0x25,0x00,0x37,0x00,0x16,0x00,0x00,0x00,0x00,  // e  8x12
    0x00,0x22,0xC0,0x3F,0xE0,0x3F,0x20,0x22,0x60,0x02,0x40,0x00,0x00,0x00,0x00,0x00,  // f  8x12
    0x00,0x4E,0x00,0xDF,0x00,0x91,0x00,0x91,0x00,0xFE,0x00,0x7F,0x00,0x01,0x00,0x00,  // g  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x00,0x02,0x00,0x01,0x00,0x3F,0x00,0x3E,0x00,0x00,  // h  8x12
    0x00,0x00,0x00,0x21,0x00,0x21,0x60,0x3F,0x60,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,  // i  8x12
    0x00,0x60,0x00,0xE0,0x00,0x81,0x00,0x81,0x60,0xFF,0x60,0x7F,0x00,0x00,0x00,0x00,  // j  8x12
    0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x00,0x04,0x00,0x0E,0x00,0x3B,0x00,0x31,0x00,0x00,  // k  8x12
    0x00,0x00,0x20,0x20,0x20,0x20,0xE0,0x3F,0xE0,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,  // l  8x12
    0x00,0x3F,0x00,0x3F,0x00,0x01,0x00,0x1F,0x00,0x01,0x00,0x3F,0x00,0x3E,0x00,0x00,  // m  8x12
    0x00,0x3F,0x00,0x3F,0x00,0x01,0x00,0x01,0x00,0x3F,0x00,0x3E,0x00,0x00,0x00,0x00,  // n  8x12
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x00,0x21,0x00,0x3F,0x00,0x1E,0x00,0x00,0x00,0x00,  // o  8x12
    0x00,0x81,0x00,0xFF,0x00,0xFE,0x00,0xA1,0x00,0x21,0x00,0x3F,0x00,0x1E,0x00,0x00,  // p  8x12
    0x00,0x1E,0x00,0x3F,0x00,0x21,0x00,0xA1,0x00,0xFE,0x00,0xFF,0x00,0x81,0x00,0x00,  // q  8x12
    0x00,0x21,0x00,0x3F,0x00,0x3F,0x00,0x24,0x00,0x03,0x00,0x07,0x00,0x06,0x00,0x00,  // r  8x12
    0x00,0x12,0x00,0x37,0x00,0x25,0x00,0x29,0x00,0x3B,0x00,0x12,0x00,0x00,0x00,0x00,  // s  8x12
    0x00,0x01,0x80,0x1F,0xC0,0x3F,0x00,0x21,0x00,0x31,0x00,0x11,0x00,0x00,0x00,0x00,  // t  8x12
    0x00,0x1F,0x00,0x3F,0x00,0x20,0x00,0x20,0x00,0x1F,0x00,0x3F,0x00,0x20,0x00,0x00,  // u  8x12
    0x00,0x0F,0x00,0x1F,0x00,0x30,0x00,0x30,0x00,0x1F,0x00,0x0F,0x00,0x00,0x00,0x00,  // v  8x12
    0x00,0x0F,0x00,0x3F,0x00,0x30,0x00,0x0C,0x00,0x30,0x00,0x3F,0x00,0x0F,0x00,0x00,  // w  8x12
    0x00,0x21,0x00,0x33,0x00,0x1E,0x00,0x0C,0x00,0x1E,0x00,0x33,0x00,0x21,0x00,0x00,  // x  8x12
    0x00,0x80,0x00,0x8F,0x00,0x9F,0x00,0xD0,0x00,0x70,0x00,0x3F,0x00,0x0F,0x00,0x00,  // y  8x12
    0x00,0x33,0x00,0x39,0x00,0x29,0x00,0x25,0x00,0x27,0x00,0x33,0x00,0x00,0x00,0x00,  // z  8x12
*/

  uint8_t largeText [] =                                                                                // large letters we actually use
  {
    0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00,0x00,0x00,      // B  12x16
    0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0x03,0x30,0x03,0x00,0x00,      // E  12x16
    0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00,      // I  12x16
    0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0xC7,0x00,0xFE,0x00,0x7C,0x00,0x00,      // P  12x16
    0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00,      // T  12x16
    0x00,0x00,0x1C,0x00,0x3E,0x40,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0xE0,0x3F,0xC0,0x00,0x00,      // a  12x16
    0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0xE0,0x30,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // d  12x16
    0x00,0x00,0x0F,0x80,0x1F,0xC0,0x3B,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x13,0xC0,0x01,0x80,0x00,0x00,      // e  12x16
    0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,      // n  12x16
    0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00,      // o  12x16
    0x00,0x00,0x11,0xC0,0x33,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0x60,0x1E,0x40,0x00,0x00,0x00,0x00,0x00,0x00,      // s  12x16
    0x00,0x00,0x0F,0xE0,0x1F,0xE0,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x18,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x00,      // u  12x16
    0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x03,0x00,0x00,      // F  12x16
    0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0x00,      // r  12x16
    0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x60,0x3F,0xEC,0x3F,0xEC,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,      // i  12x16
    0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,      // l  12x16
    0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x3F,0xC7,0x3F,0xC6,0x00,0x00       // G  12x16
    
  };
/*       Full alpha table for future use
0x00,0x00,0x38,0x00,0x3F,0x00,0x07,0xE0,0x06,0xFC,0x06,0x1F,0x06,0x1F,0x06,0xFC,0x07,0xE0,0x3F,0x00,0x38,0x00,0x00,0x00,      // A  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00,0x00,0x00,      // B  12x16
0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0C,0x0C,0x00,0x00,      // C  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00,      // D  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0x03,0x30,0x03,0x00,0x00,      // E  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x03,0x00,0x00,      // F  12x16
0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x3F,0xC7,0x3F,0xC6,0x00,0x00,      // G  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // H  12x16
0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00,      // I  12x16
0x00,0x00,0x0E,0x00,0x1E,0x00,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00,      // J  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x01,0xE0,0x03,0xF0,0x07,0x38,0x0E,0x1C,0x1C,0x0E,0x38,0x07,0x30,0x03,0x00,0x00,      // K  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00,      // L  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x1E,0x00,0x78,0x01,0xE0,0x01,0xE0,0x00,0x78,0x00,0x1E,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // M  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x0E,0x00,0x38,0x00,0xF0,0x03,0xC0,0x07,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // N  12x16
0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00,      // O  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0xC7,0x00,0xFE,0x00,0x7C,0x00,0x00,      // P  12x16
0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x36,0x03,0x3E,0x07,0x1C,0x0E,0x3F,0xFC,0x33,0xF0,0x00,0x00,      // Q  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x03,0x83,0x07,0x83,0x0F,0x83,0x1D,0xC7,0x38,0xFE,0x30,0x7C,0x00,0x00,      // R  12x16
0x00,0x00,0x0C,0x3C,0x1C,0x7E,0x38,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xC7,0x1F,0x8E,0x0F,0x0C,0x00,0x00,      // S  12x16
0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00,      // T  12x16
0x00,0x00,0x07,0xFF,0x1F,0xFF,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00,      // U  12x16
0x00,0x00,0x00,0x07,0x00,0x3F,0x01,0xF8,0x0F,0xC0,0x3E,0x00,0x3E,0x00,0x0F,0xC0,0x01,0xF8,0x00,0x3F,0x00,0x07,0x00,0x00,      // V  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x1C,0x00,0x06,0x00,0x03,0x80,0x03,0x80,0x06,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // W  12x16
0x00,0x00,0x30,0x03,0x3C,0x0F,0x0E,0x1C,0x03,0x30,0x01,0xE0,0x01,0xE0,0x03,0x30,0x0E,0x1C,0x3C,0x0F,0x30,0x03,0x00,0x00,      // X  12x16
0x00,0x00,0x00,0x03,0x00,0x0F,0x00,0x3C,0x00,0xF0,0x3F,0xC0,0x3F,0xC0,0x00,0xF0,0x00,0x3C,0x00,0x0F,0x00,0x03,0x00,0x00,      // Y  12x16
0x00,0x00,0x30,0x03,0x3C,0x03,0x3E,0x03,0x33,0x03,0x31,0xC3,0x30,0xE3,0x30,0x33,0x30,0x1F,0x30,0x0F,0x30,0x03,0x00,0x00,      // Z  12x16
0x00,0x00,0x1C,0x00,0x3E,0x40,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0xE0,0x3F,0xC0,0x00,0x00,      // a  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00,      // b  12x16
0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x18,0xC0,0x08,0x80,0x00,0x00,      // c  12x16
0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0xE0,0x30,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00,      // d  12x16
0x00,0x00,0x0F,0x80,0x1F,0xC0,0x3B,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x13,0xC0,0x01,0x80,0x00,0x00,      // e  12x16
0x00,0x00,0x00,0xC0,0x00,0xC0,0x3F,0xFC,0x3F,0xFE,0x00,0xC7,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,      // f  12x16
0x00,0x00,0x03,0x80,0xC7,0xC0,0xCE,0xE0,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xE6,0x60,0x7F,0xE0,0x3F,0xE0,0x00,0x00,      // g  12x16
0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,0x00,0x00,      // h  12x16
0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x60,0x3F,0xEC,0x3F,0xEC,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,      // i  12x16
0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0xC0,0x60,0xFF,0xEC,0x7F,0xEC,0x00,0x00,0x00,0x00,0x00,0x00,      // j  12x16
0x00,0x00,0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x38,0x60,0x30,0x00,0x00,0x00,0x00,0x00,      // k  12x16
0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,      // l  12x16
0x00,0x00,0x3F,0xE0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,      // m  12x16
0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,      // n  12x16
0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00,      // o  12x16
0x00,0x00,0xFF,0xE0,0xFF,0xE0,0x0C,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x1C,0xE0,0x0F,0xC0,0x07,0x80,0x00,0x00,      // p  12x16
0x00,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x0C,0x60,0xFF,0xE0,0xFF,0xE0,0x00,0x00,      // q  12x16
0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0x00,      // r  12x16
0x00,0x00,0x11,0xC0,0x33,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0x60,0x1E,0x40,0x00,0x00,0x00,0x00,0x00,0x00,      // s  12x16
0x00,0x00,0x00,0x60,0x00,0x60,0x1F,0xFE,0x3F,0xFE,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,      // t  12x16
0x00,0x00,0x0F,0xE0,0x1F,0xE0,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x18,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x00,      // u  12x16
0x00,0x00,0x00,0x60,0x01,0xE0,0x07,0x80,0x1E,0x00,0x38,0x00,0x38,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00,      // v  12x16
0x00,0x00,0x07,0xE0,0x1F,0xE0,0x38,0x00,0x1C,0x00,0x0F,0xE0,0x0F,0xE0,0x1C,0x00,0x38,0x00,0x1F,0xE0,0x07,0xE0,0x00,0x00,      // w  12x16
0x00,0x00,0x30,0x60,0x38,0xE0,0x1D,0xC0,0x0F,0x80,0x07,0x00,0x0F,0x80,0x1D,0xC0,0x38,0xE0,0x30,0x60,0x00,0x00,0x00,0x00,      // x  12x16
0x00,0x00,0x00,0x00,0x00,0x60,0x81,0xE0,0xE7,0x80,0x7E,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00,0x00,0x00,      // y  12x16
0x00,0x00,0x30,0x60,0x38,0x60,0x3C,0x60,0x36,0x60,0x33,0x60,0x31,0xE0,0x30,0xE0,0x30,0x60,0x30,0x20,0x00,0x00,0x00,0x00       // z  12x16
*/
    
//****************************************************************************************************
// Initialization code
void setup() {

  // Set up pins for interface to LCD display
  DDRD  = 0x00;                                     // Set 8 bit bus as inputs for now
  pinMode(a0pin, OUTPUT);                           // a0pin    A0 to LCD
  pinMode(rwPin, OUTPUT);                           // rwPin    R/W control to LCD
  pinMode(ePin, OUTPUT);                            // ePin     Enable to LCD
  pinMode(clPin, OUTPUT);                           // clPin    Clock output to LCD
  pinMode(cs1pin, OUTPUT);                          // cs1pin   Chip select 1 to LCD
  pinMode(cs2pin, OUTPUT);                          // cs2pin   Chip select 2 to LCD
  pinMode(A2, INPUT);
  digitalWrite(A2, HIGH);                           // Set analog pin 2 as input and enable internal pullup.  Will be used for keyline sensing.

  // Set up timer to create clock for LCD display
  TCCR2B = TCCR2B & 0b11111000 | 0x02;              // set timer 2 for divide ratio of 16.  This gives 3.9 kHz instead of 2, but the LCD seems fine.
  analogWrite( clPin, 127);                         // set to 50% duty cycle
 
  // Set up wire.h as I2C slave receiver
  Wire.begin(ADDRESS);                              // Set up I2C as slave at ADDRESS
  Wire.onReceive(receiveEvent);                     // register I2C receive event

  // Initialize LCD, enable and clear
  selectChip(WGLCD_CHIPBOTH);                       // select both LCD chips on
  displayOn();                                      // Switch LCD displays on
  setSDL(0);                                        // Set start display line
  setMapping(WGLCD_SEGMAP_NORMAL);                  // select normal segment mapping
  fillScreen(0x00);                                 // Clear screen
}
//**************************************************************************************
// Main processing loop
// Processes and decodes data from the receiveEvent
// Continuously loops asynchronously from the receiveEvent(s)
void loop() {
//***********************************************************************************************      
// Decode retrieved data into segment information
//
// The following was determined by close examination of the LCD itself and then tracing pins back to
// the two PCF8576 driver chips.  This uses the standard 7 segment nomenclature:
//      A
//    -----
//   |     |
//  F|     |B
//   |  G  |
//    -----
//   |     |
//  E|     |C
//   |     |
//    -----
//      D
//
// First PCF8576     Byte               Second PCF8576     Byte
// S0  10 MHz A        0                S0  100 Hz A         5
// S1  10 MHz B        0                S1  100 Hz B         5
// S2  10 MHz C        0                S2  100 Hz C         5
// S3  10 MHz D        0                S3  100 Hz D         5
// S4  10 MHz E        0                S4  100 Hz E         5
// S5  10 MHz F        0                S5  100 Hz F         5
// S6  10 MHz G        0                S6  100 Hz G         5
// S7  Decimal point   0                S7  MHz              5
// S8  1 MHz A         1                S8  10 Channel A     6
// S9  1 MHz B         1                S9  10 Channel B     6
// S10 1 MHz C         1                S10 10 Channel C     6
// S11 1 MHz D         1                S11 10 Channel D     6
// S12 1 MHz E         1                S12 10 Channel E     6
// S13 1 MHz F         1                S13 10 Channel F     6
// S14 1 MHz G         1                S14 10 Channel G     6
// S15 PROG            1                S15 Right arrow      6
// S16 100 kHz A       2                S16 1 Channel A      7
// S17 100 kHz B       2                S17 1 Channel B      7
// S18 100 kHz C       2                S18 1 Channel C      7
// S19 100 kHz D       2                S19 1 Channel D      7
// S20 100 kHz E       2                S20 1 Channel E      7
// S21 100 kHz F       2                S21 1 Channel F      7
// S22 100 kHz G       2                S22 1 Channel G      7
// S23 RX              2                S23 Left arrow       7
// S24 10 kHz A        3                S24 HI               8
// S25 10 kHz B        3                S25 M                8
// S26 10 kHz C        3                S26 LO               8
// S27 10 kHz D        3                S27 F                8
// S28 10 kHz E        3                S28 K                8
// S29 10 kHz F        3                S29                  8
// S30 10 kHz G        3                S30                  8
// S31 EXTERNAL        3                S31                  8
// S32 1 kHz A         4                S32 Bar graph L      9
// S33 1 kHz B         4                S33 Bar graph M      9
// S34 1 kHz C         4                S34 Bar graph H      9
// S35 1 kHz D         4                S35                  9
// S36 1 kHz E         4                S36                  9
// S37 1 kHz F         4                S37                  9
// S38 1 kHz G         4                S38                  9
// S39 CH              4                S39                  9

//****************************************************************************
// Decode LCD digits and symbols
//
    MHz10 = segment2number(LCDdata[0]);                     // Frequency data
    MHz1 = segment2number(LCDdata[1]);
    kHz100 = segment2number(LCDdata[2]);
    kHz10 = segment2number(LCDdata[3]);
    kHz1 = segment2number(LCDdata[4]);
    Hz100 = segment2number(LCDdata[5]);
    channel10 = segment2number(LCDdata[6]);                 // Channel data (channels over 9 not used)
    channel1 = segment2number(LCDdata[7]);
                                
    LCDdecimalpoint = LCDdata[0] & bit0;                    // decimal point
    LCDprog = LCDdata[1] & bit0;                            // PROG, program mode
    LCDRX = LCDdata[2] & bit0;                              // RX, receive indication
    LCDexternal = LCDdata[3] & bit0;                        // External, not sure what this is for
    LCDch = LCDdata[4] & bit0;                              // CH, channel symbol
    LCDmhz = LCDdata[5] & bit0;                             // MHz symbol
    LCDleftarrow = LCDdata[6] & bit0;                       // Left tuning arrow
    LCDrightarrow = LCDdata[7] & bit0;                      // Right tuning arrow
    LCDhi = LCDdata[8] & bit7;                              // HI, high power indication
    LCDm = LCDdata[8] & bit6;                               // M, medium power indication
    LCDlo = LCDdata[8] & bit5;                              // LO, low power indication
    LCDf = LCDdata[8] & bit4;                               // what is this used for?
    LCDk = LCDdata[8] & bit3;                               // what is this used for?

    invMHz10 = 0;                                           // When in frequency programming mode the digit being changed blinks.
    invMHz1 = 0;                                            // This is done by setting the LCD chips to alternate between two segment bitmaps.
    invKHz100 = 0;                                          // This code finds the differences between these two maps and sets flag(s)
    invKHz10 = 0;                                           // to be used later on to invert (black/white) the digit being programmed.
    invKHz1 = 0;
    invHz100 = 0;
    if (LCDprog){
      if ((LCDdata[0] & B11111110) != (LCDdata[10] & B11111110)){invMHz10 = 1;}
      if ((LCDdata[1] & B11111110) != (LCDdata[11] & B11111110)){invMHz1 = 1;}
      if ((LCDdata[2] & B11111110) != (LCDdata[12] & B11111110)){invKHz100 = 1;}
      if ((LCDdata[3] & B11111110) != (LCDdata[13] & B11111110)){invKHz10 = 1;}
      if ((LCDdata[4] & B11111110) != (LCDdata[14] & B11111110)){invKHz1 = 1;}      
      if ((LCDdata[5] & B11111110) != (LCDdata[15] & B11111110)){invHz100 = 1;}      
    }
    clean = 0;
//******************************************************************************************
/* Write frequency data to display
 *  
 *  First, is frequency being displayed?  The MHz indicator is off when presenting text such as "PASS" 
 *  The 4 letter messages are in the places after the decimal point
 *  
*/ 
  if (LCDmhz){                                                          // display frequrncy if MHz symbol on
    if (LCDmhz != LCDmhzW){                                             // display MHz letters
      writeMHz(1);
      writeFdecimal(1);
      LCDmhzW = 1;
      }
    if (invMHz10){                                                      // When in blink mode the normal segment map is empty
      writeFrequency(0, segment2number(LCDdata[10]));                   // for the blinking digit so the actual number has
    }                                                                   // to be found in the alternate segment map
    else{
      writeFrequency(0, MHz10);      
    }
    if (invMHz1){
      writeFrequency(1, segment2number(LCDdata[11]));
    }
    else{
      writeFrequency(1, MHz1);      
    }
    if (invKHz100){
      writeFrequency(2, segment2number(LCDdata[12]));
    }
    else{
      writeFrequency(2, kHz100);      
    }
    if (invKHz10){
      writeFrequency(3, segment2number(LCDdata[13]));
    }
    else{
      writeFrequency(3, kHz10);      
    }
    if (invKHz1){
      writeFrequency(4, segment2number(LCDdata[14]));
    }
    else{
      writeFrequency(4, kHz1);      
    }
        if (invHz100){
      writeFrequency(5, segment2number(LCDdata[15]));
    }
    else{
      writeFrequency(5, Hz100);      
    }
    writeClean();                                                       // this clears display space which was between the last freq digit and the 'MHz'
    clean = 0;
    LCDmsgPass = 0;
    LCDmsgTune = 0;
    LCDmsgGood = 0;
    LCDmsgPoor = 0;
    LCDmsgFail = 0;
  }
  else {                                                                // display text message
    if (!(LCDmsgPass || LCDmsgTune || LCDmsgGood || LCDmsgPoor || LCDmsgFail)){
      LCDmhzW=0;
      writeFrequency(0, 0x0E);                                            // clear frequency display
      writeFrequency(1, 0x0E);
      writeFdecimal(0);
      writeFrequency(2, 0x0E);
      writeFrequency(3, 0x0E);
      writeFrequency(4, 0x0E);
      writeFrequency(5, 0x0E);
      writeClean();
      writeMHz(0);
    }
                                                                        // decode alpha displays PASS FAIL Error Tune; 4 letter start at 100 kHz digit
    if ((kHz100 == 10) & (kHz10 == 11))                 // PASS detected?
    {
      if (!LCDmsgPass){
        writeMain(0,0);                                                   // This is BITE Pass, only thing on display
        writeMain(12,0);
        writeMain(24,'P');
        writeMain(36,'a');
        writeMain(48,'s');
        writeMain(60,'s');
        writeMain(84,0);
        writeMain(96,0);
        clean = 1;                                                        // inhibit other display info
        LCDmsgPass = 1;
      }
    }
    else if ((kHz100 == 21) & (kHz10 == 22))            // tune detected?
    {
      if (!LCDmsgTune){
        writeMain(24,'T');
        writeMain(36,'u');
        writeMain(48,'n');
        writeMain(60,'e');
        writeClean();
        clean = 0;                                                        // allow other display info
        LCDmsgTune = 1;
      }
    }
    else if ((kHz100 == 6) & (kHz10 == 20))             // Good detected?
    {
      if (!LCDmsgGood){
        writeMain(24,'G');
        writeMain(36,'o');
        writeMain(48,'o');
        writeMain(60,'d');
        writeClean();
        clean = 0;                                                        // allow other display info
        LCDmsgGood = 1;
      }
    }
//    else if ((kHz100 == 10) & (kHz10 == 11))                            // Pass
//    {
//      writeMain(24,'P');
//      writeMain(36,'a');                            // this is tune pass, can't differentiate from BITE Pass, remove it?
//      writeMain(48,'s');                            // 
//      writeMain(60,'s');
//      writeClean();
//      clean = 0;                                                        // allow other display info
//    }
    else if ((kHz100 == 10) & (kHz10 == 20))              // Poor detected?
    {
      if (!LCDmsgPoor){
        writeMain(24,'P');
        writeMain(36,'o');
        writeMain(48,'o');
        writeMain(60,'r');
        writeClean();
        clean = 0;                                                        // allow other display info
        LCDmsgPoor = 1;
      }
    }
    else if ((kHz100 == 12) & (kHz10 == 11))            // Fail detected?
    {
      if (!LCDmsgFail){
        writeMain(24,'F');
        writeMain(36,'a');
        writeMain(48,'i');
        writeMain(60,'l');
        writeClean();
        clean = 0;                                                        // allow other display info
        LCDmsgFail = 1;
      }
    }
                                                                        // Still need Error indications
  }
//******************************************************************************************
/* Write meter bar and voltage to display
 *  
*/
    LCDbar = 0;
    if ((LCDdata[9] & bit7) && !(LCDdata[9] & bit6) && !(LCDdata[9] & bit5)) {LCDbar = 1;};
    if ((LCDdata[9] & bit7) && (LCDdata[9] & bit6) && !(LCDdata[9] & bit5)) {LCDbar = 2;};    
    if ((LCDdata[9] & bit7) && (LCDdata[9] & bit6) && (LCDdata[9] & bit5)) {LCDbar = 3;};  
  if (LCDbar != LCDbarW){
     writeBars(LCDbar);                                                           // write bars to display
     LCDbarW = LCDbar;                                                            // store to see if we can skip write next time around
  }

//  write voltage to display
    volts = (analogRead(1) * (float) voltsFS / 1024);                  //  read and scale analog input A1

    invertV = 0;
    if (volts > 14.5 | volts < 10.5){
      invertV = 1;
    }
    
    volt10 = (int) (volts / 10);
    volts = volts - (float)(10 * volt10);
    volt1 = (int) volts;
    volts = 10 * (volts - (float)volt1);
    voltV1 = (int) volts;

    writeStatusChar(97, 0x30 + voltV1, invertV);
    writeStatusChar(85, 0x30 + volt1, invertV);    
    writeStatusChar(77, 0x30 + volt10, invertV);

    for(uint8_t index = 93; index < 95; index++){                         // place for decimal point
      selectChip(chipFromX(index));
      selectColumn(columnFromX(index));
      selectPage(2);                                                      // page 2
      dummyRead();
      writeChar(0x00);
      selectPage(3);                                                      // page 3
      selectColumn(columnFromX(index));                                   // reset autoincremented column
      dummyRead();
      writeChar(0xC0);
    }
    writeStatusChar(107, 'V', 0);
 
//******************************************************************************************
/* Write ham band info to display
 *  
*/
  char hamBand = 0;
  uint8_t MHz10Z = MHz10;
  if (MHz10 == 0x0E){
    MHz10Z = 0;                                                                                                 // blanked leading zero corrected for
  }
  long frequency = (10000000L * MHz10Z) + (1000000L * MHz1) + (100000L * kHz100) + (10000L * kHz10) + (1000L * kHz1) + (100L * Hz100);

  if ((frequency >= 1800000) && (frequency <= 2000000)) {                                                       // 160 meters
    hamBand = 'g';
    }
  else if ((frequency >= 3600000) && (frequency <= 3800000)) {                                                  // 80 meters extra
    hamBand = 'x';
    }
  else if ((frequency > 3800000) && (frequency <= 4000000)) {                                                   // 80 meters general
    hamBand = 'g';
    }
  else if (frequency == 5330500) {                                                                              // 60 meters chan 1
    hamBand = 'g';
    }
  else if (frequency == 5346500) {                                                                              // 60 meters chan 2
    hamBand = 'g';
    }
  else if (frequency == 5357000) {                                                                              // 60 meters chan 3
    hamBand = 'g';
    }
  else if (frequency == 5371500) {                                                                              // 60 meters chan 4
    hamBand = 'g';
    }
  else if (frequency == 5403500) {                                                                              // 60 meters chan 5
    hamBand = 'g';
    }
  else if ((frequency >= 7125000) && (frequency <= 7175000)) {                                                  // 40 meters extra
    hamBand = 'x';
    }
  else if ((frequency > 7175000) && (frequency <= 7300000)) {                                                   // 40 meters general
    hamBand = 'g';
    }
  else if ((frequency >= 14150000) && (frequency <= 14225000)) {                                                // 20 meters extra
    hamBand = 'x';
    }
  else if ((frequency > 14225000) && (frequency <= 14350000)) {                                                 // 20 meters general
    hamBand = 'g';
    }
  else if ((frequency >= 18110000) && (frequency <= 18168000)) {                                                // 17 meters
    hamBand = 'g';
    }
  else if ((frequency >= 21200000) && (frequency <= 21275000)) {                                                // 15 meters extra
    hamBand = 'x';
    }
  else if ((frequency > 21275000) && (frequency <= 21450000)) {                                                 // 15 meters general
    hamBand = 'g';
    }
  else if ((frequency >= 24930000) && (frequency <= 24990000)) {                                                // 12 meters
    hamBand = 'g';
    }
  else if ((frequency >= 28330000) && (frequency <= 29700000)) {                                                // 10 meters
    hamBand = 'g';
    }
  else {
    hamBand = 0;
  }

//******************************************************************************************     
// Program mode   (disabled, I don't like the look)      

//  if (LCDprog){
//    writeStatusChar(??,'R', 0);
//  }
//  else{
//    writeStatusChar(??,0, 0);
//  }

//******************************************************************************************
/* Write channel number to display
 *  
 *  If channel number is zero then display " VFO "
*/ 
  if (clean == 0 && LCDch){                                            // only display if the CH would normally be on, this keeps it clear initially
    if (channel1 != channel1W){                                        // only do the following if the channel number has changed
      if (channel1 == 0){                                              // display "VFO"
        writeStatusChar(0, 'V', 0);                                       // 
        writeStatusChar(8, 'F', 0);                                       // 
        writeStatusChar(16, 'O', 0);                                      // 
    if (hamBand != hamBandW){
     writeStatusChar(27, hamBand, 0);                                     // write ham band to display (was 97).  Do it here, only in VFO mode
     hamBandW = hamBand;                                               // store to see if we can skip write next time around
     }

        channel1W = channel1;
      }
      else {                                                            // display channel number
        writeStatusChar(0, 'C', 0);
        writeStatusChar(8, 'h', 0);
        writeStatusChar(16, 0, 0);                                         // clear out two cols between chars
        writeStatusChar(18, (0x30 + channel1), 0);                         // 
        channel1W = channel1;    
      }
    }   
  }
  else {
    writeStatusChar(0, 0, 0);                                              // clear display
    writeStatusChar(8, 0, 0);
    writeStatusChar(16, 0, 0);
    writeStatusChar(18, 0, 0);                                             // overlap to take care of space between Ch and number
  }

//******************************************************************************************
/* Write RX to display if squelch indicates open
 *  
*/
  TXlow = digitalRead(A2);
  if (!TXlow){
    writeStatusChar(57, 'T', 1);                                        // display TX 
    writeStatusChar(65, 'x', 1);                                        // 
  }
  else{
    if (LCDRX)
    {
      writeStatusChar(57, 'R', 0);                                        // display RX 
      writeStatusChar(65, 'x', 0);                                        // 
    }
    else
    {
      writeStatusChar(57, 0, 0);                                          // clear display if neither Tx or Rx
      writeStatusChar(65, 0, 0);
    }
  }
  
//******************************************************************************************
/* Write power level to display
 *  
*/
  if (LCDlo & (LCDpowerW != 1))                                      // if radio power is low and display isn't showing low, display "Lo"
  {
    writeStatusChar(38, 'L', 0);                                        // was 50
    writeStatusChar(46, 'o', 0);                                        // was 58
    LCDpowerW = 1;
  }
  else if (LCDm & (LCDpowerW != 2))                                  // if radio power is medium and display isn't showing medium, display "M"
  {
    writeStatusChar(38, 'M', 0);
    writeStatusChar(46, 'd', 0);
    LCDpowerW = 2;
  }
  else if (LCDhi & (LCDpowerW != 3))                                 // if radio power is high and display isn't showing high, display "Hi"
  {
    writeStatusChar(38, 'H', 0);
    writeStatusChar(46, 'i', 0);
    LCDpowerW = 3;
  }
//******************************************************************************************
/* Write left or right arrow to display
 *  
*/
  if (LCDleftarrow){
    writeArrow(2);                                                   // display left arrow
  }
  else if (LCDrightarrow){
    writeArrow(1);                                                   // display right arrow    
  }
  else {
    writeArrow(0);                                                   // no arrow
  }

}  // end of main loop

//******************************************************************************************
// Function that executes whenever data is received from I2C master addressed to us.
// Time servicing this even is kept to a minimum to allow for quick recovery and processing of another
// I2C receive such as when blink registers are loaded immediately following regular registers
//
void receiveEvent(int howMany) {
    cmdbyte = true;                                                 // initial byte received is always a command byte
    LCDpointer = 0;                                                 // initialize local LCD data array pointer
    bool LCDinBank = 0;                                             // PCF8576 RAM load bank select
    bool LCDoutBank = 0;                                            // PCF8576 RAM display bank select
//*******************************************************************************************
// PCF8576 command decoder and data collector
// Receive bytes intended for PCF8576 chips and if commands, decode them to local variables and flags.  
// This data is used by the main program loop to create the display
// If data sent, store to local array.
//
    while (0 < Wire.available()){                                   // loop through all received bytes
      byte indata = Wire.read();                                    // get byte of data from receive buffer
        if (cmdbyte){                                               // if command byte, decode command
          if (indata & maskC){                                      // if the C bit is set, the next byte received after this is also a command byte
            cmdbyte = true;
          }
          else{
            cmdbyte = false;                                        // next and remaining bytes will be data
          }
                                                                    // now we will decode the command byte
          if (indata & bit6 == false){                              // if bit 6 is 0, this is a LOAD DATA POINTER COMMAND
            LCDpointer = indata & maskP; 
          }
          else{
            if (indata & bit5 == false){                            // if bit 6 is 1 and bit 5 is 0, this is a MODE SET COMMAND
              LCDpowerMode = indata & maskL;
              LCDenable = indata & maskE;
              LCDbias = indata & maskB;
              LCDmode = indata & maskM;
            }
            else{                                                   // bits 5 and 6 are 1
              if (indata & bit4 == false){                          // bits 5 and 6 are 1, bit 4 is 0, this is a DEVICE SELECT COMMAND
                LCDdevice = indata & maskAd;
              }
              else{
                if (indata & bit3 == false){                        // bits 6,5,4 are 1 and bit 3 is 0, this is a BLINK COMMAND
                  LCDblinkMode = indata & maskA;
                  LCDblinkFreq = indata & maskF;
                }
                else{                                               // last remaining command is BANK SELECT COMMAND
                  if (indata & B00000010){
                    LCDinBank = 1;                                  // which bank is this data for?   maskI  (latch on until end of receive event)
                  }
                  LCDoutBank = indata & maskO;
                }
              }
            }
          }
        }   
        else{                                                       // data byte
          // Pointer seems to always be zero in this app
          //
          // First let's just try to load second bank data (bank 1) to second half of our data buffer.  If this doesn't screw up the normal functionality
          // then perhaps the radio CPU is simply writing both full pages and we can do the later compare based on the blink bit to see what digit is to be blinking
          // (so far, so good)
          // Two bugs:  1) getting a 7 instead of 2 for 10's in MHz at times.  2) freq display blanks at times during programming, like alt digits?
          //
          if (LCDinBank){
            LCDdata[0x0A + (LCDpointer++)] = indata;                // put incoming data byte into second bank area
          }
          else{
            LCDdata[LCDpointer++] = indata;                         // get data byte, increment pointer for next data byte (20 bytes allocated, 2 pages)
          }
        }
      } 
      LCDinBank = 0;                                                // reset this after 2nd page data received
  }                                                                 //end of receive event
    
//********************************************************************************************
// Converts segment data to number or character by basic pattern match.
// Data comes in as segments ABCDEFGx where bottom bit is a don't care
//
int segment2number(byte input){
  segment = input & B11111110;                         // strip low bit, not segment data
  if (segment == B11111100){                           // pattern is "0"  
    return 0;
  }
  else if (segment == B01100000){                     // pattern is "1" or "I" 
    return 1;
  }
  else if (segment == B11011010){                     // pattern is "2" 
    return 2;
  }
  else if (segment == B11110010){                     // pattern is "3"  
    return 3;
  }
  else if (segment == B01100110){                     // pattern is "4"  
    return 4;
  }
  else if (segment == B10110110){                     // pattern is "5" or "S"
    return 5;
  }
  else if (segment == B10111110){                     // pattern is "6" or "G" 
    return 6;
  }
  else if (segment == B11100000){                     // pattern is "7"  
    return 7;
  }
  else if (segment == B11111110){                     // pattern is "8"  
    return 8;
  }
  else if (segment == B11110110){                     // pattern is "9"  
    return 9;
  }
  else if (segment == B11001110){                     // pattern is "P"  
    return 10;
  }
  else if (segment == B11101110){                     // pattern is "A" 
    return 11;
  }  
  else if (segment == B10001110){                     // pattern is "F"  
    return 12;
  }
  else if (segment == B00011100){                     // pattern is "L"  
    return 13;
  }
  else if (segment == B00000000){                     // pattern is blank   return code 0x0E
    return 14;
  }
  else if (segment == B10000000){                     // pattern is "G"    no longer used
    return 15;
  }
  else if (segment == B01111010){                     // pattern is 'd'
    return 16;
  }
  else if (segment == B00101010){                     // pattern is "n"
    return 17;
  }
  else if (segment == B10011110){                     // pattern is "E"
    return 18;
  }
  else if (segment == B00001010){                     // pattern is "r"
    return 19;
  }
  else if (segment == B00111010){                     // pattern is "o"
    return 20;
  }
  else if (segment == B00011110){                     // pattern is "t"
    return 21;
  }
  else if (segment == B00111000){                     // pattern is "u"
    return 22;
  }
  else{
    return 23;                                        // unrecognized pattern
  }
}

// LCD functions follow
// These functions adapted from code by Vanyamba
//*****************************************************************************
  /**
   * To select page. 32 lines are segmented for 4 pages 8 lines each.
   * \param line Page number (0..3).
   */
void selectPage(uint8_t pag) {   
    command(WGLCD_CMD_SEL_PAGE + pag); 
  }
//*****************************************************************************
  /**
   * To select column. 122 columns are controled by 2 chips 61 column each.
   * \param line Column number (0..60).
   */
void selectColumn(uint8_t col)  
  { 
    command(WGLCD_CMD_SEL_COL + col);  
  }
//*****************************************************************************
    /**
   * To set Start Display Line.
   * \param line Start Display Line number (0..31).
   */
void setSDL(uint8_t line) {   
    m_sdl = line;
    command(WGLCD_CMD_SET_SDL + line); 
  }

//*****************************************************************************
  /**
   * Dummy read function. See WG12232 manual.
   */
void dummyRead() { 
    readChar(); 
  }
//***************************************************************************** 
  /**
   * To start Read/Modify/Write sequence.
   */
void beginRMW() { 
    command(WGLCD_CMD_RMW);      
  }
//*****************************************************************************
  /**
   * To finish Read/Modify/Write sequence.
   */
void endRMW() { 
    command(WGLCD_CMD_END_RMW); 
  }
//*****************************************************************************
  /**
   * Software reset command. It's not equivalent of hardware reset.
   */
void softReset()  
  { 
  command(WGLCD_CMD_SOFT_RST); 
  }
//*****************************************************************************
  /**
   * To switch display on for currently selected chip.
   */
void displayOn()  
  { 
  command(WGLCD_CMD_DISP_ON);  
  }
//*****************************************************************************
  /**
   * To switch display off for currently selected chip.
   */
void displayOff() 
  { 
  command(WGLCD_CMD_DISP_OFF); 
  }
//*****************************************************************************
  /**
   * To set segment mapping for currently selected chip.
   * \param seg Segment mapping mode. WGLCD_SEGMAP_NORMAL or WGLCD_SEGMAP_INVERT.
   */
void setMapping(uint8_t seg) 
  { 
  command(WGLCD_CMD_MAP_SEG + seg);  
  }
//*****************************************************************************
  /**
   * Chip selection.
   * \param chip WGLCD_NOCHIP, WGLCD_CHIP1, WGLCD_CHIP2 or WGLCD_CHIPBOTH.
   */
void selectChip(uint8_t chip){
  digitalWrite(cs1pin, (chip & WGLCD_CHIP1) ? LOW : HIGH);
  digitalWrite(cs2pin, (chip & WGLCD_CHIP2) ? LOW : HIGH);
}
//*****************************************************************************
  /**
   * Data Transfer Mode selection.
   * \param dtm Data Transfer Mode value.
   */
void setDTM(uint8_t dtm){
  digitalWrite(rwPin, (dtm & WGLCD_DTM_RW_BIT) ? HIGH : LOW); 
  digitalWrite(a0pin, (dtm & WGLCD_DTM_CD_BIT) ? HIGH : LOW);
}
//*****************************************************************************
  /**
   * To send a command to WG12232.
   * \param cmd WG12232 command code. See datasheet.  
   * PAG: changed order of outputs to comply with datasheet
   * PAG: set CS pins before running this
   */
void command(uint8_t cmd){
  setDTM(WGLCD_DTM_WRITECMD);                     // Set up pins to write command data
  digitalWrite(ePin,  HIGH); 
  PORTD = cmd;                                    // Set command byte to bus
  DDRD  = 0xFF;                                   // Set data bus pins as outputs
  digitalWrite(ePin,  LOW);
  PORTD = 0;                                      // clear output byte
  DDRD  = 0x00;                                   // Set data bus pins as inputs
}
//*****************************************************************************
  /**
   * To write data byte to LCD memory.
   * \param ch Data byte value (0..255).
   * PAG setDTM changed to first
   */
void writeChar(uint8_t ch){
  setDTM(WGLCD_DTM_WRITEDISPDATA);                // set up pins to write display data
  digitalWrite(ePin,  HIGH); 
  PORTD = ch;                                     // place character on bus
  DDRD  = 0xFF;                                   // Set 8 bit bus as outputs
  digitalWrite(ePin,  LOW);
  PORTD = 0x00;                                   // clear any possible bus output data
  DDRD  = 0x00;                                   // Set 8 bit bus as inputs
}
//*****************************************************************************
  /**
   * To get WG12232 Status Register value. 
   */
uint8_t status(){
  digitalWrite(ePin,  LOW); 
  setDTM(WGLCD_DTM_READSTATUS);
  PORTD = 0x00;                                   // clear any possible bus output data
  DDRD  = 0x00;                                   // Set 8 bit bus as inputs
  digitalWrite(ePin,  HIGH);
  unsigned char result = PIND;                    // Read data bus
  return result;
}
//*****************************************************************************
  /**
   * To read data byte from LCD memory.
   */
uint8_t readChar(){
  digitalWrite(ePin,  LOW); 
  setDTM(WGLCD_DTM_READDISPDATA);
  PORTD = 0x00;                                   // clear any possible bus output data
  DDRD  = 0x00;                                   // Set 8 bit bus as inputs
  digitalWrite(ePin,  HIGH);
  unsigned char result = PIND;                    // Read data bus
  return result;
}
//*****************************************************************************
  /**
   * To fill whole column of 4 pages with data byte value.
   * \param x Column number (0..122).
   * \param ch Data byte value (0..255).
   */
void fillColumn(uint8_t x, uint8_t ch){
  selectChip(chipFromX(x));
  int cn = columnFromX(x);
  for(uint8_t pn = 0; pn < 4; ++pn) {
    selectPage(pn);
    selectColumn(cn);
    dummyRead();
    writeChar(ch);
  }
}
//*****************************************************************************
uint8_t pageFromY(uint8_t y){
  uint8_t line = m_sdl + y;
  if(line > 31)
    line -= 32;
  return line / 8;
}
//*****************************************************************************
uint8_t bitFromY(uint8_t y){
  uint8_t line = m_sdl + y;
  if(line > 31)
    line -= 32;
  return line % 8;
}
//*****************************************************************************
  /**
   * To fill whole row of 122 columns with 1-bit value.
   * \param y Row number (0..31).
   * \param ch 1-bit value (0..1).
   */
void fillRow(uint8_t y, uint8_t ch){
  uint8_t pixel = 1 << bitFromY(y);
  selectChip(WGLCD_CHIPBOTH);
  selectPage(pageFromY(y));
  selectColumn(0);
  beginRMW();
  for(uint8_t cn = 0; cn < 61; ++cn) {
    dummyRead();
    uint8_t mem   = readChar();
    if(ch)
      writeChar(mem | pixel);
    else
      writeChar(mem & ~pixel);
  }
  endRMW();
}
//*****************************************************************************
  /**
   * To fill whole screen with data byte value.
   * \param ch Data byte value (0..255).
   */
void fillScreen(uint8_t ch){
  selectChip(WGLCD_CHIPBOTH);                           // Select both LCD chips
  for(uint8_t pn = 0; pn < 4; ++pn) {
    selectPage(pn);                                     // send command WGLCD_CMD_SEL_PAGE + pn to select page
    selectColumn(0);                                    // send command WGLCD_CMD_SEL_COL to select column 0 
    beginRMW();                                         // send command WGLCD_CMD_RMW to begin read-modify-write operation
    for(uint8_t cn = 0; cn < 61; ++cn) {
      dummyRead();
      writeChar(ch);                                    // write all 61 columns with requested value
    }
    endRMW();                                           // send command WGLCD_CMD_END_RMW to end read-modify-write operation
  }
}
//*****************************************************************************

uint8_t chipFromX(uint8_t x){
    return (x > 60) ? WGLCD_CHIP2 : WGLCD_CHIP1; 
  }

//*****************************************************************************  
uint8_t columnFromX(uint8_t x){
    return (x > 60) ? x-61 : x;
  }
//*****************************************************************************
  /**
   * To set pixel with 1-bit value.
   * \param ch 1-bit value (0..1).
   */
void setPixel(uint8_t x, uint8_t y, uint8_t val){
  selectChip(chipFromX(x));
  selectPage(pageFromY(y)); 
  selectColumn(columnFromX(x));
  beginRMW();
  dummyRead();
  uint8_t mem   = readChar();
  uint8_t pixel = 1 << bitFromY(y);
  if(val)
    writeChar(mem | pixel);
  else
    writeChar(mem & ~pixel);
  endRMW();
}
//*****************************************************************************
/*  Write frequency to display
 * 
 * args:  digit, value
 * digit is 0 through 5 representing 10 MHz, 1 MHz, 100 kHz, 10 kHz, 1 kHz, 100 Hz
 * value is 0 through 9
 * If val = 0x0E, write a blank, this corresponds to a blank 7 segment
 * 
 * frequency digits are 12 pixels wide by 16 pixels high
 */
void writeFrequency(uint8_t digit, uint8_t val)  {
    uint8_t coordX;
    bool invert;
    if (!(((val >= 0) & (val <= 9)) | val == 0x0E)){                     // bounds checking
      return;
    }
    int offset = ((val << 4) + (val << 3));                              // multiply digit value by 24 to get table location
    // determine X coordinate of display

    switch (digit) {
      case 0:
        coordX = 0;
        invert = LCDprog && invMHz10;
        break;
      case 1:
        coordX = 12;                                                        // digit X positions on display
        invert = LCDprog && invMHz1;
        break;
      case 2:
        coordX = 30;
        invert = LCDprog && invKHz100;
        break;
      case 3:
        coordX = 42;
        invert = LCDprog && invKHz10;
        break;
      case 4:
        coordX = 54;
        invert = LCDprog && invKHz1;
        break;
      case 5:
        coordX = 66;
        invert = LCDprog && invHz100;
        break;      
    }
    // write pixel data to display
    uint8_t tempA;
    for(uint8_t index = 0; index < 12; index++) {
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(0);                                                      // page 0, top byte
      dummyRead();
      if (val == 0x0E) {
        writeChar(0);
      }
      else {
       tempA = freqDigits[(offset + 1) + (index << 1)];
       if (invert){
         writeChar(~tempA);
         }
       else{
         writeChar(tempA);   
         }
      }
      selectPage(1);                                                      // page 1, second from top byte
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      if (val == 0x0E){
        writeChar(0);                                                     // 0x0E means write zero to blank char
      }
      else {
        tempA = freqDigits[offset + (index << 1)];
        if (invert){
          writeChar(~tempA);
        }
        else {
          writeChar(tempA);        
        }
      }
  }
}
//*****************************************************************************
/*  Write frequency decimal point to display
 * 
 * frequency decimal point is 6 pixels wide by 16 pixels high
 * 
 */
void writeFdecimal(bool state){
    uint8_t coordX = 24;                                                // start at position 24
    uint8_t offset = 0;
    // write pixel data to display
    for(uint8_t index = 0; index < 6; index++){
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(0);                                                      // page 0, top byte
      dummyRead();
      if (state){
        writeChar(fDecimal[(offset + 1) + (index << 1)]);
      }
      else {
        writeChar(0);
      }
      selectPage(1);                                                      // page 1, second from top byte
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      if (state){
        writeChar(fDecimal[offset + (index << 1)]);
      }
      else {
        writeChar(0);
      }
    }
  }
//*****************************************************************************
/*  Clear display between frequency and MHz letters
 * 
 */
void writeClean(){
    uint8_t coordX = 78;                                                // start at position 24
    uint8_t offset = 0;
    // write pixel data to display
    for(uint8_t index = 0; index < 3; index++){
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(0);                                                      // page 0, top byte
      writeChar(0);
      selectPage(1);                                                      // page 1, second from top byte
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      writeChar(0);
    }
  }
//*****************************************************************************
/*  Write MHz to display
 * 
 * MHz font is 8 pixels wide by 12 pixels high with 2 pixels on top and bottom as padding
 *
 * input arg is boolean, to write or clear
 */
void writeMHz(bool state){
    uint8_t coordX = 81;                                                // start at position 91
    uint8_t offset = 0;
    // write pixel data to display
    for(uint8_t index = 0; index < 24; index++){
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(0);                                                      // page 0, top byte
      dummyRead();
      if (state){
        writeChar(fMHz[offset + (index << 1)]);
      }
      else {
        writeChar(0);
      }      
      selectPage(1);                                                      // page 1, second from top byte
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      if (state){
        writeChar(fMHz[(offset + 1) + (index << 1)]);
      }
      else {
        writeChar(0);
      }            
    }
  }
//*****************************************************************************
/* Write character to status line on display
 * 
 * Uses smallText table
 * inputs are X pixel position to write character and the ASCII character to display
 * If char=0 write zeroes
 */
void writeStatusChar(uint8_t coordX, char ascii, bool invert){
    int index = 0;                                                        // table index for font table
    uint8_t width = 8;                                                    // width of character to display (easy to make this an arg later to be more general)
    if ((ascii >= 0x31) & (ascii <= 0x39)){                               // test if character is number
       index = ((ascii - 0x31) *16);                                      // numbers are in positions 0x00 through 0x09 of table
      }
    else {
      switch (ascii) {
        case 0x43:                                                          // C
          index = 0x09 * 16;
          break;      
        case 0x46:                                                          // F
          index = 0x0A * 16;
          break;
        case 0x47:                                                          // G
          index = 0x0B * 16;
          break;  
        case 0x48:                                                          // H
          index = 0x0C * 16;
          break;  
        case 0x4C:                                                          // L
          index = 0x0D * 16;
          break;  
        case 0x4D:                                                          // M
          index = 0x0E * 16;
          break;  
        case 0x4F:                                                          // O
          index = 0x0F * 16;
          break;  
        case 0x52:                                                          // R
          index = 0x10 * 16;
          break;  
        case 0x56:                                                          // V
          index = 0x11 * 16;
          break;  
        case 0x58:                                                          // X
          index = 0x12 * 16;
          break;  
        case 0x61:                                                          // a
          index = 0x13 * 16;
          break;  
        case 0x68:                                                          // h
          index = 0x14 * 16;
          break;  
        case 0x69:                                                          // i
          index = 0x15 * 16;
          break;  
        case 0x6E:                                                          // n
          index = 0x16 * 16;
          break;  
        case 0x6F:                                                          // o
          index = 0x17 * 16;
          break;  
        case 0x78:                                                          // x
          index = 0x18 * 16;
          break;  
        case 0x7A:                                                          // z
          index = 0x19 * 16;
          break;
        case 0x00:                                                          // blank
          index = 0x00 * 16;
          break;
        case 0x64:                                                          // d
          index = 0x1A * 16;
          break;
        case 0x30:                                                          // 0
          index = 0x1B * 16;
          break;
        case 0x67:                                                          // g
          index = 0x1C * 16;
          break;
        case 0x54:                                                          // T
          index = 0x1D * 16;
          break;
        default:   
          return;                                                           // do nothing if ASCII code not something we have
        }
      } 
    for(uint8_t colm = 0; colm < width; colm++){                          // loop through all columns of width of character to display
        selectChip(chipFromX(coordX + colm));
        selectColumn(columnFromX(coordX + colm));
                                                                          // take two pixel bytes from character table and write to display
                                                                          // the following moves the characters down 2 pixels to the bottom of the display
        uint8_t inter = smallText[index + (colm << 1)];                   // multiply by 2 since two bytes per character in font table so as we step through cols 
                                                                          // we double step through pixel data bytes        
        uint8_t charA = inter << 2;
        uint8_t int02 = (inter >> 6) & 3;
        uint8_t charB = ((smallText[(index + 1) + (colm << 1)] << 2) | int02);
        if (ascii == 0x00){
          charA = 0;
          charB = 0;                                                      // if ascii character is 0x00 then write blank character
          }
                   
        selectPage(2);                                                    // page 2, third page from top
        dummyRead();
        if (invert){
          writeChar(~charA & B00000001);                                  // when inverting, linit to height of characters used in small font
          }
        else{
          writeChar(charA);   
          }
        selectPage(3);                                                    // page 3, bottom page of display
        selectColumn(columnFromX(coordX + colm));                         // reset autoincremented column
        dummyRead();
        if (invert){
         writeChar(~charB);
         }
       else{
         writeChar(charB);   
         }
       }
  }
//*****************************************************************************
/* Write character to main line on display
 * 
 * Uses largeText table, 12x16 font, for major status indications
 * inputs are X pixel position to write character and the ASCII character to display
 * If char=0 write zeroes
 */
void writeMain(uint8_t coordX, char ascii)
  {
    int index = 0;                                                        // table index for font table
    uint8_t width = 12;                                                   // width of character to display (easy to make this an arg later to be more general)
    switch (ascii) {
      case 0x42:                                                          // B
        index = 0x00 * 24;
        break;
      case 0x45:                                                          // E
        index = 0x01 * 24;
        break;
      case 0x49:                                                          // I
        index = 0x02 * 24;
        break;
      case 0x50:                                                          // P
        index = 0x03 * 24;
        break;      
      case 0x54:                                                          // T
        index = 0x04 * 24;
        break;
      case 0x61:                                                          // a
        index = 0x05 * 24;
        break;      
      case 0x64:                                                          // d
        index = 0x06 * 24;
        break;
      case 0x65:                                                          // e
        index = 0x07 * 24;
        break;      
      case 0x6E:                                                          // n
        index = 0x08 * 24;
        break;
      case 0x6F:                                                          // o
        index = 0x09 * 24;
        break;      
      case 0x73:                                                          // s
        index = 0x0A * 24;
        break;
      case 0x75:                                                          // u
        index = 0x0B * 24;
        break;   
      case 0x46:                                                          // F
        index = 0x0C * 24;
        break;   
      case 0x72:                                                          // r
        index = 0x0D * 24;
        break;   
      case 0x69:                                                          // i
        index = 0x0E * 24;
        break;   
      case 0x6C:                                                          // l
        index = 0x0F * 24;
        break;   
      case 0x47:                                                          // G
        index = 0x10 * 24;
        break;   
      default:   
        return;                                                            // do nothing if ASCII code not something we have
      }
    for(uint8_t colm = 0; colm < width; colm++)                           // loop through all columns of width of character to display
      {
        selectChip(chipFromX(coordX + colm));
        selectColumn(columnFromX(coordX + colm));
                                                                          // take two pixel bytes from character table and write to display
                                                                          // the following moves the characters down 2 pixels to the bottom of the display
        uint8_t charA = largeText[index + (colm << 1)];                   // multiply by 2 since two bytes per character in font table so as we step through cols 
                                                                          // we double step through pixel data bytes        
        uint8_t charB = (largeText[(index + 1) + (colm << 1)]);
        if (ascii == 0x00){
          charA = 0;
          charB = 0;                                                      // if ascii character is 0x00 then write blank character
        }
        selectPage(0);                                                    // page 0, top page
        dummyRead();
        writeChar(charB);
        selectPage(1);                                                    // page 1, second from top page
        selectColumn(columnFromX(coordX + colm));                         // reset autoincremented column
        dummyRead();
        writeChar(charA);
      }
  }
//*****************************************************************************
/*  Write meter bars to display
 * 
 * bars is input, 0,1,2 or 3
 * Changed 12/30/16 to 4 wide instead of 8 wide
 */
void writeBars(uint8_t bars){
    uint8_t coordX = 118;                                                // start at position 114 (last char position on status row)(now 118)
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    if (bars == 1){
        byte1 = 0x00;
        byte2 = 0xC0;
      }
    else if (bars == 2){
        byte1 = 0x00;
        byte2 = 0xC6;          
      }
    else if (bars == 3){
        byte1 = 0x30;
        byte2 = 0xC6;          
      }
    // write pixel data to display
    for(uint8_t index = 0; index < 4; index++){                           // was 8
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(2);                                                      // page 2
      dummyRead();
      writeChar(byte1);
      selectPage(3);                                                      // page 3
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      writeChar(byte2);
    }
  }
//*****************************************************************************
/*  Write arrow to display
 * 
 * type is:  0 for none, 1 for left, 2 for right
 * This routine may seem inefficient but I have tons of code space and limited global variable space
 */
  void writeArrow(uint8_t type){
    uint8_t coordX = 110;    
    uint8_t byte1;
    uint8_t byte2;
    for(uint8_t index = 0; index < 12; index++){
      if (type == 0){
        byte1 = 0x00;
        byte2 = 0x00;
      }
     else { 
      switch (index) {
        case 0x02:
          byte1 = 0x00;
          byte2 = 0x01;
          break;
        case 0x03:
          if (type == 1){
            byte1 = 0x80;
            byte2 = 0x03;    
          }
          else {
            byte1 = 0x00;
            byte2 = 0x01;
          }
          break;
        case 0x04:
          if (type == 1){
            byte1 = 0xC0;
            byte2 = 0x07;    
          }
          else {
            byte1 = 0x00;
            byte2 = 0x01;
          }
          break;
        case 0x05:
          if (type == 1){
            byte1 = 0xE0;
            byte2 = 0x0F;    
          }
          else {
            byte1 = 0x00;
            byte2 = 0x01;
          }
          break;
        case 0x06:
          if (type == 1){
            byte1 = 0x00;
            byte2 = 0x01;    
          }
          else {
            byte1 = 0x00;
            byte2 = 0x01;
          }
          break;
        case 0x07:
          if (type == 1){
            byte1 = 0x00;
            byte2 = 0x01;    
          }
          else {
            byte1 = 0xE0;
            byte2 = 0x0F;
          }
          break;
        case 0x08:
          if (type == 1){
            byte1 = 0x00;
            byte2 = 0x01;    
          }
          else {
            byte1 = 0xC0;
            byte2 = 0x07;
          }
          break;
        case 0x09:
          if (type == 1){
            byte1 = 0x00;
            byte2 = 0x01;    
          }
          else {
            byte1 = 0x80;
            byte2 = 0x03;
          }
          break;
        case 0x0A:
          byte1 = 0x00;
          byte2 = 0x01;
          break;
        default:
          byte1 = 0x00;
          byte2 = 0x00;
          break;
        }
      }
      selectChip(chipFromX(coordX + index));
      selectColumn(columnFromX(coordX + index));
      selectPage(1);                                                      // page 1
      dummyRead();
      writeChar(byte2);
      selectPage(0);                                                      // page 0
      selectColumn(columnFromX(coordX + index));                          // reset autoincremented column
      dummyRead();
      writeChar(byte1);
    }
/*
        0           1           2           3           4           5           6           7           8           9           A           B
    0x00,0x00,  0x00,0x00,  0x00,0x01,  0x80,0x03,  0xC0,0x07,  0xE0,0x0F,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0x00,0x00,      // left arrow  12x16
    0x00,0x00,  0x00,0x00,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0x00,0x01,  0xE0,0x0F,  0xC0,0x07,  0x80,0x03,  0x00,0x01,  0x00,0x00,      // right arrow  12x16
*/
  }

