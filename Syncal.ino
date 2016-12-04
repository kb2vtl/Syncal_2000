/*
  Syncal 
  Syncal 2000 display program

  Emulates original and now unobtainable Syncal 2000 LCD display allowing replacement 
  with new OLED display module which is available off the shelf.  

  This code uses liquidcrystal.h to communicate to the display and wire.h to emulate the 
  factory LCD driver chips in the radio.

  This code is in the public domain.

  Created 27 November 2016
  by Peter Gottlieb
*/

/**************************************************
 * Arduino Syncal 2000 display v0.0 - 2016/11/21
 * By Peter Gottlieb
 * This work is distributed under the GNU General 
 * Public License version 3 or later (GPL3+)
 * Please include this credit note if you want to 
 * re-use any part of this sketch. Respect my work 
 * as I'll do with yours.
 * Feel free to contact me: nerd@verizon.net
 * ************************************************/
#include <LiquidCrystal.h>
#include <Wire.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

bool LED = false;
byte c;
int x;
byte segment;
int count;

bool cmdbyte = false;                       // is the incoming byte a command?
int DataPointer = 0;                        // PCF8576 data pointer
bool LCDbias;                               // LCD bias level, no purpose here
bool LCDpowerMode;                          // LCD normal or power saving mode, no purpose here
bool LCDenable;                             // LCD display enable
byte LCDmode;                               // LCD drive mode, mode 01 (single backplane) wired in radio
byte LCDdevice;                             // LCD device select
bool LCDinBank;                             // PCF8576 RAM load bank select
bool LCDoutBank;                            // PCF8576 RAM display bank select
bool LCDblinkMode;                          // LCD blink, normal or alternation
byte LCDblinkFreq;                          // LCD blink frequency, off and 3 choices
byte LCDdata[16];                           // LCD display data storage array
int LCDpointer = 0;                         // pointer into LCDdata
byte MHz10;                                 // 10 MHz digit
byte MHz1;                                  // 1 MHz digit
byte kHz100;                                // 100 kHz digit
byte kHz10;                                 // 10 kHz digit
byte kHz1;                                  // 1 kHz digit
byte Hz100;                                 // 100 Hz digit
byte channel10;                             // 10's channel digit
byte channel1;                              // 1's channel digit
bool LCDdecimalpoint;                       // decimal point
bool LCDprog;                               // PROG
bool LCDRX;                                 // RX
bool LCDexternal;                           // EXTERNAL
bool LCDch;                                 // CH
bool LCDmhz;                                // MHz
bool LCDleftarrow;                          // left arrow
bool LCDrightarrow;                         // right arrow
bool LCDhi;                                 // HI
bool LCDm;                                  // M
bool LCDlo;                                 // LO
bool LCDf;                                  // F
bool LCDk;                                  // K
byte LCDbar;                                // 3 level bar chart for battery or transmit power


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
String disp = "0123456789PAFL *";              // characters for sending to Arduino LCD for test purposes

const int ADDRESS = 57;                      //I2C address of Syncal 2000 LCD driver


//****************************************************************************************************
// Initialization code
void setup() {
  lcd.begin(16, 2);                                 // 16 characters long, 2 rows
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);                   // turn the LED off
  LED = false;
  pinMode(7,OUTPUT);                                // trigger output pin for scope debug
  digitalWrite(7, LOW);
 
  //Set up wire.h as I2C slave receiver
  Wire.begin(ADDRESS);                              // Set up I2C as slave at ADDRESS
  Wire.onReceive(receiveEvent);                     // register I2C receive event

  //Initialize and clear display
  //lcd.clear();
  lcd.setCursor(0, 0);
 
  //Display startup message for 2 seconds
//  lcd.print("Syncal 2000     ");
//  lcd.setCursor(0, 1);                  //lcd.setCursor(col, row) 
//  lcd.print("Status:");
 // delay(2000);
  
}


//**************************************************************************************
// Main processing loop
// Not much happens here, this program is designed to update the OLED display only when
// new data comes in from the I2C bus

void loop() {
  //toggle LED
  if(LED == false)
  {
    digitalWrite(LED_BUILTIN, HIGH);                // turn the LED on
    LED = true;
  }
  else
  { 
    digitalWrite(LED_BUILTIN, LOW);                 // turn the LED off
    LED = false;
  }
  delay(250);    
}  

//******************************************************************************************
// Function that executes whenever data is received from I2C master addressed to us.
// Everything runs from this event; the main loop is the idle loop.
//
void receiveEvent(int howMany) {
    digitalWrite(7, HIGH);                            // turn on output 7
    cmdbyte = true;                                   // initial byte received is always a command byte
    LCDpointer = 0;                                   // initialize local LCD data array pointer
    
//*******************************************************************************************
// PCF8576 command decoder and data collector
// Receive bytes intended for PCF8576 chips and if commands, decode them to local variables and flags.  
// These are used later for controlling our new display.
// If data, loads to local array.
//
    while (1 < Wire.available())                        // loop through all but the last
      {                                                 // note:  this while loop takes 62 uS
      byte indata = Wire.read();                        // get byte of data from receive buffer
        if (cmdbyte){                                   // if command byte, decode command
          if (indata & maskC){                          // if the C bit is set, the next byte received after this is also a command byte
            cmdbyte = true;
          }
          else
          {
            cmdbyte = false;                            // next and remaining bytes will be data
          }
                                                        // now we will decode the command byte
          if (indata & bit6 == false){                  // if bit 6 is 0, this is a load data pointer command
            LCDpointer = indata & maskP; 
          }
          else
          {
            if (indata & bit5 == false){                 // if bit 6 is 1 and bit 5 is 0, this is a mode set command
              LCDpowerMode = indata & maskL;
              LCDenable = indata & maskE;
              LCDbias = indata & maskB;
              LCDmode = indata & maskM;
            }
            else                                          // bits 5 and 6 are 1
            {
              if (indata & bit4 == false){                // bits 5 and 6 are 1, bit 4 is 0, this is a device select command
                LCDdevice = indata & maskAd;
              }
              else
              {
                if (indata & bit3 == false){               // bits 6,5,4 are 1 and bit 3 is 0, this is a blink command
                  LCDblinkMode = indata & maskA;
                  LCDblinkFreq = indata & maskF;
                }
                else                                       // last remaining command is bank select
                {
                  LCDinBank = indata & maskI;
                  LCDoutBank = indata & maskO;
                }
              }
            }
          }
        }   
        else                                                // data byte
        {
           LCDdata[LCDpointer++] = indata;                   // get data byte, increment pointer for next data byte
        }
      }  //end while
      digitalWrite(7, LOW);                                 // turn off bit 7
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
    channel10 = segment2number(LCDdata[6]);                 // Channel data
    channel1 = segment2number(LCDdata[7]);
                                
    LCDdecimalpoint = LCDdata[0] & bit7;                    // other LCD symbols
    LCDprog = LCDdata[1] & bit7;
    LCDRX = LCDdata[2] & bit7;
    LCDexternal = LCDdata[3] & bit7;
    LCDch = LCDdata[4] & bit7;
    LCDmhz = LCDdata[5] & bit7;
    LCDleftarrow = LCDdata[6] & bit7;
    LCDrightarrow = LCDdata[7] & bit7;
    LCDhi = LCDdata[8] & bit0;
    LCDm = LCDdata[8] & bit1;
    LCDlo = LCDdata[8] & bit2;
    LCDf = LCDdata[8] & bit3;
    LCDk = LCDdata[8] & bit4;
    LCDbar = LCDdata[9] * B00000111;                        // expected values: 0, 1, 3, 7

//******************************************************************************************
// Display all collected data to OLED display



    lcd.setCursor(0, 0);                  //lcd.setCursor(col, row) 
    lcd.print(disp.charAt(MHz10));
    lcd.print(disp.charAt(MHz1));
    lcd.print(".");
    lcd.print(disp.charAt(kHz100));
    lcd.print(disp.charAt(kHz10));
    lcd.print(disp.charAt(kHz1));
    lcd.print(disp.charAt(Hz100));
    lcd.print(" MHz Ch ");
    lcd.print(disp.charAt(channel1));

    lcd.setCursor(0, 1);                  //lcd.setCursor(col, row) 
//    lcd.print(LCDdata[0],HEX);
//    lcd.print(LCDdata[1],HEX);
//    lcd.print(LCDdata[2],HEX);
//    lcd.print(LCDdata[3],HEX);
//    lcd.print(LCDdata[4],HEX);
//    lcd.print(LCDdata[5],HEX);
//    lcd.print(LCDdata[6],HEX);
//    lcd.print(LCDdata[7],HEX);
//    lcd.print(MHz10);
//    lcd.print(MHz1);
//    lcd.print(kHz100);
//    lcd.print(kHz10);
//    lcd.print(kHz1);
//    lcd.print(Hz100);
//    lcd.print(" ");
//    lcd.print(channel10);
//    lcd.print(channel1);

   
      
    }  //end of receive event

    
//********************************************************************************************
// Converts segment data to number or character by basic pattern match.
// Data comes in as segments xGFEDCBA where top bit is a don't care
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
  else if (segment == B00111110){                     // pattern is "6"  
    return 6;
  }
  else if (segment == B11100000){                     // pattern is "7"  
    return 7;
  }
  else if (segment == B11111110){                     // pattern is "8"  
    return 8;
  }
  else if (segment == B11100110){                     // pattern is "9"  
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
  else if (segment == B00000000){                     // pattern is blank
    return 14;
  }
  else{
    return 15;                                        // unrecognized pattern
  }
}




