/*  TWI (I2C) code to drive 128x64 monochrome oled display modules
    ceptimus.  September 2016
    Edited by Benik3 January 2019
*/
#ifndef SSD1X06_h
#define SSD1X06_h

#include "Arduino.h"
#include "ssdfont.h"

//#define SSD1106 // comment out one or other of these lines so as to leave the definition active for the type of display driver you have
#define SSD1306 // comment out one or other of these lines so as to leave the definition active for the type of display driver you have

#define SSD1X06_I2C_ADDRESS 0x3C // some modules can be 0x3D - depends on how module's internal SA0 has been wired

#define I2CSPEED 800000L    //normally display communicate on 400kHz. From testing it can handle faster clock up to 800kHz. In case of problems try 400000L

// pixels
#define SSD1X06_LCDWIDTH  128
#define SSD1X06_LCDHEIGHT  64
#define SSD1X06_COLUMNOFFSET 0

// display chars assuming 6x8 font
#define SSD1X06_CHARWIDTH (SSD1X06_LCDWIDTH / 6)
#define SSD1X06_CHARHEIGHT (SSD1X06_LCDHEIGHT / 8)
#define SSD1X06_NUMCHARS (SSD1X06_CHARWIDTH * SSD1X06_CHARHEIGHT)

// give names to the command bytes the display module understands - see datasheet.
#define SSD1X06_SETCONTRAST 0x81
#define SSD1X06_DISPLAYALLON_RESUME 0xA4
#define SSD1X06_DISPLAYALLON 0xA5
#define SSD1X06_NORMALDISPLAY 0xA6
#define SSD1X06_INVERTDISPLAY 0xA7
#define SSD1X06_DISPLAYOFF 0xAE
#define SSD1X06_DISPLAYON 0xAF

#define SSD1X06_SETDISPLAYOFFSET 0xD3
#define SSD1X06_SETCOMPINS 0xDA

#define SSD1X06_SETVCOMDETECT 0xDB

#define SSD1X06_SETDISPLAYCLOCKDIV 0xD5
#define SSD1X06_SETPRECHARGE 0xD9

#define SSD1X06_SETMULTIPLEX 0xA8

#define SSD1X06_SETLOWCOLUMN 0x00
#define SSD1X06_SETHIGHCOLUMN 0x10

#define SSD1X06_COLUMNADDR 0x21
#define SSD1X06_PAGEADDR   0x22

#define SSD1X06_SETSTARTLINE 0x40
#define SSD1X06_SETSTARTPAGE 0XB0
#define SSD1X06_MEMORYMODE 0x20

#define SSD1X06_COMSCANINC 0xC0
#define SSD1X06_COMSCANDEC 0xC8

#define SSD1X06_SEGREMAP 0xA0

#define SSD1X06_CHARGEPUMP 0x8D

#define SSD1X06_EXTERNALVCC 0x1
#define SSD1X06_SWITCHCAPVCC 0x2

class SSD1X06 {
  public:
    static void start(void);
    static void fillDisplay(uint8_t c); // fill whole display with (c) characters.  c is most likely ' ' or (' ' | 0x80) for reversed field
    static void displayString6x8(uint8_t row, uint8_t x, const char *s, uint8_t rvsField); // if rvsField != 0 display characters in string with reversed field
    static void displayString6x8(uint8_t row, uint8_t x, const __FlashStringHelper *s, uint8_t rvsField); // same as above for constant (FLASH memory) stored strings
    static void displayByte(uint8_t row, uint8_t x, uint8_t b); // display one vertical bar of 8 pixels - bit 0 at top, bit 7 at bottom
    static void drawLine(uint8_t row, uint8_t x, uint8_t nrow, uint8_t *b); // draw line from array of bytes
  private:
    static void displayChar6x8(uint8_t row, uint8_t x, uint8_t c);
    static void SetColmnPage(uint8_t row, uint8_t x);
};

#endif
