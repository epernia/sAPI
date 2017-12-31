/*==============================================================================
* Copyright (c) 2017, Ariel Berardi
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Library based on "arun singh" code
* http://controllerstech.blogspot.com.ar/2017/07/i2c-lcd-in-stm32.html 
====================[2017-12-02]==============================================*/

#ifndef _SAPI_DISPLAYI2C_H_
#define _SAPI_DISPLAYI2C_H_

/*==================[inclusions]==============================================*/

#include "sapi_i2c.h"

/*==================[cplusplus]===============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]==================================================*/

// Commands
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT  0x00
#define LCD_ENTRYLEFT   0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON   0x04
#define LCD_DISPLAYOFF  0x00
#define LCD_CURSORON    0x02
#define LCD_CURSOROFF   0x00
#define LCD_BLINKON     0x01
#define LCD_BLINKOFF    0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

// Flags for function set
#define LCD_8BITMODE    0x10
#define LCD_4BITMODE    0x00
#define LCD_2LINE       0x08
#define LCD_1LINE       0x00
#define LCD_5x10DOTS    0x04
#define LCD_5x8DOTS     0x00

// Flags for backlight control
#define LCD_BACKLIGHT   0x08
#define LCD_NOBACKLIGHT 0x00

/*==================[typedef]=================================================*/

typedef struct{
   uint8_t address;
   uint8_t col;
   uint8_t row;
   uint8_t functionSet;
   uint8_t displayControl;
   uint8_t entryModeSet;
   uint8_t backlight;
}displayi2c_t;

/*==================[external data declaration]===============================*/
/*==================[external functions declaration]==========================*/

// Configure display
void displayi2cInit(displayi2c_t *disp, uint8_t address, uint8_t col, 
	uint8_t row);

// Print strings on display
void displayi2cPrint(displayi2c_t *disp, uint8_t *str);

// Clean text
void displayi2cClean(displayi2c_t *disp);

// Set cursor at origin position
void displayi2cReturnHome(displayi2c_t *disp);

// Change cursor position
void displayi2cSetCursor(displayi2c_t *disp, uint8_t col, uint8_t row);

// Turn on/off backlight
void displayi2cBacklight(displayi2c_t *disp, bool_t toggle);

/*==================[cplusplus]===============================================*/

#ifdef __cplusplus
}
#endif

/*==================[examples]================================================*/
/*==================[end of file]=============================================*/

#endif /* _SAPI_DISPLAYI2C_H_ */