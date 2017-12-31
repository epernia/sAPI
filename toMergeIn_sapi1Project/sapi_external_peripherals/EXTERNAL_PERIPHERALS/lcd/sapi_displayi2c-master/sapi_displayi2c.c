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

/*==================[inclusions]==============================================*/

#include "sapi_displayi2c.h"

/*==================[macros and definitions]==================================*/
/*==================[internal data declaration]===============================*/
/*==================[internal functions declaration]==========================*/

void displayi2cCommand(displayi2c_t *disp, uint8_t cmd);
void displayi2cSendData(displayi2c_t *disp, uint8_t data);

/*==================[external data declaration]===============================*/
/*==================[internal functions definition]===========================*/

// Format and send configurations commands
void displayi2cCommand(displayi2c_t *disp, uint8_t cmd){
   uint8_t data_u, data_l;
	uint8_t data_t[4];
	
   // Split byte in high an low 4bits
   data_u = cmd & 0xf0;
	data_l = (cmd << 4) & 0xf0;
   
   // Format packet to send
	data_t[0] = data_u | 0x04 | disp->backlight;    
	data_t[1] = data_u | disp->backlight;           
	data_t[2] = data_l | 0x04 | disp->backlight;   
	data_t[3] = data_l | disp->backlight;           
   
   // 0x04 means EN=1 and RS=0
   // Always has to be send backlight state
   
   i2cWrite(I2C0, disp->address, (uint8_t *)data_t, 4, TRUE);
}

// Format and send data to show
void displayi2cSendData(displayi2c_t *disp, uint8_t data){
	uint8_t data_u, data_l;
	uint8_t data_t[4];
   
   // Split byte in high an low 4bits
	data_u = data & 0xf0;
	data_l = (data << 4) & 0xf0;
	
   // Format packet to send
   data_t[0] = data_u | 0x05 | disp->backlight; 
	data_t[1] = data_u | 0x01 | disp->backlight; 
	data_t[2] = data_l | 0x05 | disp->backlight; 
	data_t[3] = data_l | 0x01 | disp->backlight; 
   
   // 0x05 means EN=1 and RS=1
   // Always has to be send backlight state
   
   i2cWrite(I2C0, disp->address, (uint8_t *) data_t, 4, TRUE);   
}

/*==================[external functions definition]===========================*/

// Configure display
void displayi2cInit(displayi2c_t *disp, uint8_t address, uint8_t col, 
   uint8_t row){
   
   // Default configuration
   disp->address        = address;
   disp->col            = col;
   disp->row            = row;
   disp->backlight      = LCD_BACKLIGHT;
   disp->functionSet    = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
   disp->displayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
   disp->entryModeSet   = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
   
   // Start i2c port
   i2cConfig(I2C0, 10000);
   
   // Set 4bit mode
   displayi2cCommand(disp, 0x02); 
   
   // Set function set
	displayi2cCommand(disp, LCD_FUNCTIONSET | disp->functionSet);
   
   // Set display control
	displayi2cCommand(disp, LCD_DISPLAYCONTROL | disp->displayControl); 
   
   // Set entry mode set
   displayi2cCommand(disp, LCD_ENTRYMODESET | disp->entryModeSet);
   
   // Set caracter types
	displayi2cCommand(disp, LCD_SETDDRAMADDR);
}

// Print strings on display
void displayi2cPrint(displayi2c_t *disp, uint8_t *str){
   uint8_t col = 0;
   uint8_t row = 0;
   
   // Reading chars
   while(*str){
      
      // Auto-scroll cols and rows
      if(col >= disp->col){
         col = 0;
         row++;
         
         if(row >= disp->row){
            row = 0;
         }
         
         displayi2cSetCursor(disp, col, row);
      }
      
      // Send chars
      displayi2cSendData(disp, *str);
      str++;
      col++;
   }
}

// Clean text
void displayi2cClean(displayi2c_t *disp){
   displayi2cCommand(disp, LCD_CLEARDISPLAY);
}

// Set cursor at origin position
void displayi2cReturnHome(displayi2c_t *disp){
   displayi2cCommand(disp, LCD_RETURNHOME);
}

// Change cursor position
void displayi2cSetCursor(displayi2c_t *disp, uint8_t col, uint8_t row){
   uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
   
   if(row > disp->row){
      row = disp->row - 1;
   }
   
   displayi2cCommand(disp, LCD_SETDDRAMADDR | (col + row_offsets[row])); 
}

// Turn on/off backlight
void displayi2cBacklight(displayi2c_t *disp, bool_t toggle){
   if(toggle == ON){
      disp->backlight = LCD_BACKLIGHT;
   }
   else{
      disp->backlight = LCD_NOBACKLIGHT;
   }
   
   displayi2cCommand(disp, 0);
}

/*==================[end of file]=============================================*/