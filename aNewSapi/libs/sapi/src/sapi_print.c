/* Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
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
 */

/* Date: 2017-04-17 */

/*==================[inclusions]=============================================*/

#include "sapi_print.h"   // <= own header

#include "sapi_uart.h"    // <= UART header

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

// Print uart configuration

void printSetUart( print_t* printer, uartMap_t uart ){
   *printer = uart;
}

void printConfigUart( print_t* printer, uartMap_t uart, uint32_t baudRate ){
   *printer = uart;
   uartConfig( uart, baudRate );
}


// Print String

void printString( print_t printer, char* string ){
   uartWriteString( printer, string );
}

void printEnter( print_t printer ){
   uartWriteString( printer, PRINT_ENTER_STRING );
}


// Print Integer

void printIntFormat( print_t printer, int64_t number, numberFormat_t format ){

   char strNumber[65];

   if( int64ToString( number, strNumber, format ) ){
      uartWriteString( printer, strNumber );
   }
}

void printUIntFormat( print_t printer, uint64_t number, numberFormat_t format ){

   char strNumber[65];

   if( uint64ToString( number, strNumber, format ) ){
      uartWriteString( printer, strNumber );
   }
}

void printHex( print_t printer, uint64_t number, uint8_t bitSize ){
   printString( printer, uintToAsciiHex( number , bitSize ) );
}


// C++ version 0.4 char* style "itoa":
// Written by Luk�s Chmela
// Released under GPLv3.
// Modified by Eric Pernia.
bool_t int64ToString( int64_t value, char* result, uint8_t base ){
   // check that the base if valid
   if( base < 2 || base > 36 ){
      *result = '\0';
      return FALSE;
   }

   char* ptr = result, *ptr1 = result, tmp_char;
   int64_t tmp_value;

   do {
      tmp_value = value;
      value /= (int64_t)base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * (int64_t)base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return TRUE;
}

// C++ version 0.4 char* style "itoa":
// Written by Luk�s Chmela
// Released under GPLv3.
// Modified by Eric Pernia.
bool_t uint64ToString( uint64_t value, char* result, uint8_t base ){
   // check that the base if valid
   if( base < 2 || base > 36 ){
      *result = '\0';
      return FALSE;
   }

   char* ptr = result, *ptr1 = result, tmp_char;
   uint64_t tmp_value;

   do {
      tmp_value = value;
      value /= (uint64_t)base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * (uint64_t)base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return TRUE;
}

char* uintToAsciiHex( uint64_t value, uint8_t bitSize ){
   
   static char result[17];
   uint8_t i = 0;
   uint8_t vectorNumHex[] = "0123456789ABCDEF";
   
   result[bitSize/4] = 0;
   
   for( i=0; i<bitSize/4; i++ ){
      result[(bitSize/4)-i-1] = vectorNumHex[ (uint8_t)(( value & (((uint64_t)0x0F)<<(4*i)) ) >> (4*i)) ];
   }

   return result;
}

/*==================[end of file]============================================*/
