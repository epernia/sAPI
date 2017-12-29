/* Copyright 2017, Eric Pernia.
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

/* Date: 2017-01-07 */

#ifndef _SAPI_UART_PARSE_H_
#define _SAPI_UART_PARSE_H_

/*==================[inclusions]=============================================*/

#include "sapi_uart.h"
#include "sapi_delay.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

// Receive string state
typedef enum{
   // Working states
   UART_RECEIVE_STRING_INIT,
   UART_RECEIVE_STRING_RECEIVING,
   // Ok state
   UART_RECEIVE_STRING_RECEIVED_OK,
   UART_RECEIVE_STRING_RECEIVED_OK_ALL,
   // Error states
   UART_RECEIVE_STRING_ERROR, // Por ahora todos los Errores dan este como que este mal el tamaño de un vector
   UART_RECEIVE_STRING_FULL_BUFFER,
   UART_RECEIVE_STRING_TIMEOUT
} waitForReceiveStringsOrTimeoutState_t;

// Several Strings receive mode
typedef enum{
   UART_RECEIVE_STRING_FIRST,  // Receive until first match
   UART_RECEIVE_STRING_ALL,    // Continue receiving until receive all strings
   UART_RECEIVE_STRING_NO_SAVE // No save result
} waitForReceiveStringsOrTimeoutMode_t;

// Several Strings
typedef struct{
   char*           str;
   uint16_t        strLen;
   int32_t         receiveStrStartIndex; // -1 not receive, 0 or more if receive
   uint16_t        index;
   bool_t          complete;
} expectString_t;

// Several Strings
typedef struct{
   waitForReceiveStringsOrTimeoutState_t state;
   waitForReceiveStringsOrTimeoutMode_t  mode;
   expectString_t* strList;
   uint16_t        strListSize;
   char*           receiveStr;
   uint32_t        receiveStrLen;
   uint32_t        receiveStrMaxLen;
   tick_t          timeout;
   delay_t         delay;
} waitForReceiveStringsOrTimeout_t;

// Only one String
typedef struct{
   waitForReceiveStringsOrTimeoutState_t state;
   char*    string;
   uint16_t stringSize;
   uint16_t stringIndex;
   tick_t   timeout;
   delay_t  delay;
} waitForReceiveStringOrTimeout_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Check for Receive a given pattern
waitForReceiveStringsOrTimeoutState_t waitForReceiveStringOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance );

// Recibe bytes hasta que llegue el string patron que se le manda en el
// parametro string, stringSize es la cantidad de caracteres del string.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
// No almacena los datos recibidos!! Simplemente espera a recibir cierto patron.
bool_t waitForReceiveStringOrTimeoutBlocking( 
   int32_t uart, 
   char* string, uint16_t strLen, 
   tick_t timeout );

// Recibe bytes hasta que lleguen los string patrones o uno de ellos segun el
// modo
waitForReceiveStringsOrTimeoutState_t waitForReceiveStringsOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance );

// Store bytes until receive a given pattern
waitForReceiveStringsOrTimeoutState_t receiveBytesUntilReceiveStringOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance,
   char* receiveBuffer, uint32_t* receiveBufferSize );

// Guarda todos los bytes que va recibiendo hasta que llegue el string
// patron que se le manda en el parametro string, stringSize es la cantidad
// de caracteres del string.
// receiveBuffer es donde va almacenando los caracteres recibidos y
// receiveBufferSize es el tamaño de buffer receiveBuffer.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
bool_t receiveBytesUntilReceiveStringOrTimeoutBlocking(
   int32_t uart, char* string, uint16_t stringSize,
   char* receiveBuffer, uint32_t* receiveBufferSize,
   tick_t timeout );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_UART_PARSE_H_ */
