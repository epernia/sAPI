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

/* Date: 2016-02-26 */

#ifndef _SAPI_UART_H_
#define _SAPI_UART_H_

/*==================[inclusions]=============================================*/

#include "sapi_delay.h"
#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef enum{
   // Working states
   UART_RECEIVE_STRING_INIT,
   UART_RECEIVE_STRING_RECEIVING,
   // Ok state
   UART_RECEIVE_STRING_RECEIVED_OK,
   UART_RECEIVE_STRING_RECEIVED_OK_ALL,
   // Error states
   UART_RECEIVE_STRING_ERROR, // Por ahora todos los Errores dan este como que este mal el tamaño de un vector
   UART_RECEIVE_STRING_TIMEOUT
} waitForReceiveStringsOrTimeoutState_t;

typedef enum{
   UART_RECEIVE_STRING_FIRST,  // Receive until first match
   UART_RECEIVE_STRING_ALL,    // Continue receiving until receive all strings
   UART_RECEIVE_STRING_NO_SAVE // No save result
} waitForReceiveStringsOrTimeoutMode_t;

typedef struct{
   char*           str;
   uint16_t        strLen;
   int32_t         receiveStrStartIndex; // -1 not receive, 0 or more if receive
   uint16_t        index;
   bool_t          complete;
} expectString_t;

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

/*
typedef struct{
   waitForReceiveStringOrTimeoutState_t state;
   char*    string;
   uint16_t stringSize;
   uint16_t stringIndex;
   tick_t   timeout;
   delay_t  delay;
} waitForReceiveStringOrTimeout_t;
*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Receive

waitForReceiveStringsOrTimeoutState_t waitForReceiveStringsOrTimeout(
   uartMap_t uart, waitForReceiveStringsOrTimeout_t* instance );


bool_t waitForReceiveStringOrTimeoutBlocking(
   uartMap_t uart, char* str, uint16_t strLen, tick_t timeout );



void uartConfig( uartMap_t uart, uint32_t baudRate );

bool_t uartReadByte( uartMap_t uart, uint8_t* receivedByte );
void uartWriteByte( uartMap_t uart, uint8_t byte );

void uartWriteString( uartMap_t uart, char* str );

/*==================[ISR external functions declaration]======================*/

/* 0x28 0x000000A0 - Handler for ISR UART0 (IRQ 24) */
void UART0_IRQHandler(void);
/* 0x2a 0x000000A8 - Handler for ISR UART2 (IRQ 26) */
void UART2_IRQHandler(void);
/* 0x2b 0x000000AC - Handler for ISR UART3 (IRQ 27) */
void UART3_IRQHandler(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_UART_H_ */
