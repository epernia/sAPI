/* Copyright 2015-2017, Eric Pernia.
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

/* Date: 2015-08-07 */

#ifndef _SAPI_UART_H_
#define _SAPI_UART_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"
#include "sapi_board_map.h"
#include "sapi_soc_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#define uartConfig uartInit

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// UART RX Interrupt Enable/Disable
void uartRxInterruptSet( int32_t uart, bool_t enable );
// UART TX Interrupt Enable/Disable
void uartTxInterruptSet( int32_t uart, bool_t enable );

// UART RX Interrupt set callback function that is excecuted when event ocurrs
void uartRxInterruptCallbackSet( 
   int32_t uart,                    // UART
   callBackFuncPtr_t rxIsrCallback  // pointer to function
);
// UART TX Interrupt set callback function that is excecuted when event ocurrs
void uartTxInterruptCallbackSet( 
   int32_t uart,                    // UART
   callBackFuncPtr_t txIsrCallback  // pointer to function
);

//-------------------------------------------------------------

// Return TRUE if have unread data in RX FIFO
bool_t uartRxReady( int32_t uart );
// Return TRUE if have space in TX FIFO
bool_t uartTxReady( int32_t uart );
// Read from RX FIFO
uint8_t uartRxRead( int32_t uart );
// Write in TX FIFO
void uartTxWrite( int32_t uart, uint8_t value );

//-------------------------------------------------------------

// UART Initialization
void uartInit( int32_t uart, uint32_t baudRate );

// Read 1 byte from RX FIFO, check first if exist aviable data
bool_t uartReadByte( int32_t uart, uint8_t* receivedByte );
// Blocking Write 1 byte to TX FIFO
void uartWriteByte( int32_t uart, uint8_t value );

// Blocking Send a string
void uartWriteString( int32_t uart, char* str );

/*==================[ISR external functions declaration]======================*/

// 0x28 0x000000A0 - Handler for ISR UART0 (IRQ 24)
void UART0_IRQHandler(void);
// 0x2a 0x000000A8 - Handler for ISR UART2 (IRQ 26)
void UART2_IRQHandler(void);
// 0x2b 0x000000AC - Handler for ISR UART3 (IRQ 27)
void UART3_IRQHandler(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_UART_H_ */
