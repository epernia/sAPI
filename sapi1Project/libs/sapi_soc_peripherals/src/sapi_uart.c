/* Copyright 2014, Pablo Ridolfi (UTN-FRBA).
 * Copyright 2014, Juan Cecconi.
 * Copyright 2015-2017, Eric Pernia.
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

/*==================[inclusions]=============================================*/

#include "sapi_uart.h"

#include "string.h"
#include "sapi_circularBuffer.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef struct{
   LPC_USART_T*      uartAddr;
   lpc4337ScuPin_t   txPin;
   lpc4337ScuPin_t   rxPin;
   IRQn_Type         uartIrqAddr;
} uartLpcConfig_t;

/*==================[internal data declaration]==============================*/

static volatile callBackFuncPtr_t rxIsrCallbackUART0 = 0;
static volatile callBackFuncPtr_t rxIsrCallbackUART2 = 0;
static volatile callBackFuncPtr_t rxIsrCallbackUART3 = 0;

static volatile callBackFuncPtr_t txIsrCallbackUART0 = 0;
static volatile callBackFuncPtr_t txIsrCallbackUART2 = 0;
static volatile callBackFuncPtr_t txIsrCallbackUART3 = 0;

static const uartLpcConfig_t lpcUarts[] = {
// { uartAddr, { txPort, txpin, txfunc }, { rxPort, rxpin, rxfunc }, uartIrqAddr  },
   // UART_GPIO (GPIO1 = U0_TXD, GPIO2 = U0_RXD)
   { LPC_USART0, { 6, 4, FUNC2 }, { 6, 5, FUNC2 }, USART0_IRQn }, // 0
   // UART_485 (RS485/Profibus)
   { LPC_USART0, { 9, 5, FUNC7 }, { 9, 6, FUNC7 }, USART0_IRQn }, // 1
   // UART not routed
   {  LPC_UART1, { 0, 0, 0     }, { 0, 0, 0     }, UART1_IRQn  }, // 2
   // UART_USB
   { LPC_USART2, { 7, 1, FUNC6 }, { 7, 2, FUNC6 }, USART2_IRQn }, // 3
   // UART_ENET
   { LPC_USART2, { 1,15, FUNC1 }, { 1,16, FUNC1 }, USART2_IRQn }, // 4
   // UART_232
   { LPC_USART3, { 2, 3, FUNC2 }, { 2, 4, FUNC2 }, USART3_IRQn }  // 5   
};

static const lpc4337ScuPin_t lpcUart485DirPin = {
   6, 2, FUNC2
};


/*
   callBackFuncPtr_t txIsrCallback;
   callBackFuncPtr_t rxIsrCallback;
*/

/*==================[internal functions declaration]=========================*/

static void uartProcessIRQ( uartMap_t uart );

/*==================[internal functions definition]==========================*/

static void uartProcessIRQ( uartMap_t uart )
{
   uint8_t status = Chip_UART_ReadLineStatus( lpcUarts[uart].uartAddr );

   // Rx Interrupt
   if(status & UART_LSR_RDR) // uartRxReady
   {
      // Execute callback      
      if( ( uart == UART_GPIO ) && (rxIsrCallbackUART0 != 0) )
         (*rxIsrCallbackUART0)();
      
      if( ( uart == UART_USB )  && (rxIsrCallbackUART2 != 0) )
         (*rxIsrCallbackUART2)();
      
      if( ( uart == UART_232 )  && (rxIsrCallbackUART3 != 0) )
         (*rxIsrCallbackUART3)();
   }
   
   // Tx Interrupt
   if( ( status & UART_LSR_THRE ) && // uartTxReady
       ( Chip_UART_GetIntsEnabled(LPC_USART3) & UART_IER_THREINT ) ){

      // Execute callback      
      if( ( uart == UART_GPIO ) && (txIsrCallbackUART0 != 0) )
         (*txIsrCallbackUART0)();
      
      if( ( uart == UART_USB )  && (txIsrCallbackUART2 != 0) )
         (*txIsrCallbackUART2)();
      
      if( ( uart == UART_232 )  && (txIsrCallbackUART3 != 0) )
         (*txIsrCallbackUART3)();
   }
}

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// UART RX Interrupt Enable/Disable
void uartRxInterruptSet( int32_t uart, bool_t enable ){
   if( enable ){  
      // Enable UART Receiver Buffer Register Interrupt
      Chip_UART_IntEnable( lpcUarts[uart].uartAddr, UART_IER_RBRINT );
      // Enable UART line status interrupt. LPC43xx User manual page 1118
      //NVIC_SetPriority( lpcUarts[uart].uartIrqAddr, 6 );
      // Enable Interrupt for UART channel
      NVIC_EnableIRQ( lpcUarts[uart].uartIrqAddr );
   } else{
      // Disable UART Receiver Buffer Register Interrupt
      Chip_UART_IntDisable( lpcUarts[uart].uartAddr, UART_IER_RBRINT );
      // Disable Interrupt for UART channel
      NVIC_DisableIRQ( lpcUarts[uart].uartIrqAddr );
   }
}

// UART TX Interrupt Enable/Disable
void uartTxInterruptSet( int32_t uart, bool_t enable ){
   if( enable ){  
      // Enable THRE irq (TX)
      Chip_UART_IntEnable( lpcUarts[uart].uartAddr, UART_IER_THREINT );
      NVIC_EnableIRQ( lpcUarts[uart].uartIrqAddr );
   } else{
      // Disable THRE irq (TX)
      Chip_UART_IntDisable( lpcUarts[uart].uartAddr, UART_IER_THREINT );
      NVIC_DisableIRQ( lpcUarts[uart].uartIrqAddr );
   }
}

// UART RX Interrupt set callback function that is excecuted when event ocurrs
void uartRxInterruptCallbackSet( 
   int32_t uart,                    // UART
   callBackFuncPtr_t rxIsrCallback  // pointer to function
){
   if( rxIsrCallback != 0 ){
      // Set callback
      if( uart == UART_GPIO ) rxIsrCallbackUART0 = rxIsrCallback;
      if( uart == UART_485  ) rxIsrCallbackUART0 = rxIsrCallback;
      if( uart == UART_USB  ) rxIsrCallbackUART2 = rxIsrCallback;
      if( uart == UART_ENET ) rxIsrCallbackUART2 = rxIsrCallback;
      if( uart == UART_232  ) rxIsrCallbackUART3 = rxIsrCallback;      
   }
}

// UART TX Interrupt set callback function that is excecuted when event ocurrs
void uartTxInterruptCallbackSet( 
   int32_t uart,                    // UART
   callBackFuncPtr_t txIsrCallback  // pointer to function
){
   if( txIsrCallback != 0 ){
      // Set callback
      if( uart == UART_GPIO ) txIsrCallbackUART0 = txIsrCallback;
      if( uart == UART_485  ) txIsrCallbackUART0 = txIsrCallback;
      if( uart == UART_USB  ) txIsrCallbackUART2 = txIsrCallback;
      if( uart == UART_ENET ) txIsrCallbackUART2 = txIsrCallback;
      if( uart == UART_232  ) txIsrCallbackUART3 = txIsrCallback;      
   }
}

// disable tx and rx interrupt
//Chip_UART_IntDisable( LPC_USART2, 
//                      UART_IER_THREINT |
//                      UART_IER_RBRINT );

//case STARTTX:
   // disable THRE irq (TX)
   //Chip_UART_IntDisable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
   // this one calls write
   //ciaaDriverUart_txConfirmation(device);
   // enable THRE irq (TX)
   //Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
//break;
//case SET_FIFO_TRIGGER_LEVEL:
   //Chip_UART_SetupFIFOS((LPC_USART_T *)device->loLayer,  UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | (int32_t)param);
//break;

//#define UART_RX_FIFO_SIZE       (16)


//-------------------------------------------------------------

// Return TRUE if have unread data in RX FIFO
bool_t uartRxReady( int32_t uart ){
   return Chip_UART_ReadLineStatus( lpcUarts[uart].uartAddr ) & UART_LSR_RDR;
}
// Return TRUE if have space in TX FIFO
bool_t uartTxReady( int32_t uart ){
   return Chip_UART_ReadLineStatus( lpcUarts[uart].uartAddr ) & UART_LSR_THRE; 
}
// Read from RX FIFO
uint8_t uartRxRead( int32_t uart ){
   return Chip_UART_ReadByte( lpcUarts[uart].uartAddr );
}
// Write in TX FIFO
void uartTxWrite( int32_t uart, uint8_t value ){
   Chip_UART_SendByte( lpcUarts[uart].uartAddr, value );
}

//-------------------------------------------------------------

// UART Initialization
void uartConfig( int32_t uart, uint32_t baudRate ){
   // Initialize UART
   Chip_UART_Init( lpcUarts[uart].uartAddr );
   // Set Baud rate
   Chip_UART_SetBaud( lpcUarts[uart].uartAddr, baudRate );
   // Restart FIFOS using FCR (FIFO Control Register).
   // Set Enable, Reset content, set trigger level
   Chip_UART_SetupFIFOS( lpcUarts[uart].uartAddr, 
                         UART_FCR_FIFO_EN | 
                         UART_FCR_TX_RS   | 
                         UART_FCR_RX_RS   | 
                         UART_FCR_TRG_LEV0 );
   // Dummy read
   Chip_UART_ReadByte( lpcUarts[uart].uartAddr );
   // Enable UART Transmission
   Chip_UART_TXEnable( lpcUarts[uart].uartAddr );
   // Configure SCU UARTn_TXD pin
   Chip_SCU_PinMux( lpcUarts[uart].txPin.lpcScuPort, 
                    lpcUarts[uart].txPin.lpcScuPin, 
                    MD_PDN,
                    lpcUarts[uart].txPin.lpcScuFunc );
   // Configure SCU UARTn_RXD pin
   Chip_SCU_PinMux( lpcUarts[uart].rxPin.lpcScuPort, 
                    lpcUarts[uart].rxPin.lpcScuPin, 
                    MD_PLN | MD_EZI | MD_ZI,
                    lpcUarts[uart].rxPin.lpcScuFunc );
   
   // Specific configurations for RS485
   if( uart == UART_485 ){
      // Specific RS485 Flags
      Chip_UART_SetRS485Flags( LPC_USART0,
                               UART_RS485CTRL_DCTRL_EN | 
                               UART_RS485CTRL_OINV_1     );
      // UARTn_DIR extra pin for RS485
      Chip_SCU_PinMux( lpcUart485DirPin.lpcScuPort, 
                       lpcUart485DirPin.lpcScuPin, 
                       MD_PDN, 
                       lpcUart485DirPin.lpcScuFunc );
   }
}

// Read 1 byte from RX FIFO, check first if exist aviable data
bool_t uartReadByte( int32_t uart, uint8_t* receivedByte ){
   bool_t retVal = TRUE;
   if ( uartRxReady(uart) ){
      *receivedByte = uartRxRead(uart);
   } else{
      retVal = FALSE;
   }
   return retVal;
}

// Blocking Write 1 byte to TX FIFO
void uartWriteByte( int32_t uart, uint8_t value ){
   // Wait for space in FIFO (blocking)
   while( uartTxReady( uart ) == FALSE );
   // Send byte
   uartTxWrite( uart, value );
}

// Blocking Send a string
void uartWriteString( int32_t uart, char* str ){
   while( *str != 0 ){
      uartWriteByte( uart, (uint8_t)*str );
      str++;
   }
}

/*==================[ISR external functions definition]======================*/

__attribute__ ((section(".after_vectors")))

// UART0 (GPIO1 y GPIO2 or RS485/Profibus) 
// 0x28 0x000000A0 - Handler for ISR UART0 (IRQ 24)
void UART0_IRQHandler(void){
   uartProcessIRQ( UART_GPIO );
}

// UART2 (USB-UART) or UART_ENET
// 0x2a 0x000000A8 - Handler for ISR UART2 (IRQ 26)
void UART2_IRQHandler(void){
   uartProcessIRQ( UART_USB );
}

// UART3 (RS232)
// 0x2b 0x000000AC - Handler for ISR UART3 (IRQ 27)
void UART3_IRQHandler(void){
   uartProcessIRQ( UART_232 );
}

/*==================[end of file]============================================*/
