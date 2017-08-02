/* Copyright 2015, Eric Pernia.
 * Copyright 2016, Ian Olivieri.
 * Copyright 2016-2017, Eric Pernia.
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
 */

/* Date: 2015-09-23 */

#ifndef _SAPI_PERIPHERALMAP_H_
#define _SAPI_PERIPHERALMAP_H_

/*==================[inclusions]=============================================*/

#include "lpc4337_soc_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef enum{
   // P1 header
   T_FIL1 = P4_1,
   T_COL2 = P7_5,
   T_COL0 = P1_5,
   T_FIL2 = P4_2,
   T_FIL3 = P4_3,
   T_FIL0 = P4_0,
   T_COL1 = P7_4,
   
   CAN_TD = P3_2,
   CAN_RD = P3_1,
   
   RS232_TXD = P2_3,
   RS232_RXD = P2_4,

   // P2 header
   GPIO8 = P6_12,
   GPIO7 = P6_11,
   GPIO5 = P6_9,
   GPIO3 = P6_7,
   GPIO1 = P6_4,
   
   LCD1  = P4_4,      
   LCD2  = P4_5,      
   LCD3  = P4_6,     
   LCDRS = P4_8,      
   LCD4  = P4_10,
   
   SPI_MISO = P1_3,
   
   ENET_TXD1   = P1_20,
   ENET_TXD0   = P1_18,
   ENET_MDIO   = P1_17,
   ENET_CRS_DV = P1_16,
   ENET_MDC    = P7_7,
   ENET_TXEN   = P0_1,
   ENET_RXD1   = P0_0,
   
   GPIO6 = P6_10,
   GPIO4 = P6_8,
   GPIO2 = P6_5,
   GPIO0 = P6_1,
   
   LCDEN = P4_9,
   
   SPI_MOSI = P1_4,
   
   ENET_RXD0 = P1_15,

   // Switches
   TEC1 = P1_0,
   TEC2 = P1_1,
   TEC3 = P1_2,
   TEC4 = P1_6,

   // Leds
   LEDR = P2_0,
   LEDG = P2_1,
   LEDB = P2_2,
   LED1 = P2_10,
   LED2 = P2_11,
   LED3 = P2_12
} gpioBoardMap_t;

typedef enum{
   CH3 = ADC0_3, 
   CH2 = ADC0_2, 
   CH1 = ADC0_1
} adcBoardMap_t;

typedef enum{
   UART_USB = UART2,   // UART2 (USB-UART)
   UART_232 = UART3,   // UART3 (RS232)
   UART_485 = UART0    // UART0 (RS485/Profibus)    
} uartBoardMap_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_PERIPHERALMAP_H_ */
