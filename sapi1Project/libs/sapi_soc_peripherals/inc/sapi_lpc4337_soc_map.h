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
 */

/* Date: 2015-09-23 */

#ifndef _SAPI_LPC4337_SOC_MAP_H_
#define _SAPI_LPC4337_SOC_MAP_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/* ----- Begin Pin Config Structs NXP LPC4337 ----- */

typedef struct{
   int8_t port;
   int8_t pin;
} pinConfigLpc4337_t;

/* ------ End Pin Config Structs NXP LPC4337 ------ */


/* ------- Begin EDU-CIAA-NXP Peripheral Map ------ */

/* Defined for sapi_gpio.h */
typedef enum{
   /* EDU-CIAA-NXP */

   // P1 header
   P4_1,
   P7_5,
   P1_5,
   P4_2,
   P4_3,
   P4_0,
   P7_4,

   P3_2,
   P3_1,

   P2_3,
   P2_4,

   // P2 header
   P6_12,
   P6_11,
   P6_9,
   P6_7,
   P6_4,

   P4_4,
   P4_5,
   P4_6,
   P4_8,
   P4_10,

   P1_3,

   P1_20,
   P1_18,
   P1_17,
   P1_16,
   P7_7,
   P0_1,
   P0_0,

   P6_10,
   P6_8,
   P6_5,
   P6_1,

   P4_9,

   P1_4,

   P1_15,

   // Switches
   // 36   37     38     39
   P1_0,
   P1_1,
   P1_2,
   P1_6,

   // Leds
   // 40   41     42     43     44     45
   P2_0,
   P2_1,
   P2_2,
   P2_10,
   P2_11,
   P2_12,


   /* CIAA-NXP */ /// Ver!
 /* 46     47     48     49     50     51     52     53 */
   DI0,
   DI1,
   DI2,
   DI3,
   DI4,
   DI5,
   DI6,
   DI7,
 /* 54     55     56     57     58     59     60     61 */
   DO0,
   DO1,
   DO2,
   DO3,
   DO4,
   DO5,
   DO6,
   DO7
} gpioSocMap_t;

/* Defined for sapi_adc.h */
typedef enum{
   ADC0_CH4 = 62,   // Ver CIAA NXP
   ADC0_CH3 = 63,
   ADC0_CH2 = 64,
   ADC0_CH1 = 65
} adcSocMap_t;

/* Defined for sapi_dac.h */
typedef enum{
   DAC
} dacSocMap_t;

/* Defined for sapi_uart.h */
typedef enum{
   UART0,
   UART2,
   UART3
} uartSocMap_t;

/*Defined for sapi_timer.h*/
//NOTE: if servo is enable (servoConfig used) the only available timer to use is TIMER0
typedef enum{
   TIMER0,
   TIMER1,
   TIMER2,
   TIMER3,
   SYSTICK,
   SCT
} timerSocMap_t;

/*Defined for sapi_i2c.h*/
/* Comment because already defined in "i2c_18xx_43xx.h"*/
/*
typedef enum{
   I2C0 // TODO: Add support for I2C1
} i2cMap_t;
*/
typedef uint8_t i2cSocMap_t;

typedef enum{
   SPI0
} spiSocMap_t;

/* ------- End EDU-CIAA-NXP Peripheral Map -------- */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_LPC4337_SOC_MAP_H_ */
