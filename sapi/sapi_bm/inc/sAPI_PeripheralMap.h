/* Copyright 2015, Eric Pernia.
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

#ifndef _SAPI_PERIPHERALMAP_H_
#define _SAPI_PERIPHERALMAP_H_
/** @brief Brief for this header file.
 **
 ** Full description for this header file.
 **
 **/

/** \addtogroup groupName Group Name
 ** @{ */

/*
 * Initials    Name
 * ---------------------------
 * ENP          Eric Pernia
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150923   v0.0.1   First version
 */

/*==================[inclusions]=============================================*/

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

/* Defined for sAPI_DigitalIO.h */
typedef enum{

	DIO0,  DIO1,  DIO2,  DIO3,  DIO4,  DIO5,  DIO6,  DIO7,  DIO8,  DOI9,
   DIO10, DIO11, DIO12, DIO13, DIO14, DIO15, DIO16, DIO17, DIO18, DIO19,
   DIO20, DIO21, DIO22, DIO23, DIO24, DIO25, DIO26, DIO27, DIO28, DIO29,
   DIO30, DIO31, DIO32, DIO33, DIO34, DIO35,
   TEC1,  TEC2,  TEC3,  TEC4,
   LED1,  LED2,  LED3,  LEDR,  LEDG,  LEDB,

	DI0,   DI1,   DI2,   DI3,   DI4,   DI5,   DI6,   DI7,
	DO0,   DO1,   DO2,   DO3,   DO4,   DO5,   DO6,   DO7
} DigitalIOMap_t;

/* Defined for sAPI_AnalogIO.h */
/* 46        47   48   49 */
typedef enum{
   AI2 = 46, AI1, AI0, AO
} AnalogIOMap_t;

/* Defined for sAPI_Uart.h */
typedef enum{
   UART_USB, UART_232, UART_485
} UartMap_t;

/* ------- End EDU-CIAA-NXP Peripheral Map -------- */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_PERIPHERALMAP_H_ */
