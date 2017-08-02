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

/* Date: 2015-09-23 */
/* Date: 2017-06-02 EFM32HG support */

#ifndef _SAPI_GPIO_H_
#define _SAPI_GPIO_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#define gpioInit()   gpioConfig( 0, GPIO_ENABLE )

/*==================[typedef]================================================*/

/* Pin modes */
/*
 *  INPUT  =  0    (No PULLUP or PULLDOWN)
 *  OUTPUT =  1
 *  INPUT_PULLUP
 *  INPUT_PULLDOWN
 *  INPUT_REPEATER (PULLUP and PULLDOWN)
 *  INITIALIZE
 */
typedef enum{
   GPIO_INPUT, GPIO_OUTPUT,
   GPIO_INPUT_PULLUP, GPIO_INPUT_PULLDOWN,
   GPIO_ENABLE, GPIO_DISABLE
} gpioConfig_t;


// Interrupt ------------------------------------

typedef sAPI_FuncPtr_t gpioInterruptCallback_t;

typedef enum{
   GPIO_INTERRUPT_DISABLE            = 0x00, // default value
   GPIO_INTERRUPT_LEVEL              = 0x10, // Level-sensitive (high/low). Modify flags:

      GPIO_LOW                       = 0x01, // default modify flag value
      GPIO_HIGH                      = 0x02,
      GPIO_BOTH                      = 0x03,

      GPIO_INTERRUPT_LEVEL_LOW       = GPIO_INTERRUPT_LEVEL | GPIO_LOW,
      GPIO_INTERRUPT_LEVEL_HIGH      = GPIO_INTERRUPT_LEVEL | GPIO_HIGH,
      GPIO_INTERRUPT_LEVEL_BOTH      = GPIO_INTERRUPT_LEVEL | GPIO_BOTH,

   GPIO_INTERRUPT_EDGE               = 0x20, // Edge (Rising/falling).
   GPIO_INTERRUPT_AYNCHRONOUS_EDGE   = 0x30, // Asynchronous Edge (Rising/falling).
   GPIO_INTERRUPT_SYNCHRONOUS_EDGE   = 0x40, // Synchronous Edge (Rising/falling). Modify flags:

      GPIO_FALLING                   = 0x01, // default modify flag value
      GPIO_RISING                    = 0x02,

      GPIO_INTERRUPT_EDGE_FALLING    = GPIO_INTERRUPT_EDGE | GPIO_FALLING,
      GPIO_INTERRUPT_EDGE_RISING     = GPIO_INTERRUPT_EDGE | GPIO_RISING,
      GPIO_INTERRUPT_EDGE_BOTH       = GPIO_INTERRUPT_EDGE | GPIO_BOTH

} gpioInterrupt_t;


/* ----- Begin Pin Config Structs NXP LPC4337 ----- */

typedef struct{
   int8_t port;
   int8_t pin;
} gpioConfigLpc4337_t;

typedef struct{
    pinConfigLpc4337_t pinName;
                int8_t func;
   gpioConfigLpc4337_t gpio;
} pinConfigGpioLpc4337_t;

/* ------ End Pin Config Structs NXP LPC4337 ------ */


/* ------ Begin EM4 ------------------------------- */

// Al registes use same pin order:
// Bits   5   4   3   2   1   0
// Pins PE13 PF2 PF1 PC9 XXX PA0

typedef enum{
   EM4_RESET = 0x00, // This indicates an em4 wake-up from RESET
   EM4_PA0   = 0x01, // This bit indicates an em4 wake-up request occurred on pin A0
   EM4_PC9   = 0x04, // This bit indicates an em4 wake-up request occurred on pin C9
   EM4_PF1   = 0x08, // This bit indicates an em4 wake-up request occurred on pin F1
   EM4_PF2   = 0x10, // This bit indicates an em4 wake-up request occurred on pin F2
   EM4_PE13  = 0x20, // This bit indicates an em4 wake-up request occurred on pin E13
   EM4_PC4   = 0x40  // This bit indicates an em4 wake-up request occurred on pin C4
}EM4_pin_t;

/* ------ End EM4 --------------------------------- */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

bool_t gpioConfig( gpioMap_t pin, gpioConfig_t config );
bool_t gpioRead( gpioMap_t pin );
bool_t gpioWrite( gpioMap_t pin, bool_t value );
bool_t gpioToggle( gpioMap_t pin );

// Enable or disable the peripheral energy and clock
bool_t gpioPowerSet( bool_t power );


// Interrupt ------------------------------------

bool_t gpioInterruptCallbackSet( gpioMap_t pin,
                                 gpioInterruptCallback_t interruptCallback );

bool_t gpioInterruptSet( gpioMap_t pin, gpioInterrupt_t interruptMode );

//gpioInterruptCallback_t gpioInterruptCallbackGet( gpioMap_t pin );

//gpioInterrupt_t gpioInterruptGet( gpioMap_t pin );


/* ------ EM4 ------------------------------- */

// Configure/deconfigure pin for Wake-up from EM4
void gpioConfigEM4Wakeup( EM4_pin_t em4Pin, bool_t enable,
                          bool_t wakupCondition );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[example]================================================*/

/*
// GPIOs Test

#define BOARD_PUSH_BUTTON_PB0   PC9
#define BOARD_PUSH_BUTTON_PB1   PC10

#define BOARD_LED0              PF4
#define BOARD_LED1              PF5

gpioConfig( BOARD_PUSH_BUTTON_PB0, GPIO_INPUT );
gpioConfig( BOARD_PUSH_BUTTON_PB1, GPIO_INPUT );

gpioConfig( BOARD_LED0, GPIO_OUTPUT );
gpioConfig( BOARD_LED1, GPIO_OUTPUT );

while(1){
   gpioWrite( BOARD_LED0, !gpioRead( BOARD_PUSH_BUTTON_PB0 ) );
   gpioWrite( BOARD_LED1, !gpioRead( BOARD_PUSH_BUTTON_PB1 ) );
}
*/

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_GPIO_H_ */
