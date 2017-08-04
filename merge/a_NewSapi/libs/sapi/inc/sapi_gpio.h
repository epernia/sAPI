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

#ifndef _SAPI_GPIO_H_
#define _SAPI_GPIO_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_soc_map.h"
#include "sapi_peripheral_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/*
#define GPIO_STRENGTH(n)    ( (n) << 4 )
#define GPIO_SPEED(n)       ( (n) << 7 )
*/
/*==================[typedef]================================================*/
  
/* GPIO Config int32_t */
/*

gpioConfig[31:0] = gpioInterrupt_t[24:31], 
                   gpioStrenght_t[16:23], 
                   gpioSpeed_t[8:15],  
                   gpioPull_t[7:4]
                   gpioMode_t[3:0]

               intm   int   stre  speed  pull    mode
  31  28 27  24 23  20 19  16 15  12 11  8   7  4    3  0
   0000   0000   0000   0000   0000   0000   0000    0000
                    0      0      0      0      0 Di    0   DISABLE
                   01     01      1      1      1 Up    1   INPUT
                   10     10     10     10     10 Dn   10   OUTPUT
                                 11     11             10   PUSHPULL
                                100    100             11   OPENDRAIN
                                101    101             11   OPENCOLLECTOR
                                110    110
                                111    111
*/

/* GPIO Properties */  
/*
 All GPIO:
 - Power

 Single pin:
 - Mode
 - Strenght
 - Speed
 - Interrupt (Ver cuales pueden tener)
*/
   
/* GPIO Properties values */   
   
typedef enum{
   // Disable mode
   GPIO_DISABLE           = 0,
   // Input mode   
   GPIO_INPUT             = 1,
   // Output modes   
   GPIO_OUTPUT            = 2,
   GPIO_PUSHPULL          = GPIO_OUTPUT, // default value with GPIO_OUTPUT
   GPIO_OPENDRAIN         = 3,
   GPIO_OPENCOLLECTOR     = GPIO_OPENDRAIN,
} gpioMode_t;
   
typedef enum{
   GPIO_PULL_DISABLE      = (0<<4),      // default value
   GPIO_PULL_UP           = (1<<4),
   GPIO_PULL_DOWN         = (2<<4)
} gpioPull_t;
   
typedef enum{
   // Input Speed modes   
   GPIO_SPEED0 = (0 << 8),   
   GPIO_SPEED1 = (1 << 8),
   GPIO_SPEED2 = (2 << 8),
   GPIO_SPEED3 = (3 << 8),
   GPIO_SPEED4 = (4 << 8),
   GPIO_SPEED5 = (5 << 8),
   GPIO_SPEED6 = (6 << 8),
   GPIO_SPEED7 = (7 << 8)
} gpioSpeed_t;
   
typedef enum{
   // Output Strength   
   GPIO_STRENGTH0 = (0 << 12),   
   GPIO_STRENGTH1 = (1 << 12),
   GPIO_STRENGTH2 = (2 << 12),
   GPIO_STRENGTH3 = (3 << 12),
   GPIO_STRENGTH4 = (4 << 12),
   GPIO_STRENGTH5 = (5 << 12),
   GPIO_STRENGTH6 = (6 << 12),
   GPIO_STRENGTH7 = (7 << 12)
} gpioStrenght_t;
   
typedef enum{
   // Input Interrupt
   GPIO_INTERRUPT_DISABLE      = (0 << 16), // default value
   GPIO_INTERRUPT_LEVEL        = (1 << 16), // Level-sensitive (high/low). Modify flags:
   GPIO_INTERRUPT_EDGE         = (2 << 16), // Edge (Rising/falling).
   
      GPIO_LOW                 = (1 << 20), // default modify flag value
      GPIO_HIGH                = (2 << 20),
      GPIO_BOTH                = (3 << 20),
      GPIO_FALLING             = (1 << 20), // default modify flag value
      GPIO_RISING              = (2 << 20), 
   
   GPIO_INTERRUPT_LEVEL_LOW    = GPIO_INTERRUPT_LEVEL | GPIO_LOW,
   GPIO_INTERRUPT_LEVEL_HIGH   = GPIO_INTERRUPT_LEVEL | GPIO_HIGH,
   GPIO_INTERRUPT_LEVEL_BOTH   = GPIO_INTERRUPT_LEVEL | GPIO_BOTH,
   
   GPIO_INTERRUPT_EDGE_FALLING = GPIO_INTERRUPT_EDGE | GPIO_FALLING,
   GPIO_INTERRUPT_EDGE_RISING  = GPIO_INTERRUPT_EDGE | GPIO_RISING
} gpioInterrupt_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/* --------- Peripheral configutation methods ------------------------------ */

// Initialize
void gpioInit( void );
   
// power. Enable or disable the peripheral energy and clock
void gpioPowerSet( bool_t power );
bool_t gpioPowerGet( void );
   
/* -- Single Pin property getters and setters methods - */
   
// direction
void gpioModeSet( int32_t gpioName, gpioMode_t mode );
gpioMode_t gpioModeGet( int32_t gpioName );
   
// value
void gpioValueSet( int32_t gpioName, bool_t value );
bool_t gpioValueGet( int32_t gpioName );

// speed
void gpioSpeedSet( int32_t gpioName, gpioSpeed_t speed );
gpioSpeed_t gpioSpeedGet( int32_t gpioName );
   
// stength
void gpioStengthSet( int32_t gpioName, gpioStrenght_t speed );
gpioStrenght_t gpioStengthGet( int32_t gpioName );

/* ------- Single Pin multiple property getters and setters methods -------- */

// config  is an uint32_t with "an OR" of direction, value, speed, stength
void gpioConfig( int32_t gpioName, int32_t config );

/* ------------ Interrupt properties methods ----------- */   
   
// inputInterruptCallback
void gpioInterruptCallbackSet( int32_t gpioName,
                               interruptCallback_t interruptCallback );
interruptCallback_t gpioInterruptCallbackGet( int32_t pin );
   
// inputInterrupt
void gpioInterruptSet( int32_t gpioName, gpioInterrupt_t interruptMode );
gpioInterrupt_t gpioInterruptGet( int32_t gpioName );
   
// Se setea la interrupcion vacia por ejemplo para despertar de modo bajo consumo.
// Si esta sin callback no hace nada la isr.
// Algunos módulos como UART podrían hacer algo más en la ISR y luego ejecutar el callback
// de usuario (hook a la ISR)

/* ----------------- POSIX like methods ---------------- */

// Write a GPIO
#define gpioWrite( gpioName, value )    gpioSetValue( (gpioName), (value) )

// Read a GPIO
#define gpioRead( gpioName )            gpioGetValue( (gpioName) )

/* -------------- Especific modes methods -------------- */
   
// Toggle a GPIO output --> Only for output mode
gpioOutputToggle( int32_t gpioName );

// Read a GPIO input with a debounce time --> Only for input mode
gpioInputReadDebounced( int32_t gpioName, tick_t debounceTime );
   
/* ----------------- GPIO PORT method ------------------ */

// Config Group of GPIOs
void gpioGroupModeSet( int32_t* gpios, 
                       uint8_t gpiosSize, 
                       int32_t config );
// definir port "virtuales" hasta 32 pines max.
// pueden ser de 8,16,32 se le pasa un vector de pines.

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[module example]=========================================*/

/*
#include "sapi.h"

#define LED0              GPIO0
#define LED1              GPIO1
#define PUSH_BUTTON_0     GPIO2

int main(void)
{
   boardInit();   
   gpioInit();

   gpioModeSet( PUSH_BUTTON_0, GPIO_INPUT );

   gpioConfig( LED0, GPIO_OUTPUT );
   gpioConfig( LED1, GPIO_OUTPUT );
   
   gpioToggle( LED1 );

   while( TRUE )
   {
      gpioWrite( LED0, !gpioRead( PUSH_BUTTON_0 ) );
   }
   
   return 0;
}
*/

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_GPIO_H_ */


/*
-----------------------------------

Separar el enum de GPIO por tipo de propiedad:
 - Mode
 - Power
 - Strenght
 - Speed
 - Interrupt
 
en gpioConfig() poner de parámetros un pin tipo gpioName y un uint32_t para
los parametros "oreados" que van a ser diferentes enums, esto no debería tirar warnings.

El periph gpio va a tener los módos eléctricos de los pines salvo los
modos que usan perifericos especificos.
*/
