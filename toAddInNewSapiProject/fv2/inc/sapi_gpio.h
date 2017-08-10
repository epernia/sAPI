/* Copyright 2015-2016, Eric Pernia.
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

#define GPIO_STRENGTH(n)    ( (n) << 4 )
#define GPIO_SPEED(n)       ( (n) << 7 )

/*==================[typedef]================================================*/

/* GPIO Config enum type */
/*
gpioConfig_t[15:0] bits = power[15], event[14:10], speed[9:7], mode[6:0]

event[14:10] bits = falling[14], rising[13], asynch_edge[12], edge[11], 
                    level[10]

mode[6:0] bits = strength[6:4], open_drain[3], pull_down[2], pull_up[1], 
                 input_output[0]
*/
typedef enum{
   
   // Output modes   
   GPIO_OUTPUT            = 0,
      GPIO_PUSHPULL       = 0, // default value with GPIO_OUTPUT
      GPIO_OPENDRAIN      = (1<<3),
      GPIO_OPENCOLLECTOR  = GPIO_OPENDRAIN,

   // Input modes   
   GPIO_INPUT             = 1,

   // Pull modes   
   GPIO_NOPULL   = 0, // default value with GPIO_INPUT
   GPIO_PULLUP   = (1<<1),
   GPIO_PULLDOWN = (1<<2),
   
   // Output Strength   
   GPIO_STRENGTH0 = (0 << 4),   
   GPIO_STRENGTH1 = (1 << 4),
   GPIO_STRENGTH2 = (2 << 4),
   GPIO_STRENGTH3 = (3 << 4),
   GPIO_STRENGTH4 = (4 << 4),
   GPIO_STRENGTH5 = (5 << 4),
   GPIO_STRENGTH6 = (6 << 4),
   GPIO_STRENGTH7 = (7 << 4),
   
   // Input Speed modes   
   GPIO_SPEED0 = (0 << 7),   
   GPIO_SPEED1 = (1 << 7),
   GPIO_SPEED2 = (2 << 7),
   GPIO_SPEED3 = (3 << 7),
   GPIO_SPEED4 = (4 << 7),
   GPIO_SPEED5 = (5 << 7),
   GPIO_SPEED6 = (6 << 7),
   GPIO_SPEED7 = (7 << 7),
   
   // Input Interrupt
   GPIO_INTERRUPT_DISABLE            = 0x00, // default value
   GPIO_INTERRUPT_LEVEL              = 0x10, // Level-sensitive (high/low). Modify flags:
      GPIO_LOW                       = 0x01, // default modify flag value
      GPIO_HIGH                      = 0x02,
      GPIO_BOTH                      = 0x03,
   GPIO_INTERRUPT_EDGE               = 0x20, // Edge (Rising/falling).
   GPIO_INTERRUPT_AYNCHRONOUS_EDGE   = 0x30, // Asynchronous Edge (Rising/falling).
   GPIO_INTERRUPT_SYNCHRONOUS_EDGE   = 0x40, // Synchronous Edge (Rising/falling). Modify flags:
      GPIO_FALLING                   = 0x01, // default modify flag value
      GPIO_RISING                    = 0x02

} gpioConfig_t;
   
   
/* GPIO instance struct type */
typedef struct{
   gpioConfig_t config;
   interruptCallback_t interruptCallback;
} Gpio_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/* --------- Peripheral configutation methods ---------- */

// Initialize
void gpioInit( void );
   
// power. Enable or disable the peripheral energy, clock
void gpioPowerSet( gpioName_t gpioName, gpioConfig_t power );
gpioConfig_t gpioPowerGet( gpioName_t gpioName );

   
/* ------- Single Pin multiple property getters and setters methods -------- */
   
void gpioConfig( gpioName_t gpioName, gpioConfig_t config );
   
/* -- Single Pin property getters and setters methods - */
   
// direction
void gpioDirSet( gpioName_t gpioName, gpioConfig_t direction );
gpioConfig_t gpioDirGet( gpioName_t gpioName );
   
// value
void gpioValueSet( gpioName_t gpioName, bool_t value );
bool_t gpioValueGet( gpioName_t gpioName );

// speed
void gpioSpeedSet( gpioName_t gpioName, gpioConfig_t speed );
gpioConfig_t gpioSpeedGet( gpioName_t gpioName );
   
// stength
void gpioStengthSet( gpioName_t gpioName, gpioConfig_t speed );
gpioConfig_t gpioStengthGet( gpioName_t gpioName );
   

/* ----------------- POSIX like methods ---------------- */
   
// inputInterruptCallback
void gpioInterruptCallbackSet( gpioName_t gpioName,
                               interruptCallback_t interruptCallback );
interruptCallback_t gpioInterruptCallbackGet( gpioMap_t pin );
   
// inputInterrupt
void gpioInterruptSet( gpioName_t gpioName, gpioConfig_t interruptMode );
gpioConfig_t gpioInterruptGet( gpioName_t gpioName );

/* ----------------- POSIX like methods ---------------- */

// Config a GPIO
#define gpioConfig( gpioName, value )   gpioSetValue( (gpioName), (mode) )

// Write a GPIO
#define gpioWrite( gpioName, value )    gpioSetValue( (gpioName), (value) )

// Read a GPIO
#define gpioRead( gpioName )            gpioGetValue( (gpioName) )


/* ------------------- Action methods ------------------ */

// Config Group of GPIOs
void gpioGroupModeSet( gpioName_t* gpios, 
                       uint8_t gpiosSize, 
                       gpioConfig_t config );

/* -------------- Especific modes methods -------------- */
   
// Toggle a GPIO output --> Only for output mode
gpioOutputToggle( gpioName_t );

// Read a GPIO input with a debounce time --> Only for input mode
gpioInputReadDebounced( gpioName_t gpioName, tick_t debounceTime );


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
