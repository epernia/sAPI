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

#ifndef _SAPI_H_
#define _SAPI_H_

/*==================[inclusions]=============================================*/


/*==================[inclusions]=============================================*/

// ---- Base modules ----------------------------------------------------------

#include "sapi_datatypes.h"
//#include "sapi_isr_vector.h"

// ---- SoC (System on Chip) Cores --------------------------------------------

#include "sapi_core.h"               // Use ASM instructions
#include "sapi_cyclesCounter.h"    // Use Debug Registers

// ---- SoC (System on Chip) Peripherals --------------------------------------

#include "sapi_gpio.h"             // Use GPIO peripherals

//#include "sapi_adc.h"              // Use ADC peripherals
//#include "sapi_dac.h"              // Use DAC peripheral

#include "sapi_uart.h"             // Use UART peripherals
//#include "sapi_i2c.h"              // Use I2C peripherals
//#include "sapi_spi.h"              // Use SPI peripherals

//#include "sapi_timer.h"          // Use Timer peripherals
//#include "sapi_rtc.h"              // Use RTC peripheral

// ---- Board -----------------------------------------------------------------

#include "sapi_board.h"            // Use clock peripheral and cyclesCounter
#include "sapi_peripheral_map.h"
#include "sapi_board_map.h"
#include "sapi_soc_map.h"

// ---- Time ------------------------------------------------------------------

#include "sapi_tick.h"              // Use Timer module (Systick peripheral)
#include "sapi_delay.h"             // Use sapi_tick module

// ---- Data types conversions ------------------------------------------------

#include "sapi_convert.h"

// ---- Streams ---------------------------------------------------------------

#include "sapi_stdio.h"                  // Use sapi_uart module

#include "sapi_print.h"                  // Use sapi_uart module
#include "sapi_debugPrint.h"             // Use sapi_print module
#include "sapi_consolePrint.h"           // Use sapi_print module

#include "sapi_uart_parse.h"       // Use sapi_uart module

// ---- Buffers ---------------------------------------------------------------

#include "sapi_circularBuffer.h"

// ---- Software peripherals --------------------------------------------------

//#include "sapi_soft_uart.h"         // Use sapi_gpio and sapi_tick modules
//#include "sapi_soft_i2c.h"          // Use sapi_gpio and sapi_tick modules

// ---- External peripherals --------------------------------------------------

// External peripherals connected to GPIOs
//#include "sapi_rgb.h"                    // Use TIMER peripheral
//#include "sapi_7_segment_display.h"      // Use sapi_gpio and sapi_delay modules
//#include "sapi_lcd.h"                    // Use sapi_gpio peripheral
////#include "sapi_lcd_character_display.h"  // Use sapi_gpio peripheral
//#include "sapi_keypad.h"                 // Use sapi_gpio and sapi_delay modules

// External peripherals connected to Timer outputs
//#include "sapi_pwm.h"                    // Use sapi_sct and sapi_gpio modules
//#include "sapi_servo.h"                  // Use sapi_timer and sapi_gpio modules

// External peripherals connected to UART
//#include "sapi_esp8266.h"                // Use sapi_uart module

// External peripherals connected to I2C
//#include "sapi_hmc5883l.h"               // Use sapi_i2c module

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_H_ */
