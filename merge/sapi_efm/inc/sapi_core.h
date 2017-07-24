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

/* Date: 2017-06-02 */

#ifndef _SAPI_CORE_H_
#define _SAPI_CORE_H_


// Energy Modes ---------------------------------------------------------------
                                      // EM  | ClkSrc | Freq    | Consumption
// ----- Run mode, EM0 --------------------------------------------------------
//em_EM0_Hfxo();                      // EM0 | HFXO   | 24 MHz  | 3.09 mA
//em_EM0_Hfrco(cmuHFRCOBand_21MHz);   // EM0 | HFRCO  | 21 MHz  | 2.69 mA
//em_EM0_Hfrco(cmuHFRCOBand_14MHz);   // EM0 | HFRCO  | 14 MHz  | 1.82 mA
//em_EM0_Hfrco(cmuHFRCOBand_11MHz);   // EM0 | HFRCO  | 11 MHz  | 1.45 mA
//em_EM0_Hfrco(cmuHFRCOBand_7MHz);    // EM0 | HFRCO  | 6.6 MHz | 907.56 uA
//em_EM0_Hfrco(cmuHFRCOBand_1MHz);    // EM0 | HFRCO  | 1.2 MHz | 199.01 uA
// ----- Sleep mode, EM1 ------------------------------------------------------
//em_EM1_Hfxo();                      // EM1 | HFXO   | 24 MHz  | 1.21 mA
//em_EM1_Hfrco(cmuHFRCOBand_21MHz);   // EM1 | HFRCO  | 21 MHz  | 1.05 mA
//em_EM1_Hfrco(cmuHFRCOBand_14MHz);   // EM1 | HFRCO  | 14 MHz  | 729.79 uA
//em_EM1_Hfrco(cmuHFRCOBand_11MHz);   // EM1 | HFRCO  | 11 MHz  | 594.52 uA
//em_EM1_Hfrco(cmuHFRCOBand_7MHz);    // EM1 | HFRCO  | 6.6 MHz | 391.54 uA
//em_EM1_Hfrco(cmuHFRCOBand_1MHz);    // EM1 | HFRCO  | 1.2 MHz | 109.17 uA
// ----- Deep sleep mode, EM2 -------------------------------------------------
//em_EM2_LfrcoRTC();                  // EM2 | RTC    | 1 Hz    | 1.11 uA
// ----- Stop mode, EM3 -------------------------------------------------------
//em_EM3_Ulfrco();                    // EM3 | ULFRCO |         | 815.54 nA
// ----- Shut off mode, EM4 ---------------------------------------------------
//em_EM4();                           // EM4 |        |         | 145 nA (estimado)
// ----------------------------------------------------------------------------

/*

Sleep Mode (Energy Mode 1)
--------------------------

In Sleep Mode the clock to the CPU is disabled. All peripherals, as well as RAM and Flash are available.
The EFM32 has extensive support for operation in this mode. By using the Peripheral Reflex System
(PRS) and DMA, several operations can be performed autonomously. For example, the timer may
repeatedly trigger an ADC conversion at a given instant. When the conversion is complete, the result
is moved by the DMA to RAM. When a given number of conversions have been performed, the DMA
may wake up the CPU using an interrupt.

Sleep Mode is entered by executing either the "Wait for Interrupt (WFI)" or the "Wait for Event (WFE)"
instruction.

// void EMU_EnterEM1(void);


Deep Sleep Mode (Energy Mode 2)
-------------------------------

In the Deep Sleep Mode no high frequency oscillators are running, i.e. only asynchronous or low
frequency peripherals are available. This mode is extremely energy efficient, and may be used for a
wide range of useful cases. A few examples:

 - The Low Energy Sensor Interface (LESENSE) is monitoring a sensor
 - The LCD controller drives an LCD.
 - The LEUART is receiving or transmitting a byte.
 - The I2C is performing address match check.
 - The RTC is counting and will wake up the CPU after a programmed number of ticks.
 - An Analog Comparator (ACMP) is comparing a voltage to a programmed threshold.
 - GPIO is checking for transitions on an IO line.

Deep Sleep Mode is entered by first setting the SLEEPDEEP bit in the System Control Register (SCR)
and then executing either the "Wait for Interrupt (WFI)" or the "Wait for Event (WFE)" instruction.

// void EMU_EnterEM2(bool restore);


Stop Mode (Energy Mode 3)
-------------------------

Stop Mode differ from Deep Sleep Mode in that no oscillator (except the ULFRCO to the watch dog)
is running.

Modules available in Stop Mode are:

 - I2C Address Check
 - Watchdog
 - Asynchronous Pin Interrupt
 - Analog Comparator (ACMP)
 - Voltage Comparator (VCMP)

Stop Mode is entered the same way as Deep Sleep Mode, except that also the low frequency oscillators
are turned off.

// void EMU_EnterEM3(bool restore);


Shut Off Mode (Energy Mode 4)
-----------------------------

In Shut Off Mode the EFM32 is completely shut down and the current consumption is as low as 20nA.
The only way to exit this energy mode is with a reset.

Shut Off Mode is entered by writing 0x3 and then 0x2 four times to the EM4CTRL field in the EMU_CTRL
register of the Energy Management Unit (EMU).

// void EMU_EnterEM4(void);

*/

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_gpio.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Initialize power modes
void corePowerInit(void);

// Set power mode
bool_t corePowerModeSet( uint8_t mode );

// Returns pin that cause the Wake-up or RESET
EM4_pin_t corePowerWakeupCause( void );


/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_CORE_H_ */
