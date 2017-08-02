/* Copyright 2016, Eric Pernia.
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

/* Date: 2016-03-07 */

/*==================[inclusions]=============================================*/

#include "sapi_rtc.h"

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_assert.h"
#include "em_rtc.h"
#include "em_device.h"

/*==================[macros and definitions]=================================*/

#define RTC_FREQ       32768

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

// Save RTC interrupt time
volatile tick_t interruptTime = 0;

bool_t rtc_power = OFF;

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/*
 * @Brief: Configure RTC peripheral.
 * @param  rtc_t rtc: RTC structure
 * @return bool_t true (1) if config it is ok
 *
 * @brief Enables LFACLK and selects LFXO as clock source for RTC
 */
bool_t rtcInit( rtc_t * rtc ){

   bool_t retVal = TRUE;

   // Enable LE domain registers
   CMU_ClockEnable( cmuClock_CORELE, true );

   // Enable LFXO as LFACLK in CMU. This will also start LFXO
   CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFXO );

   // Set a clock divisor of 32 to reduce power consumption.
   CMU_ClockDivSet( cmuClock_RTC, cmuClkDiv_32 );

   // Enable RTC clock
   CMU_ClockEnable( cmuClock_RTC, true );

   // Initialize RTC
   RTC_Init(1);

   // Start Counter
   RTC_Enable(true);

   return retVal;
}


bool_t rtcInterruptCallbackSet( tick_t time ){
   bool_t retVal = TRUE;

   return retVal;
}


// Sets up the RTC to generate an interrupt every "time" seconds
bool_t rtcInterruptSet( tick_t time ){ // time in seconds

   bool_t retVal = TRUE;

   interruptTime = time;

   // Stop Counter
   RTC_Enable(false);

   // Interrupt every "time" seconds
   RTC_CompareSet(0, (RTC_FREQ / 32) * time );

   // Enable interrupt
   NVIC_EnableIRQ( RTC_IRQn );

   RTC_IntEnable( RTC_IEN_COMP0 );

   // Start Counter
   RTC_Enable(true);

   return retVal;
}

/*
 * @Brief: Get time from RTC peripheral.
 * @param  rtc_t rtc: RTC structure
 * @return bool_t true (1) if config it is ok
 */
bool_t rtcRead( rtc_t * rtc ){

   bool_t retVal = TRUE;

   /*
   rtc->sec = rtcTime.time[RTC_TIMETYPE_SECOND];
   rtc->min = rtcTime.time[RTC_TIMETYPE_MINUTE];
   rtc->hour = rtcTime.time[RTC_TIMETYPE_HOUR];
   rtc->wday = rtcTime.time[RTC_TIMETYPE_DAYOFWEEK];
   rtc->mday = rtcTime.time[RTC_TIMETYPE_DAYOFMONTH];
   rtc->month = rtcTime.time[RTC_TIMETYPE_MONTH];
   rtc->year = rtcTime.time[RTC_TIMETYPE_YEAR];
   */

   return retVal;
}

/*
 * @Brief: Set time on RTC peripheral.
 * @param  rtc_t rtc: RTC structure
 * @return bool_t true (1) if config it is ok
 */
bool_t rtcWrite( rtc_t * rtc ){

   bool_t retVal = TRUE;

   /*
   rtcTime.time[RTC_TIMETYPE_SECOND]     = rtc->sec;
   rtcTime.time[RTC_TIMETYPE_MINUTE]     = rtc->min;
   rtcTime.time[RTC_TIMETYPE_HOUR]       = rtc->hour;
   rtcTime.time[RTC_TIMETYPE_DAYOFMONTH] = rtc->wday;
   rtcTime.time[RTC_TIMETYPE_DAYOFMONTH] = rtc->mday;
   rtcTime.time[RTC_TIMETYPE_MONTH]      = rtc->month;
   rtcTime.time[RTC_TIMETYPE_YEAR]	     = rtc->year;
*/

   return retVal;
}


void rtcReset( void ){
   // Reset RTC Counter
   RTC_CounterReset();
}


// Enable or disable the peripheral energy and clock
bool_t rtcPowerSet( bool_t power ){
   if( power ){

      rtc_power = ON;

      // Start Counter
      RTC_Enable(true);

      mag3110ExitStandby();

   } else{

      rtc_power = OFF;

      // Stop Counter
      RTC_Enable(false);
   }
   return TRUE;
}

/*==================[Interrupt]==============================================*/

// @brief RTC Interrupt Handler.
//     Updates minutes and hours.
void RTC_IRQHandler(void){
   // Clear interrupt source
   RTC_IntClear(RTC_IFC_COMP0);

   // Increase time by one minute (if interrupts every minute)
   /*
   minutes++;
   if(minutes > 59){
     minutes = 0;
     hours++;
     if(hours > 23){
       hours = 0;
     }
   }
   */
}


/*==================[end of file]============================================*/
