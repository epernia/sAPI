/* Copyright 2016, Ian Olivieri
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

/* Date: 2016-02-10 */

#ifndef SAPI_TIMER_H_
#define SAPI_TIMER_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros and definitions]=================================*/

/*==================[typedef]================================================*/
typedef void (*voidFunctionPointer_t)(void);


// NEW TIMER --------------------------------------------------------------------------------------------

typedef enum{
   TIMER_TICKER,
   TIMER_OVERFLOW,
   TIMER_MATCH,
   TIMER_MATCH_OUTPUT,
   TIMER_PWM,
   TIMER_INPUT_CAPTURE
} timerMode_t;



typedef enum{
   TIMER_CAPTURE_IN_RISING_EDGE,
   TIMER_CAPTURE_IN_FALLING_EDGE,
   TIMER_CAPTURE_IN_BOTH_EDGES
} timerInputCaptureMode_t;


typedef enum{
   TIMER_SET_OUTPUT_ON_MATCH,      // (Set high on match)
   TIMER_CLEAR_OUTPUT_ON_MATCH,    // (Set low on match)
   TIMER_TOGGLESET_OUTPUT_ON_MATCH // (Toggle on match)
} timerMatchOutputMode_t;





typedef enum{
   TIMER0_MATCH_PIN0,
   TIMER0_MATCH_PIN1,
   TIMER0_MATCH_PIN2,
   TIMER0_MATCH_PIN3
} timer0MatchPins_t;

typedef enum{
   TIMER1_MATCH_PIN0,
   TIMER1_MATCH_PIN1,
   TIMER1_MATCH_PIN2,
   TIMER1_MATCH_PIN3
} timer1MatchPins_t;

typedef enum{
   TIMER2_MATCH_PIN0,
   TIMER2_MATCH_PIN1,
   TIMER2_MATCH_PIN2,
   TIMER2_MATCH_PIN3
} timer2MatchPins_t;

typedef enum{
   TIMER3_MATCH_PIN0,
   TIMER3_MATCH_PIN1,
   TIMER3_MATCH_PIN2,
   TIMER3_MATCH_PIN3
} timer3MatchPins_t;



typedef enum{
   TIMER0_CAPTURE_PIN0,
   TIMER0_CAPTURE_PIN1,
   TIMER0_CAPTURE_PIN2,
   TIMER0_CAPTURE_PIN3
} timer0CapturePins_t;

typedef enum{
   TIMER1_CAPTURE_PIN0,
   TIMER1_CAPTURE_PIN1,
   TIMER1_CAPTURE_PIN2,
   TIMER1_CAPTURE_PIN3
} timer1CapturePins_t;

typedef enum{
   TIMER2_CAPTURE_PIN0,
   TIMER2_CAPTURE_PIN1,
   TIMER2_CAPTURE_PIN2,
   TIMER2_CAPTURE_PIN3
} timer2CapturePins_t;

typedef enum{
   TIMER3_CAPTURE_PIN0,
   TIMER3_CAPTURE_PIN1,
   TIMER3_CAPTURE_PIN2,
   TIMER3_CAPTURE_PIN3
} timer3CapturePins_t;



typedef enum{
   CLOCK_SOURCE_INTERNAL_MCU_CLK,
   CLOCK_SOURCE_EXTERNAL_CLK
} clockSource_t;

typedef enum{
   PRESCALER_1,
   PRESCALER_8,
   PRESCALER_16,
   PRESCALER_32,
   PRESCALER_64,
   PRESCALER_128,
   PRESCALER_256,
   PRESCALER_512,
   PRESCALER_1024
} prescaler_t;



void timerConfig( timerMap_t timer, timerMode_t mode, clockSource_t clkSource, prescaler_t prescaler );

void timerSetInputCaptureEvent( timerMap_t timer, sAPI_FuncPtr_t eventHook );

void timerSetMatchEvent( timerMap_t timer, sAPI_FuncPtr_t eventHook );

// NEW TIMER --------------------------------------------------------------------------------------------

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/*
 * @Brief   Initialize Timer peripheral
 * @param   timerNumber:   Timer number, 0 to 3
 * @param   ticks:   Number of ticks required to finish the cycle.
 * @param   voidFunctionPointer:   function to be executed at the end of the timer cycle
 * @return   nothing
 * @note   For the 'ticks' parameter, see function Timer_microsecondsToTicks
 */
void Timer_Init(uint8_t timerNumber , uint32_t ticks, voidFunctionPointer_t voidFunctionPointer);

/*
 * @Brief   Disables timer peripheral
 * @param   timerNumber:   Timer number, 0 to 3
 * @return   nothing
 */
void Timer_DeInit(uint8_t timerNumber);

/*
 * @Brief   Converts a value in microseconds (uS = 1x10^-6 sec) to ticks
 * @param   uS:   Value in microseconds
 * @return   Equivalent in Ticks for the LPC4337
 * @note   Can be used for the second parameter in the Timer_init
 */
uint32_t Timer_microsecondsToTicks(uint32_t uS);

/*
 * @Brief   Enables a compare match in a timer
 * @param   timerNumber:   Timer number, 0 to 3
 * @param   compareMatchNumber:   Compare match number, 1 to 3
 * @param   ticks:   Number of ticks required to reach the compare match.
 * @param   voidFunctionPointer: function to be executed when the compare match is reached
 * @return   None
 * @note   For the 'ticks' parameter, see function Timer_microsecondsToTicks
 */
void Timer_EnableCompareMatch(uint8_t timerNumber, uint8_t compareMatchNumber , uint32_t ticks, voidFunctionPointer_t voidFunctionPointer);

/*
 * @brief   Disables a compare match of a timer
 * @param   timerNumber:   Timer number, 0 to 3
 * @param   compareMatchNumber:   Compare match number, 1 to 3
 * @return   None
 */
void Timer_DisableCompareMatch(uint8_t timerNumber, uint8_t compareMatchNumber);

/*
 * @Purpose:   Allows the user to change the compare value n? 'compareMatchNumber' of timer 'timerNumber'.
 *    This is specially useful to generate square waves if used in the function for the TIMERCOMPAREMATCH0 (because
 *    that compare match resets the timer counter), which will be passed as a parameter when initializing a timer
 * @note:  The selected time (3rd parameter) must be less than TIMERCOMPAREMATCH0's compareMatchTime_uS
 *   for the compare match to make the interruption
 */
void Timer_SetCompareMatch(uint8_t timerNumber, uint8_t compareMatchNumber,uint32_t ticks);

/*==================[ISR external functions declaration]=====================*/
/*
 * @Brief:   Executes the functions passed by parameter in the Timer_init,
 *   at the chosen frequencies
 */
void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void TIMER3_IRQHandler(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* SAPI_TIMER_H_ */
