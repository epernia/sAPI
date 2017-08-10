/* Copyright 2017, Eric Pernia.
 * All rights reserved.
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

/* Date: 2017-06-02 */

/*==================[inclusions]=============================================*/

#include "sapi_clock.h"   // <= own header

// emlib includes
#include "em_system.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
//#include "em_assert.h"
//#include "em_gpio.h"
//#include "em_i2c.h"
//#include "em_leuart.h"
//#include "em_usart.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void startDelay( uint32_t iterations );

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void startDelay( uint32_t iterations ){
   volatile uint32_t i=0;
   for( i=0; i<iterations; i++ );
}


/****************************************************************/
// FROM emodes.c
/****************************************************************/

/**************************************************************************//**
 * @brief   Disable high frequency clocks
 *****************************************************************************/
static void disableHFClocks(void)
{
 // Disable High Frequency Peripheral Clocks
  CMU_ClockEnable(cmuClock_HFPER, false);
#if defined( CMU_HFPERCLKEN0_USART0 )
  CMU_ClockEnable(cmuClock_USART0, false);
#endif
  CMU_ClockEnable(cmuClock_USART1, false);
  CMU_ClockEnable(cmuClock_TIMER0, false);
  CMU_ClockEnable(cmuClock_TIMER1, false);
#if defined( CMU_HFPERCLKEN0_TIMER2 )
  CMU_ClockEnable(cmuClock_TIMER2, false);
#endif
  CMU_ClockEnable(cmuClock_ACMP0, false);
  CMU_ClockEnable(cmuClock_PRS, false);
  CMU_ClockEnable(cmuClock_IDAC0, false);
  CMU_ClockEnable(cmuClock_GPIO, false);
  CMU_ClockEnable(cmuClock_VCMP, false);
  CMU_ClockEnable(cmuClock_ADC0, false);
  CMU_ClockEnable(cmuClock_I2C0, false);

  // Disable High Frequency Core/Bus Clocks
  CMU_ClockEnable(cmuClock_AES, false);
  CMU_ClockEnable(cmuClock_DMA, false);
  CMU_ClockEnable(cmuClock_HFLE, false);
#if defined( CMU_HFCORECLKEN0_USB )
  CMU_ClockEnable(cmuClock_USB, false);
#endif
#if defined( CMU_HFCORECLKEN0_USBC )
  // Disable USBC clock by choosing unused oscillator (LFXO)
  CMU_ClockEnable(cmuClock_USBC, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  CMU_ClockSelectSet(cmuClock_USBC, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_USBC, false);
#endif
}

/**************************************************************************//**
 * @brief   Disable low frequency clocks
 *****************************************************************************/
static void disableLFClocks(void)
{
    // Enable LFXO for Low Frequency Clock Disables
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  // Disable Low Frequency A Peripheral Clocks
  // Note: LFA clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_RTC, false);
    CMU_ClockEnable(cmuClock_PCNT0, false);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_Disabled);

  // Disable Low Frequency B Peripheral Clocks
  // Note: LFB clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_LEUART0, false);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

#if defined( _CMU_LFCCLKEN0_MASK )
  // Disable Low Frequency C Peripheral Clocks
  // Note: LFC clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_USBLE, false);
    CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_Disabled);
#endif

    // Disable Low Frequency Oscillator
    CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
}

/****************************************************************/

/*==================[external functions definition]==========================*/

// Set up and initialize system clock
bool_t clockInit( uint32_t frequencyHz ){

   // LFXO enable

   // HFXO enable

   // LFACLK Setup

   if( frequencyHz == 14000000 ){




      //------------ LOW FREQUENCY CLOCK ----------------------

      /***************************************************************************//**
       * @brief
       *   Enable/disable oscillator.
       *
       * @note
       *   WARNING: When this function is called to disable either cmuOsc_LFXO or
       *   cmuOsc_HFXO the LFXOMODE or HFXOMODE fields of the CMU_CTRL register
       *   are reset to the reset value. I.e. if external clock sources are selected
       *   in either LFXOMODE or HFXOMODE fields, the configuration will be cleared
       *   and needs to be reconfigured if needed later.
       *
       * @param[in] osc
       *   The oscillator to enable/disable.
       *
       * @param[in] enable
       *   @li true - enable specified oscillator.
       *   @li false - disable specified oscillator.
       *
       * @param[in] wait
       *   Only used if @p enable is true.
       *   @li true - wait for oscillator start-up time to timeout before returning.
       *   @li false - do not wait for oscillator start-up time to timeout before
       *     returning.
       ******************************************************************************/
      /*
      // Oscillator types.
      typedef enum
      {
        cmuOsc_LFXO,     // Low frequency crystal oscillator.
        cmuOsc_LFRCO,    // Low frequency RC oscillator.
        cmuOsc_HFXO,     // High frequency crystal oscillator.
        cmuOsc_HFRCO,    // High frequency RC oscillator.
        cmuOsc_AUXHFRCO, // Auxiliary high frequency RC oscillator.
      #if defined( _CMU_STATUS_USHFRCOENS_MASK )
        cmuOsc_USHFRCO,  // USB high frequency RC oscillator
      #endif
      #if defined( CMU_LFCLKSEL_LFAE_ULFRCO ) || defined( CMU_LFACLKSEL_LFA_ULFRCO )
        cmuOsc_ULFRCO    // Ultra low frequency RC oscillator.
      #endif
      } CMU_Osc_TypeDef;
       */
      CMU_OscillatorEnable( cmuOsc_HFRCO, true, true );


      /***************************************************************************//**
       * @brief
       *   Select reference clock/oscillator used for a clock branch.
       *
       * @details
       *   Notice that if a selected reference is not enabled prior to selecting its
       *   use, it will be enabled, and this function will wait for the selected
       *   oscillator to be stable. It will however NOT be disabled if another
       *   reference clock is selected later.
       *
       *   This feature is particularly important if selecting a new reference
       *   clock for the clock branch clocking the core, otherwise the system
       *   may halt.
       *
       * @param[in] clock
       *   Clock branch to select reference clock for. One of:
       *   @li #cmuClock_HF
       *   @li #cmuClock_LFA
       *   @li #cmuClock_LFB @if _CMU_LFCLKSEL_LFAE_ULFRCO
       *   @li #cmuClock_LFC
       *   @endif            @if _SILICON_LABS_32B_PLATFORM_2
       *   @li #cmuClock_LFE
       *   @endif
       *   @li #cmuClock_DBG @if DOXYDOC_USB_PRESENT
       *   @li #cmuClock_USBC
       *   @endif
       *
       * @param[in] ref
       *   Reference selected for clocking, please refer to reference manual for
       *   for details on which reference is available for a specific clock branch.
       *   @li #cmuSelect_HFRCO
       *   @li #cmuSelect_LFRCO
       *   @li #cmuSelect_HFXO
       *   @li #cmuSelect_LFXO
       *   @li #cmuSelect_HFCLKLE
       *   @li #cmuSelect_AUXHFRCO
       *   @li #cmuSelect_HFCLK @ifnot DOXYDOC_EFM32_GECKO_FAMILY
       *   @li #cmuSelect_ULFRCO
       *   @endif
       ******************************************************************************/
      // High Frequency Clock select
      // Using HFRCO at 14MHz as high frequency clock, HFCLK
      CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );


      /***************************************************************************//**
       * @brief
       *   Enable/disable a clock.
       *
       * @details
       *   In general, module clocking is disabled after a reset. If a module
       *   clock is disabled, the registers of that module are not accessible and
       *   reading from such registers may return undefined values. Writing to
       *   registers of clock disabled modules have no effect. One should normally
       *   avoid accessing module registers of a module with a disabled clock.
       *
       * @note
       *   If enabling/disabling a LF clock, synchronization into the low frequency
       *   domain is required. If the same register is modified before a previous
       *   update has completed, this function will stall until the previous
       *   synchronization has completed. Please refer to CMU_FreezeEnable() for
       *   a suggestion on how to reduce stalling time in some use cases.
       *
       * @param[in] clock
       *   The clock to enable/disable. Notice that not all defined clock
       *   points have separate enable/disable control, please refer to CMU overview
       *   in reference manual.
       *
       * @param[in] enable
       *   @li true - enable specified clock.
       *   @li false - disable specified clock.
       ******************************************************************************/
      // Enable peripheral clock
      CMU_ClockEnable( cmuClock_HFPER, true );



      /*
      1 MHz
      7 MHz
      11 MHz
      14 MHz (Default)
      21 MHz
      28 MHz
      */
//      CMU_HFRCOBandSet( cmuHFRCOBand_14MHz );

      /*
      1 MHz
      7 MHz
      11 MHz
      14 MHz (Default)
      21 MHz
      28 MHz
      */
//      CMU_AUXHFRCOBandSet( cmuAUXHFRCOBand_14MHz );

      /*
      24 MHz (Default frequency for device without USB function)
      48 MHz (Default frequency for device with USB function)
      */
//      CMU_USHFRCOBandSet( cmuUSHFRCOBand_48MHz );


      /***************************************************************************//**
       * @brief
       *   Set clock divisor/prescaler.
       *
       * @note
       *   If setting a LF clock prescaler, synchronization into the low frequency
       *   domain is required. If the same register is modified before a previous
       *   update has completed, this function will stall until the previous
       *   synchronization has completed. Please refer to CMU_FreezeEnable() for
       *   a suggestion on how to reduce stalling time in some use cases.
       *
       * @param[in] clock
       *   Clock point to set divisor/prescaler for. Notice that not all clock points
       *   have a divisor/prescaler, please refer to CMU overview in the reference
       *   manual.
       *
       * @param[in] div
       *   The clock divisor to use (<= cmuClkDiv_512).
       ******************************************************************************/
      // When using this function some attention should be made to both parameters.
      // Not all clocks have a prescaler and the maximum prescaling value is also
      // not the same for the different clocks
      //HFPERCLK divide by 1:
//      CMU_ClockDivSet( cmuClock_HFPER, cmuClkDiv_1 );



      //------------ LOW FREQUENCY CLOCK ----------------------


      // LF clock tree setup - Enable LF clocks
      CMU_ClockEnable( cmuClock_HFLE, true );

      //
      CMU_ClockSelectSet( cmuClock_LFB, cmuSelect_LFRCO );
   }

   startDelay(200); // FIXME: ver por que tarda en estabilizar

   // Read clock settings and update SystemCoreClock variable
   SystemCoreClockUpdate();

   // Peripheral Clock enables on each individual module

   return TRUE;
}


// Enable or disable general clock
bool_t clockPowerSet( clockSource_t clock, bool_t power ){

   switch(clock){

      case CLOCK_HIGH_FREQ:
         if( power ){

         } else{
            // Disable High Frequency Clocks
            disableHFClocks();
         }
      break;

      case CLOCK_LOW_FREQ:
         if( power ){
            // LF clock tree setup - Enable LF clocks
            CMU_ClockEnable(cmuClock_CORELE, true);
         } else{
            // Disable Low Frequency Clocks
            disableLFClocks();
         }
      break;

      case CLOCK_PERIPHERAL:
         if( power ){
            // Enable peripheral clock
            CMU_ClockEnable(cmuClock_HFPER, true);
         } else{
            // Disable peripheral clock
            CMU_ClockEnable(cmuClock_HFPER, false);
         }
      break;

      default:
      break;
   }
   return TRUE;
}

/*==================[end of file]============================================*/
