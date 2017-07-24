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

#include "sapi_core.h"   // <= own header
#include "sapi_clock.h"  // <= own header

#include "em_system.h"
#include "em_assert.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_device.h"
#include "emodes.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

// Initialize core power management
void corePowerInit(void){
   // Energy modes
   // Use default settings for EM23
   EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
   // Initialize EM23 with default parameters
   EMU_EM23Init(&em23Init);
}

// Set power mode
bool_t corePowerModeSet( uint8_t mode ){
   switch(mode){

      case 0:
         return FALSE;
      break;

      case 1:
         return FALSE;
      break;

      case 2:
         // Enter EM2
         EMU_EnterEM2(true);
      break;

      case 3:
         // Enter EM3
         EMU_EnterEM3(true);
      break;

      case 4:
         // Enter EM4
         EMU_EnterEM4();
      break;

      default:
         return FALSE;
      break;
   }
   return TRUE;
}


// Returns pin that cause the Wake-up or RESET
EM4_pin_t corePowerWakeupCause( void ){

   // Check EM4 Wake-up Cause Register
   EM4_pin_t em4Pin = GPIO->EM4WUCAUSE;

   if( em4Pin == 0 ){
      return EM4_RESET;
   } else{
      // Exit EM4
      // GPIO->EM4WUEN = ~(EM4_WU_PF2_HIGH); // Ver
      GPIO->CMD = 1;   // EM4WUCLR = 1, to clear all previous events
      return em4Pin;
   }
}


/*==================[end of file]============================================*/
