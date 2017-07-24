/* Copyright 2016, Ian Olivieri
 * Copyright 2016, Eric Pernia.
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

/* Date: 2016-02-20 */

/*==================[inclusions]=============================================*/

#include "sapi_adc.h"

#include "em_system.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_assert.h"
#include "em_adc.h"
#include "em_gpio.h"

/*==================[macros and definitions]=================================*/

// Thermometer output gradient for EFM32HG322 (EFM32HG322 datasheet page 36)
#define TGRAD_ADCTH_EFM32HG322_MV_C     -1.92f // mV / °C
#define TGRAD_ADCTH_EFM32HG322_ADCC_C   -6.3f  // ADC_Codes / °C

#define TGRAD_ADCTH   TGRAD_ADCTH_EFM32HG322_MV_C

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static bool_t ADC0_power = OFF;

static uint32_t adcRefVoltage = 3300; // ADC Reference voltage in millivolts //5000; //1250;
static uint32_t adcCounts = 4096; // ADC 2^(number of conversion bits)

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/*
 * @brief:  enable/disable the ADC and DAC peripheral
 * @param:  ADC_ENABLE, ADC_DISABLE
 * @return: none
*/
void adcConfig( adcConfig_t config ){ // adcConfig( ADC_ENABLE );

   switch(config){
      // Enable ADC0 peripheral
      case ADC_ENABLE: {

         adcPowerSet( ON );

         // Config ADC0 sample mode
         ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

         /*
         init.ovsRateSel = adcOvsRateSel2;
         init.lpfMode = adcLPFilterBypass;
         init.warmUpMode = adcWarmupNormal;
         init.tailgate = 0;
         */

         init.timebase = ADC_TimebaseCalc(0);
         init.prescale = ADC_PrescaleCalc(400000, 0); // init.prescale = ADC_PrescaleCalc(7000000, 0);

         ADC_Init(ADC0, &init);
      }
      break;

      // Disable ADC0 peripheral
      case ADC_DISABLE:
         adcPowerSet( OFF );
      break;
   }
}


/*
 * @brief   Get the value of one ADC channel. Mode: BLOCKING
 * @param   AI0 ... AIn
 * @return  analog value
 */
uint16_t adcRead( adcMap_t analogInput ){ // adcRead( CH4 );

   uint32_t analogValue = 0;

   // Set conversion Channel

   // ADC_InitSingle
   ADC_InitSingle_TypeDef initsingle = ADC_INITSINGLE_DEFAULT;

   initsingle.prsSel = adcPRSSELCh0;
   initsingle.acqTime = adcAcqTime1;

   // Disable GPIO mode from pin
   switch(analogInput){
      case CH4:
         /* Pin PD4 is Disabled */
         GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                            | GPIO_P_MODEL_MODE4_DISABLED;
      break;
      case CH5:
         /* Pin PD5 is Disabled */
         GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                            | GPIO_P_MODEL_MODE5_DISABLED;
      break;
      case CH6:
         /* Pin PD6 is Disabled */
         GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                            | GPIO_P_MODEL_MODE6_DISABLED;
      break;
      case CH7:
         /* Pin PD7 is Disabled */
         GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                            | GPIO_P_MODEL_MODE7_DISABLED;
      break;
      case ADC_TEMP:
         /*
          * 23.3.4.2 Temperature Measurement (EFM32HG-RM page 487)
          *
          * The ADC includes an internal temperature sensor. This sensor is
          * characterized during production and the temperature readout from
          * the ADC at production temperature, ADC0_TEMP_0_READ_1V25, is given
          * in the Device Information (DI) page. The production temperature,
          * CAL_TEMP_0, is also given in this page. The temperature gradient,
          * TGRAD_ADCTH (mV/degree Celsius), for the sensor is found in the
          * datasheet for the devices. By selecting 1.25 V internal reference
          * and measuring the internal temperature sensor with 12 bit
          * resolution, the temperature can be calculated according to the
          * following formula:
          *
          * tempCelsius = CAL_TEMP_0 - ( ADC0_TEMP_0_READ_1V25 - ADC_result ) * Vref / (4096 * TGRAD_ADCTH)
          *
          */
      break;
      default:
         return 0;
      break;
   }

   if( analogInput == ADC_TEMP){
      initsingle.reference = adcRef1V25;  // Temperature measure at 1.25V Vref
      adcRefVoltage = 1250;
   } else{
      initsingle.reference = adcRefVDD;   // ADC CHannels Vref at VDD = 3.3V
      adcRefVoltage = 3300;
   }

   // Set conversion Channel (continue)
   initsingle.resolution = adcRes12Bit;
   initsingle.input = analogInput;        // adcSingleInpCh4;
   initsingle.diff = 0;
   initsingle.prsEnable = 0;
   initsingle.leftAdjust = 0;
   initsingle.rep = 0;

   // Initialize a single sample conversion on selected channel.
   ADC_InitSingle( ADC0, &initsingle );

   // Initialize a single sample conversion
   ADC_Start( ADC0, adcStartSingle );

   // Wait until end of conversion
   while( ADC0->STATUS & ADC_STATUS_SINGLEACT ); // Antes
   //while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ); // En ejemplo Temp ADC

   // Get ADC result
   analogValue = ADC_DataSingleGet( ADC0 );

   return (uint16_t)analogValue;
}



uint32_t adcReadMillivolts( adcMap_t analogInput ){

   uint32_t millivolts = 0;

   uint16_t analogValue = adcRead( analogInput );

   millivolts = ( ((uint32_t)analogValue) * adcRefVoltage ) / adcCounts;

   return millivolts;
}



/*
 * 23.3.4.2 Temperature Measurement (EFM32HG-RM page 487)
 *
 * The ADC includes an internal temperature sensor. This sensor is
 * characterized during production and the temperature readout from the ADC at
 * production temperature, ADC0_TEMP_0_READ_1V25, is given in the Device
 * Information (DI) page. The production temperature, CAL_TEMP_0, is also given
 * in this page. The temperature gradient, TGRAD_ADCTH (mV/degree Celsius), for
 * the sensor is found in the datasheet for the devices. By selecting 1.25 V
 * internal reference and measuring the internal temperature sensor with 12 bit
 * resolution, the temperature can be calculated according to the following
 * formula:
 *
 * tempCelsius = CAL_TEMP_0 - ( ADC0_TEMP_0_READ_1V25 - ADC_result ) * Vref / (4096 * TGRAD_ADCTH)
 *
 */
float adcReadTemperature( void ){

   float tempCelsius = -273.0; // cero abasoluto
   float VRef = ((float)adcRefVoltage) / 1000.0;

   // CAL_TEMP_0: Factory calibration temperature from device information page.
   float calTemp0 = (float)( (DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                                     >> _DEVINFO_CAL_TEMP_SHIFT );

   //ADC0_TEMP_0_READ_1V25;
   float adc0Temp0Read1V25 = (float)( (DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT );

   float adcResult = (float)((int32_t)adcRead( ADC_TEMP ));

   tempCelsius = calTemp0 - ( adc0Temp0Read1V25 - adcResult ) * VRef / ( (float)adcCounts * TGRAD_ADCTH);

   tempCelsius = (adc0Temp0Read1V25 - adcResult) * VRef;
   tempCelsius = tempCelsius / ( (float)adcCounts * TGRAD_ADCTH );
   tempCelsius = calTemp0 - tempCelsius;

   // TODO: Ver http://community.silabs.com/t5/32-bit-MCU-Knowledge-Base/EFR32-ADC-Internal-Temperature-Sensor/ta-p/185989

   return tempCelsius;
}


// Enable or disable the peripheral energy and clock
bool_t adcPowerSet( bool_t power ){
   // Valid only for ADC0
   if( power ){
      // Enable clock for ADC0
      CMU_ClockEnable( cmuClock_ADC0, true );

      // Enable ADC0

      ADC0_power = ON;

   } else{
      // Disable ADC0
      // Make sure scan sequence or single measure is not in progress
      ADC0->CMD = ADC_CMD_SCANSTOP | ADC_CMD_SINGLESTOP; // TODO: To be tested

      // Disable clock for ADC0
      CMU_ClockEnable( cmuClock_ADC0, false );

      ADC0_power = OFF;
   }
   return TRUE;
}


// TODO: Ver sensor de temperatura interno incluido en el EFM32HG (Ref manual, pag 487)

/*==================[end of file]============================================*/
