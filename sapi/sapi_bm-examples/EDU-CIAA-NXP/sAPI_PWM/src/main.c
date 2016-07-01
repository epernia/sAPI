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

/** \brief Main example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Main example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * ENP          Eric Pernia
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 2016-04-26   v0.0.1   First version
 */

/*==================[inclusions]=============================================*/

#include "main.h"         /* <= own header */

#include "sAPI.h"         /* <= sAPI header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t myTickHook(void *ptr){

   digitalWrite( LEDG, 1 );

   return 1;
}

/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

 /* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void)
{
   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializar la placa */
   boardConfig();

   /* Inicializar el conteo de Ticks con resolución de 1ms, sin tickHook */
   tickConfig( 1, 0 );
   /* Inicializar el conteo de Ticks con resolución de 1ms, con tickHook */
   //tickConfig( 1, myTickHook );

   /* Inicializar DigitalIO */
   digitalConfig( 0, ENABLE_DIGITAL_IO );

   /* Configuración de pines de entrada para
	   Teclas de la CIAA-NXP */
   digitalConfig( TEC1, INPUT );
   digitalConfig( TEC2, INPUT );
   digitalConfig( TEC3, INPUT );
   digitalConfig( TEC4, INPUT );

   /* Configuración de pines de salida para
	   Leds de la CIAA-NXP */
   digitalConfig( LEDR, OUTPUT );
   digitalConfig( LEDG, OUTPUT );
   digitalConfig( LEDB, OUTPUT );
   digitalConfig( LED1, OUTPUT );
   digitalConfig( LED2, OUTPUT );
   digitalConfig( LED3, OUTPUT );

   /* Variable de Retardo no bloqueante */
   delay_t delayBase1;
   delay_t delayBase2;
   delay_t delayBase3;

   /* Inicializar Retardo no bloqueante con tiempo en ms
	   500 ms = 0,5 s */
   delayConfig( &delayBase1, 100 );
   delayConfig( &delayBase2, 200 );
   delayConfig( &delayBase3, 1000 );

	//uint8_t i = 0;

	bool_t valor = 0;

   uint8_t dutyCycle1 = 0; /* 0 a 255 */
   uint8_t dutyCycle2 = 0;
   uint8_t dutyCycle3 = 0;

   uint8_t pwmVal = 0; /* 0 a 255 */

   /* Config PWM */
   valor = pwmConfig( 0,     PWM_TIMERS_ENABLE );

   valor = pwmConfig( PWM7,  PWM_OUTPUT_ENABLE );
   valor = pwmConfig( PWM8,  PWM_OUTPUT_ENABLE );
   valor = pwmConfig( PWM9, PWM_OUTPUT_ENABLE );


   /* Use PWM */
   valor = pwmWrite( PWM7, dutyCycle1 );
   pwmVal = pwmRead( PWM7 );


   /* ------------- REPETIR POR SIEMPRE ------------- */
	while(1) {

      valor = !digitalRead( TEC1 );
		digitalWrite( LEDB, valor );

      if( delayRead(&delayBase1) )
         dutyCycle1++;
      if( dutyCycle1>255 )
         dutyCycle1 = 0;
      pwmWrite( PWM7, dutyCycle1 );

      if( delayRead(&delayBase2) )
         dutyCycle2++;
      if( dutyCycle2>255 )
         dutyCycle2 = 0;
      pwmWrite( PWM8, dutyCycle2 );

      if( delayRead(&delayBase3) )
          dutyCycle3 += 25;
      if( dutyCycle3>255 )
         dutyCycle3 = 0;
      pwmWrite( PWM9, dutyCycle3 );


   }

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este
	   programa no es llamado por ningun S.O. */
	return 0 ;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
