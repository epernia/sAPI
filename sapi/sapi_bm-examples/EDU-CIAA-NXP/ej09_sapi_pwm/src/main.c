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

/*
 * Date: 2016-07-03
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

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializar la placa */
   boardConfig();

   /* Inicializar el conteo de Ticks con resolución de 1ms, sin tickHook */
   tickConfig( 1, 0 );

   /* Inicializar DigitalIO */
   digitalConfig( 0, ENABLE_DIGITAL_IO );

   /* Configuración de pines de entrada para Teclas de la CIAA-NXP */
   digitalConfig( TEC1, INPUT );
   digitalConfig( TEC2, INPUT );
   digitalConfig( TEC3, INPUT );
   digitalConfig( TEC4, INPUT );

   /* Configuración de pines de salida para Leds de la CIAA-NXP */
   digitalConfig( LEDR, OUTPUT );
   digitalConfig( LEDG, OUTPUT );
   digitalConfig( LEDB, OUTPUT );
   digitalConfig( LED1, OUTPUT );
   digitalConfig( LED2, OUTPUT );
   digitalConfig( LED3, OUTPUT );

   /* Variables de Retardos no bloqueantes */
   delay_t delayBase1;
   delay_t delayBase2;
   delay_t delayBase3;

   /* Inicializar Retardo no bloqueante con tiempo en milisegundos 
      (500ms = 0,5s) */
   delayConfig( &delayBase1, 50 );
   delayConfig( &delayBase2, 200 );
   delayConfig( &delayBase3, 1000 );

   bool_t valor = 0;

   uint8_t  dutyCycle1 = 0; /* 0 a 255 */
   uint16_t dutyCycle2 = 0;
   uint16_t dutyCycle3 = 0;

   uint8_t pwmVal = 0; /* 0 a 255 */

   /* Configurar PWM */
   valor = pwmConfig( 0,    ENABLE_PWM_TIMERS );

   valor = pwmConfig( PWM7, ENABLE_PWM_OUTPUT );
   valor = pwmConfig( PWM8, ENABLE_PWM_OUTPUT );
   valor = pwmConfig( PWM9, ENABLE_PWM_OUTPUT );

   /* Usar PWM */
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
          dutyCycle2 += 25;
      if( dutyCycle2>255 )
         dutyCycle2 = 0;
      pwmWrite( PWM8, (uint8_t)dutyCycle2 );

      if( delayRead(&delayBase3) )
          dutyCycle3 += 50;
      if( dutyCycle3>255 )
         dutyCycle3 = 0;
      pwmWrite( PWM9, (uint8_t)dutyCycle3 );

   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
