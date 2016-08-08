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
 * Date: 2016-04-26
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

   /* Variable de Retardo no bloqueante */
   delay_t delay;

   /* Inicializar Retardo no bloqueante con tiempo en milisegundos 
      (500ms = 0,5s) */
   delayConfig( &delay, 500 );

   int8_t i = 3;
   uint8_t secuencia = 0;

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(1) {

      if ( !digitalRead( TEC1 ) ){
         secuencia = 0;
      }
      if ( !digitalRead( TEC2 ) ){
         /* Velocidad Rapida */
         delayWrite( &delay, 150 );
      }
      if ( !digitalRead( TEC3 ) ){
         /* Velocidad Lenta */
         delayWrite( &delay, 750 );
      }
      if ( !digitalRead( TEC4 ) ){
         secuencia = 1;
      }

      /* delayRead retorna TRUE cuando se cumple el tiempo de retardo */
      if ( delayRead( &delay ) ){
         if ( !secuencia ){
            i--;
         }
         else{
            i++;
         }
      }

      if ( i == 0 ){
         digitalWrite( LEDB, ON );
         digitalWrite( LED1, OFF );
         digitalWrite( LED2, OFF );
         digitalWrite( LED3, OFF );
      }
      if ( i == 1 ){
         digitalWrite( LEDB, OFF );
         digitalWrite( LED1, ON );
         digitalWrite( LED2, OFF );
         digitalWrite( LED3, OFF );
      }
      if ( i == 2 ){
         digitalWrite( LEDB, OFF );
         digitalWrite( LED1, OFF );
         digitalWrite( LED2, ON );
         digitalWrite( LED3, OFF );
      }
      if ( i == 3 ){
         digitalWrite( LEDB, OFF );
         digitalWrite( LED1, OFF );
         digitalWrite( LED2, OFF );
         digitalWrite( LED3, ON );
      }

      if ( i < 0 ){
         i = 3;
      }
      if ( i > 3 ){
         i = 0;
      }

   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
