/* Copyright 2017, Eric Pernia.
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
 * Date: 2017-07-28
 */

/*==================[inlcusiones]============================================*/

#include "sapi.h"       // <= Biblioteca sAPI
#include "chip.h"       // <= Biblioteca LPCOpen, capa chip (del fabricante NXP)

/*==================[definiciones y macros]==================================*/

// Para configurar cualquier pin hay que saber todos estos datos:
// Los que van en SCU: SCU_PORT, SCU_PIN, SCU_FUNC y los que van en GPIO:
// GPIO_PORT, GPIO_PIN

// Ver en sapi_gpio.c el pin que sAPI llama GPIO0 (por la serigrafía de la
// placa EDU-CIAA) es:

// SCU
#define GPIO0_SCU_PORT   6
#define GPIO0_SCU_PIN    1
#define GPIO0_SCU_FUNC   SCU_MODE_FUNC0

// GPIO
#define GPIO0_GPIO_PORT  3
#define GPIO0_GPIO_PIN   0

// Interrupt
#define PININT_INDEX         0                  // PININT index used for GPIO mapping
#define PININT_IRQ_HANDLER   GPIO0_IRQHandler   // GPIO interrupt IRQ function name
#define PININT_NVIC_NAME     PIN_INT0_IRQn      // GPIO interrupt NVIC interrupt name

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// Handler interrupt from GPIO pin or GPIO pin mapped to PININT
void PININT_IRQ_HANDLER(void)
{
   // Se da aviso que se trato la interrupcion
   Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   
   // Se realiza alguna accion.
   gpioToggle(LEDB);
}

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();   
   
   /* Configuracion de GPIO0 de la EDU-CIAA-NXP como entrada con pull-up */
   gpioConfig( GPIO0, GPIO_INPUT_PULLUP );

   
   // ---> Comienzo de funciones LPCOpen para configurar la interrupcion

   // Configure interrupt channel for the GPIO pin in SysCon block
   Chip_SCU_GPIOIntPinSel( PININT_INDEX, GPIO0_GPIO_PORT, GPIO0_GPIO_PIN );

   // Configure channel interrupt as edge sensitive and falling edge interrupt
   Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );

   // Enable interrupt in the NVIC
   NVIC_ClearPendingIRQ( PININT_NVIC_NAME );
   NVIC_EnableIRQ( PININT_NVIC_NAME );

   // ---> Fin de funciones LPCOpen
   
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {      
      // No hace nada
   } 

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta 
   // directamenteno sobre un microcontroladore y no es llamado/ por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
