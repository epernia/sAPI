/*==============================================================================
* Copyright (c) 2017, Ariel Berardi
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
*
* Library based on "arun singh" code
* http://controllerstech.blogspot.com.ar/2017/07/i2c-lcd-in-stm32.html 
====================[2017-12-02]==============================================*/

/*==================[inclusions]==============================================*/

#include "sapi.h"    
#include "sapi_displayi2c.h"

/*==================[macros and definitions]==================================*/

// Caracteristicas del modulo LCD
#define LCD_ADDRESS     0x3F  // Address del modulo
#define LCD_COLS        16    // Numero de columnas o caracteres por linea
#define LCD_ROWS        2     // Numero de filas

/*==================[internal data declaration]===============================*/

CONSOLE_PRINT_ENABLE

/*==================[internal functions declaration]==========================*/
/*==================[internal data definition]================================*/

// Display object
displayi2c_t displayi2c;


/*==================[external data definition]================================*/
/*==================[main function definition]================================*/


// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------

   // Inicializar y configurar la plataforma
   boardConfig();
   
   // Inicializo port i2c
   
   // Inicializa el LCD
   displayi2cInit(&displayi2c, 0x3F, 16, 2); 
   displayi2cPrint(&displayi2c, "HOLA MUNDO!!! TEXTO LARGO!");

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {
      if(!gpioRead(TEC1)){
         displayi2cBacklight(&displayi2c, TRUE); 
      }
      
      if(!gpioRead(TEC2)){
         displayi2cBacklight(&displayi2c, FALSE);
      }
      
      if(!gpioRead(TEC3)){
         displayi2cClean(&displayi2c);
      }
      
      if(!gpioRead(TEC4)){
         displayi2cReturnHome(&displayi2c);
         displayi2cPrint(&displayi2c, "NUEVO TEXTO!");
      }
      
      delay(250);
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[internal functions definition]===========================*/
/*==================[external functions definition]===========================*/
/*==================[end of file]=============================================*/