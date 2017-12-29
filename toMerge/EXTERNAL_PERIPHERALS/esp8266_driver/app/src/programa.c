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
 * Date: 2016-04-26
 */

/* Diagrama de conexion ESP8266

   VCC ESP8266 <--> +3.3V EDU-CIAA-NXP
   RST ESP8266 <--> (SIN CONEXION)
 CH_PD ESP8266 <--> +3.3V EDU-CIAA-NXP
    TX ESP8266 <--> RX EDU-CIAA-NXP

    RX ESP8266 <--> TX EDU-CIAA-NXP
 GPIO0 ESP8266 <--> (SIN CONEXION)
 GPIO0 ESP8266 <--> (SIN CONEXION)
   GND ESP8266 <--> GND EDU-CIAA-NXP

  AT commands: http://www.pridopia.co.uk/pi-doc/ESP8266ATCommandsSet.pdf
*/

/*==================[inclusions]=============================================*/

#include "sapi.h"                 /* <= sAPI header */

#include <string.h>   // <= Biblioteca de manejo de Strings, ver:
// https://es.wikipedia.org/wiki/String.h
// http://www.alciro.org/alciro/Programacion-cpp-Builder_12/funciones-cadenas-caracteres-string.h_448.htm

/*==================[macros and definitions]=================================*/

#define BAUD_RATE 115200 // Baudrate por defecto del ESP8266

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

DEBUG_PRINT_ENABLE

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void imprimirMensajeDeBienvenida( void ){

   /* Imprimo el mensaje de bienvenida */
   uartWriteString( UART_USB, 
      "Bievenido al asistente de configuracion del modulo ESP8266\r\n" );
   uartWriteString( UART_USB, 
      "Antes de continuar, por favor asegurese que su terminal\r\n" );
   uartWriteString( UART_USB, 
      "serie dispone del terminador CR+LF (enter)\r\n\r\n" );
   uartWriteString( UART_USB, 
      "A continuacion se realiza un listado de algunos de los\r\n" );
   uartWriteString( UART_USB, "comandos AT disponibles:\r\n\r\n" );
   uartWriteString( UART_USB, 
      "> Saber si el modulo responde correctamente: AT\r\n" );
   uartWriteString( UART_USB, 
      "> Version del Firmware: AT+GMR\r\n" );
   uartWriteString( UART_USB, "> Resetear el modulo: AT+RST\r\n" );
   uartWriteString( UART_USB, 
      "> Listar todas las redes disponibles: AT+CWLAP\r\n" );
   uartWriteString( UART_USB, 
      "> Checkear la red actual: AT+CWJAP?\r\n" );
   uartWriteString( UART_USB, 
      "> Unirse a una red: AT+CWJAP=\"nombreRedInalambrica\",\"password\"\r\n" );
   uartWriteString( UART_USB, 
      "  - NOTA: Las comillas dobles son parte del mensaje\r\n" );
   uartWriteString( UART_USB, 
      "> Salir de la red: AT+CWQAP\r\n" );

   delay(100);
}


/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializar la placa */
   boardConfig();

   /* Inicializar las UART a 115200 baudios */
   uartConfig( UART_USB, BAUD_RATE );
   uartConfig( UART_232, BAUD_RATE );
   
   
   
   
   // PRUEBA: "saraza pepe pope hola marola pirola chau"

   expectString_t expectStringList[4] = {
      { .str = "mar", .strLen = 3 },
      { .str = "pepe", .strLen = 4 },
      { .str = "hola marola", .strLen = 11 },
      { .str = "chau", .strLen = 4 }
   };
   char receivedStringResult[50];

   waitForReceiveStringsOrTimeout_t waitStrList = {
      .state = UART_RECEIVE_STRING_INIT,
      .mode = UART_RECEIVE_STRING_ALL, // UART_RECEIVE_STRING_FIRST or UART_RECEIVE_STRING_ALL
      .strList = expectStringList,
      .strListSize = 4,
      .receiveStr = receivedStringResult,
      .receiveStrMaxLen = 55,
      .timeout = 10000
   };
   waitForReceiveStringsOrTimeoutState_t waitStrListResultState;

   while( waitStrListResultState != UART_RECEIVE_STRING_RECEIVED_OK     &&
          waitStrListResultState != UART_RECEIVE_STRING_RECEIVED_OK_ALL &&
          waitStrListResultState != UART_RECEIVE_STRING_ERROR           &&
          waitStrListResultState != UART_RECEIVE_STRING_TIMEOUT )
   {
      waitStrListResultState = waitForReceiveStringsOrTimeout( UART_USB, &waitStrList );
   }

   // Print result
   if( (waitStrListResultState == UART_RECEIVE_STRING_RECEIVED_OK) ){
      debugPrintlnString( "UART_RECEIVE_STRING_RECEIVED_OK" );
   }
   if( (waitStrListResultState == UART_RECEIVE_STRING_RECEIVED_OK_ALL) ){
      debugPrintlnString( "UART_RECEIVE_STRING_RECEIVED_OK_ALL" );
   }
   if( (waitStrListResultState == UART_RECEIVE_STRING_ERROR) ){
      debugPrintlnString( "UART_RECEIVE_STRING_ERROR" );
   }
   if( (waitStrListResultState == UART_RECEIVE_STRING_TIMEOUT) ){
      debugPrintlnString( "UART_RECEIVE_STRING_TIMEOUT" );
   }
   
   debugPrintString( "Index of: 'mar': " );
   debugPrintlnInt( expectStringList[0].receiveStrStartIndex );   
   debugPrintString( "Index of: 'pepe': " );
   debugPrintlnInt( expectStringList[1].receiveStrStartIndex );   
   debugPrintString( "Index of: 'hola marola': " );
   debugPrintlnInt( expectStringList[2].receiveStrStartIndex );   
   debugPrintString( "Index of: 'chau': " );
   debugPrintlnInt( expectStringList[3].receiveStrStartIndex );
   
   debugPrintString( "receivedStringResult = " );
   debugPrintlnString( receivedStringResult );
   debugPrintEnter();

   while(1);   
   

   uint8_t rxData = 0;

   char miTexto[] = "OK\r\n";
   char miTexto_Red[] = "+CWJAP:\"miWifi\"\r\n";

   bool_t received = FALSE;


   /* Si presionan TEC1 muestro el mensaje de bienvenida */
/*
   imprimirMensajeDeBienvenida();


   // Analizar respuestas de conexion
   uartWriteString( UART_USB,
      "\r\n\r\nAnalizar respuestas de conexion\r\n\r\n");
   
   uartWriteString( UART_232,
      "AT\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );

   uartWriteString( UART_232,
      "AT+CWMODE?\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );

   uartWriteString( UART_232,
      "AT+CWLAP\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );

   uartWriteString( UART_232,
      "AT+CWJAP?\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );
             
   // Analizar respuestas de envio de datos
   
   uartWriteString( UART_USB,
      "\r\n\r\nAnalizar respuestas de envio de datos:\r\n\r\n");
   
             
   uartWriteString( UART_232,
      "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );
             
   uartWriteString( UART_232,
      "AT+CIPSEND=39\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );
             
   uartWriteString( UART_232,
      "GET /update?key=P4JCKHMIPAI5TZ3C&1=99\r\n");
      
   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "\r\n",
                sizeof("\r\n"),
                1000
             );
   
   
   delay(30000);
*/
   
/*
   uartWriteString( UART_232,
     "AT\r\n");

   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                miTexto,
                sizeof(miTexto),
                1000
             );

   if( received ){
     uartWriteString( UART_USB, "\r\nRecibi OK del ESP\r\n" );
   }
   // Si no lo recibe indica que salio de la funcion
   // waitForReceiveStringOrTimeoutBlocking  por timeout.
   else{
     uartWriteString( UART_USB, "\r\nError de OK\r\n" );
   }

   delay(3000);

   uartWriteString( UART_232,
     "AT+CWJAP?\r\n");

   received = waitForReceiveStringOrTimeoutBlocking(
                UART_232,
                "+CWJAP:\"casaEric\"\r\n",
                sizeof("+CWJAP:\"casaEric\"\r\n"),
                1000
             );

   if( received ){
     uartWriteString( UART_USB, "\r\nEsta conectado a la red que corresponde\r\n" );
   }
   // Si no lo recibe indica que salio de la funcion
   // waitForReceiveStringOrTimeoutBlocking  por timeout.
   else{
     uartWriteString( UART_USB, "\r\nError de conexion de red\r\n" );
   }

   delay(3000);
*/

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(1) {

      /* Si presionan TEC1 muestro el mensaje de bienvenida */
      if( !gpioRead( TEC1 ) ){
         gpioWrite( LEDB, ON );
         imprimirMensajeDeBienvenida();
         gpioWrite( LEDB, OFF );
      }
      
      /* Si presionan TEC2 muestro el mensaje de bienvenida */
      if( !gpioRead( TEC2 ) ){
         gpioWrite( LED1, ON );
         delay(500);
         uartWriteString( UART_USB, ">> Send: AT\r\n" );
         uartWriteString( UART_232, "AT\r\n" );
         gpioWrite( LED1, OFF );
         
         received = waitForReceiveStringOrTimeoutBlocking( UART_232,
                       "OK\r\n", strlen("OK\r\n"), 1000 );
         if( received ){
            uartWriteString( UART_USB, "Recibio OK\r\n" );
         }
         
      }
      
      // Bridge entre UART_USB y UART_232

      // Si recibe un byte de la UART_USB lo guardarlo en la variable rxData.
      if( uartReadByte( UART_USB, &rxData ) ){
         // Se reenvía el dato a la UART_232
         uartWriteByte( UART_232, rxData );
      }

      // Si recibe un byte de la UART_232 lo guardarlo en la variable rxData. 
      if( uartReadByte( UART_232, &rxData ) ){
         // Se reenvía el dato a la UART_USB
         uartWriteByte( UART_USB, rxData );
      }
      
   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/