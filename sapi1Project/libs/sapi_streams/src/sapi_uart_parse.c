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

/* Date: 2017-01-07 */

/*==================[inclusions]=============================================*/

#include "sapi_uart.h"
#include "sapi_uart_parse.h"

#include "string.h"
#include "sapi_circularBuffer.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal functions definition]==========================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Check for Receive a given pattern
waitForReceiveStringsOrTimeoutState_t waitForReceiveStringOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance ){

   uint8_t receiveByte;
   //char receiveBuffer[100];

   switch( instance->state ){

      case UART_RECEIVE_STRING_INIT:

         delayConfig( &(instance->delay), instance->timeout );

         instance->stringIndex = 0;

         instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

      case UART_RECEIVE_STRING_RECEIVING:

         if( uartReadByte( uart, &receiveByte ) ){

            //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
/*            if( (instance->stringIndex) <= 100 ){
               receiveBuffer[instance->stringIndex] = receiveByte;
            }
*/
            if( (instance->string)[(instance->stringIndex)] == receiveByte ){

               (instance->stringIndex)++;

               if( (instance->stringIndex) == (instance->stringSize - 1) ){
                  instance->state = UART_RECEIVE_STRING_RECEIVED_OK;

//                  receiveBuffer[instance->stringIndex] = '\0';

                  //uartWriteString( UART_DEBUG, receiveBuffer ); // TODO: DEBUG
                  //uartWriteString( UART_DEBUG, "\r\n" );        // TODO: DEBUG
               }

            }

         }

         if( delayRead( &(instance->delay) ) ){
            instance->state = UART_RECEIVE_STRING_TIMEOUT;
            //uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
         }

      break;

      case UART_RECEIVE_STRING_RECEIVED_OK:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_TIMEOUT:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      default:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;
   }

   return instance->state;
}
/*
// FIXME: Mantener por compatibilidad!!
bool_t waitForReceiveStringOrTimeoutBlocking(
   int32_t uart, char* str, uint16_t strLen, tick_t timeout ){

   bool_t retVal = TRUE; // True if OK
      
   expectString_t expectStringList = { .str = str, .strLen = strLen };
   char receivedStringResult[100];

   waitForReceiveStringsOrTimeout_t waitStrList = {
      .state = UART_RECEIVE_STRING_INIT,
      .mode = UART_RECEIVE_STRING_FIRST, // UART_RECEIVE_STRING_FIRST or UART_RECEIVE_STRING_ALL
      .strList = &expectStringList,
      .strListSize = 1,
      .receiveStr = receivedStringResult,
      .receiveStrMaxLen = 99,
      .timeout = timeout
   };
   waitForReceiveStringsOrTimeoutState_t waitStrListResultState;

   while( waitStrListResultState != UART_RECEIVE_STRING_RECEIVED_OK     &&
          waitStrListResultState != UART_RECEIVE_STRING_RECEIVED_OK_ALL &&
          waitStrListResultState != UART_RECEIVE_STRING_ERROR           &&
          waitStrListResultState != UART_RECEIVE_STRING_TIMEOUT )
   {
      waitStrListResultState = waitForReceiveStringsOrTimeout( UART_USB, &waitStrList );
   }
   
   if( (waitStrListResultState == UART_RECEIVE_STRING_TIMEOUT) || 
       (waitStrListResultState == UART_RECEIVE_STRING_TIMEOUT) )
   {
      retVal = FALSE;
   }
   
   return retVal;
}
*/

// Recibe bytes hasta que llegue el string patron que se le manda en el
// parametro string, stringSize es la cantidad de caracteres del string.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
// No almacena los datos recibidos!! Simplemente espera a recibir cierto patron.
bool_t waitForReceiveStringOrTimeoutBlocking( 
   int32_t uart, 
   char* string, uint16_t stringSize,
   tick_t timeout ){

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringsOrTimeout_t waitText;
   waitForReceiveStringsOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_INIT;

   waitText.state = UART_RECEIVE_STRING_INIT;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ){
      waitTextState = waitForReceiveStringOrTimeout( uart, &waitText );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ){
      retVal = FALSE;
   }

   return retVal;
}


// Recibe bytes hasta que lleguen los string patrones o uno de ellos segun el
// modo
waitForReceiveStringsOrTimeoutState_t waitForReceiveStringsOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance ){

   uint8_t receiveByte;
   uint32_t i=0;
   bool_t allReceived = TRUE;

   switch( instance->state ){

      case UART_RECEIVE_STRING_INIT:

         // Initialize all internal struct variables
         for( i=0; i<(instance->strListSize); i++ ){
            (instance->strList)[i].index = 0;
            (instance->strList)[i].receiveStrStartIndex = -1;
            (instance->strList)[i].complete = FALSE;
         }
         instance->receiveStrLen = 0;
         delayConfig( &(instance->delay), instance->timeout );

         // Change state
         instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

      case UART_RECEIVE_STRING_RECEIVING:
         /*
         Si recibo un byte y la cantidad de recibidos es menor al maximo permitido
            FOR each in List
               Si complete == FALSE
                  Si acierta y encuentra el primer caracter (index = 0), setea receiveStrStartIndex con (receiveStrLen = -1)
                  Si acierta incrementa index (index++), luego de incrementar el index: 
                     Si index == strLen
                        complete = TRUE
                        si está en modo UART_RECEIVE_STRING_FIRST 
                           return TRUE
                  Si falla resetea index (index = 0) y resetea receiveStrStartIndex (receiveStrStartIndex = -1)
         */
      
         if( uartReadByte( uart, &receiveByte ) ){

            // Save byte in receiveStr
            if( ((instance->receiveStrLen) + 1) <= (instance->receiveStrMaxLen) ){
               // if are space available
               (instance->receiveStr)[(instance->receiveStrLen)] = receiveByte; // writeIdx = len - 1 (increment len later)
               (instance->receiveStrLen)++; // increment len
               
               // For each string in list
               for( i=0; i<(instance->strListSize); i++ ){
                  if( (instance->strList)[i].complete == FALSE ){
                     // if char match 
                     if( (instance->strList)[i].str[(instance->strList)[i].index] == receiveByte ){
                        // if match first char in string, set receiveStrStartIndex
                        if( (instance->strList)[i].index == 0 ){
                           (instance->strList)[i].receiveStrStartIndex = (instance->receiveStrLen)-1;                        
                        }
                        // Incremet index
                        (instance->strList)[i].index++;
                        // if index == strLen, complete = TRUE
                        if( (instance->strList)[i].index == (instance->strList)[i].strLen ){
                           (instance->strList)[i].complete = TRUE;
                           // if mode == UART_RECEIVE_STRING_FIRST, change state and return 
                           if( instance->mode == UART_RECEIVE_STRING_FIRST ){
                              // Change state
                              instance->state = UART_RECEIVE_STRING_RECEIVED_OK;
                              // add NULL = "\0" on last char in in receiveStr
                              (instance->receiveStr)[(instance->receiveStrLen)] = '\0';
                              return (instance->state);
                           }
                        }
                     }
                     else{ // if not match, index = 0 and receiveStrStartIndex = -1
                        (instance->strList)[i].index = 0;
                        (instance->strList)[i].receiveStrStartIndex = -1;
                     }
                  }
               }
               
               // if all in list are complete received, change state and return               
               for( i=0; i<(instance->strListSize); i++ ){      
                  allReceived &= (instance->strList)[i].complete;
               }
               if( allReceived ){ 
                  // Change state
                  instance->state = UART_RECEIVE_STRING_RECEIVED_OK_ALL;
                  // add NULL = "\0" on last char in in receiveStr
                  (instance->receiveStr)[(instance->receiveStrLen)] = '\0';
                  return (instance->state);                
               }
               
            } else{
               // if are not space available
               instance->state = UART_RECEIVE_STRING_ERROR; // Me quede sin espacio en receiveStr
               return (instance->state);
            }
         }

         // If timout reached change state and return
         if( delayRead( &(instance->delay) ) ){
            instance->state = UART_RECEIVE_STRING_TIMEOUT;
            return (instance->state);
         }

      break;

      case UART_RECEIVE_STRING_RECEIVED_OK:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_RECEIVED_OK_ALL:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_TIMEOUT:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_ERROR:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      default:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;
   }

   return (instance->state);
}


// Store bytes until receive a given pattern
waitForReceiveStringsOrTimeoutState_t receiveBytesUntilReceiveStringOrTimeout(
   int32_t uart, waitForReceiveStringsOrTimeout_t* instance,
   char* receiveBuffer, uint32_t* receiveBufferSize ){

   uint8_t receiveByte;
   static uint32_t i = 0;
   //uint32_t j = 0;
   //uint32_t savedReceiveBufferSize = *receiveBufferSize;

   switch( instance->state ){

      case UART_RECEIVE_STRING_INIT:

         delayConfig( &(instance->delay), instance->timeout );

         instance->stringIndex = 0;
         i = 0;

         instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

      case UART_RECEIVE_STRING_RECEIVING:

         if( uartReadByte( uart, &receiveByte ) ){

            //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
            if( i < *receiveBufferSize ){
               receiveBuffer[i] = receiveByte;
               i++;
            } else{
               instance->state = UART_RECEIVE_STRING_FULL_BUFFER;
               *receiveBufferSize = i;
               i = 0;
               return instance->state;
            }

            if( (instance->string)[(instance->stringIndex)] == receiveByte ){

               (instance->stringIndex)++;

               if( (instance->stringIndex) == (instance->stringSize - 1) ){
                  instance->state = UART_RECEIVE_STRING_RECEIVED_OK;
                  *receiveBufferSize = i;
                  /*
                  // TODO: For debug purposes
                  for( j=0; j<i; j++ ){
                     uartWriteByte( UART_DEBUG, receiveBuffer[j] );
                  }
                  uartWriteString( UART_DEBUG, "\r\n" );
                  */
                  i = 0;
               }

            }

         }

         if( delayRead( &(instance->delay) ) ){
            instance->state = UART_RECEIVE_STRING_TIMEOUT;
            //uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
            *receiveBufferSize = i;
            i = 0;
         }

      break;

      case UART_RECEIVE_STRING_RECEIVED_OK:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_TIMEOUT:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      case UART_RECEIVE_STRING_FULL_BUFFER:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;

      default:
         instance->state = UART_RECEIVE_STRING_INIT;
      break;
   }

   return instance->state;
}

// Guarda todos los bytes que va recibiendo hasta que llegue el string
// patron que se le manda en el parametro string, stringSize es la cantidad
// de caracteres del string.
// receiveBuffer es donde va almacenando los caracteres recibidos y
// receiveBufferSize es el tamaño de buffer receiveBuffer.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
bool_t receiveBytesUntilReceiveStringOrTimeoutBlocking( 
   int32_t uart, 
   char* string, uint16_t stringSize, 
   char* receiveBuffer, uint32_t* receiveBufferSize,
   tick_t timeout ){

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringsOrTimeout_t waitText;
   waitForReceiveStringsOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_INIT;

   waitText.state = UART_RECEIVE_STRING_INIT;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ){
      waitTextState = receiveBytesUntilReceiveStringOrTimeout(
                         uart, &waitText,
                         receiveBuffer, receiveBufferSize );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ){
      retVal = FALSE;
   }

   return retVal;
}

/*==================[end of file]============================================*/
