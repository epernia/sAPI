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
 * Date:2016-7-03
 */

/*==================[inclusions]=============================================*/

#include "main.h"     /* <= own header */

#include "sAPI.h"     /* <= sAPI header */

/*==================[macros and definitions]=================================*/

#define BAUD_RATE 115200

/*==================[internal data declaration]==============================*/

/* Buffer */
static uint8_t uartBuff[5];

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t myTickHook(void *ptr){

   digitalWrite( LEDG, 1 );

   return 1;
}

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

/* Show date and time in format "DD/MM/YYYY, HH:MM:SS" */
void showDateAndTime( RTC_t * rtc ){
   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->mday), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio el dia */
   if( (rtc->mday)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   uartWriteByte( UART_USB, '/' );

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->month), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio el mes */
   if( (rtc->month)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   uartWriteByte( UART_USB, '/' );

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->year), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio el año */
   if( (rtc->year)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   
   
   uartWriteString( UART_USB, (uint8_t*) ", ");
   

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->hour), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio la hora */
   if( (rtc->hour)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   uartWriteByte( UART_USB, ':' );

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->min), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio los minutos */
  // uartBuff[2] = 0;    /* NULL */
   if( (rtc->min)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   uartWriteByte( UART_USB, ':' );

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->sec), (char*)uartBuff, 10 ); /* 10 significa decimal */
   /* Envio los segundos */
   if( (rtc->sec)<10 ) 
      uartWriteByte( UART_USB, '0' );
   uartWriteString( UART_USB, uartBuff );
   
   
   /* Envio un 'enter' */
   uartWriteString( UART_USB, (uint8_t*) "\r\n");
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
int main(void){
   
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

   /* Inicializar Uart */
   uartConfig(UART_USB, BAUD_RATE);
   
   RTC_t rtc;
   
   rtc.year = 2016;
   rtc.month = 7;
   rtc.mday = 3;
   rtc.wday = 1;
   rtc.hour = 13;
   rtc.min = 17;
   rtc.sec= 0;

   bool_t val = 0;
   
   /* Initialize RTC */
   val = rtcConfig( &rtc );
   
   uint8_t i = 0;

   delay_t delay1s;
   delayConfig( &delay1s, 1000 );
   
   delay(2000);
   
   for( i=0; i<10; i++ ){
      /* Get time */
      val = rtcRead( &rtc );
      /* Show date and time in format "DD/MM/YYYY, HH:MM:SS" */
      showDateAndTime( &rtc );
      delay(1000);
   }
   
   rtc.year = 2016;
   rtc.month = 7;
   rtc.mday = 3;
   rtc.wday = 1;
   rtc.hour = 14;
   rtc.min = 30;
   rtc.sec= 0;
   
   /* Set time */
   val = rtcWrite( &rtc );

   /* ------------- REPETIR POR SIEMPRE ------------- */
	while(1) {

      if( delayRead(&delay1s) ){
         /* Get time */
         val = rtcRead( &rtc );
         /* Show date and time in format "DD/MM/YYYY, HH:MM:SS" */
         showDateAndTime( &rtc );
      }
      //delay(1000);
   }

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este
	   programa no es llamado por ningun S.O. */
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
