/*============================================================================
 * Licencia: BSD-3clauses
 * Autor: Eric Pernia
 * Fecha: 7/12/2017
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "sapi.h"        // <= Biblioteca sAPI

/*==================[definiciones y macros]==================================*/

//#define UART_SELECTED   UART_GPIO
//#define UART_SELECTED   UART_485
#define UART_SELECTED   UART_USB
//#define UART_SELECTED   UART_ENET
//#define UART_SELECTED   UART_232

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/


#ifndef UART_DEBUG
   #define UART_DEBUG UART_USB
#endif


// rxBuff, buffer name (strcture)
// 1, each element size in bytes
// 128, amount of elements in buffer
circularBufferNew( uartRxBuff, 1, 128 );

void emptyBuff( void ){
   uartWriteString( UART_DEBUG, "\r\nBuffer vacio.\r\n" );
}

void fullBuff( void ){
   uint8_t txData = 0;
   uartWriteString( UART_DEBUG, "\r\nBuffer lleno. Se imprime el contenido: \r\n" );
   while( circularBufferRead( &uartRxBuff, &txData ) == CIRCULAR_BUFFER_NORMAL ){
		uartWriteByte( UART_DEBUG, txData );
   }
}


void rxIsrHandler( void ){
   uint8_t receivedByte = 0;
   uartReadByte( UART_SELECTED, &receivedByte );
   circularBufferWrite( &uartRxBuff, &receivedByte );
   //uartWriteByte( UART_SELECTED, receivedByte );
}

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   boardConfig(); // Inicializar y configurar la plataforma

   circularBufferConfig( uartRxBuff, 1, 128 );
   circularBufferEmptyBufferCallbackSet( &uartRxBuff, // buffer structure
	                                     emptyBuff );  // pointer to emptyBuffer function
   circularBufferFullBufferCallbackSet( &uartRxBuff,  // buffer structure
		                                  fullBuff );   // pointer to fullBuffer function

   uartConfig( UART_SELECTED, 115200 );
   uartRxInterruptCallbackSet( UART_SELECTED, rxIsrHandler );
   uartRxInterruptSet( UART_SELECTED, ON );
   
   
   // Para habilitar o deshabilitar el evento:
   //uartEvent( UART_USB, UART_RECEIVE_BYTE, ENABLE );

   // y para el callback que atiende el evento
   //uartEventCallbackSet( UART_USB, UART_RECEIVE_BYTE, evtFunPtr &evtParam );
   
   // Para habilitar o deshabilitar el evento:
   //uartEvent( UART_USB, UART_TRANSMISSOR_FREE, ENABLE );
   
   //UART_TRANSMISSOR_FREE
   //UART_TXBUF_EMPTY
   //UART_TX_READY
   //UART_RX_DATA
   //UART_RX_BUFFER
   //UART_RX_BYTE
   //O _RECV_ y _SEND_


   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {
      sleepUntilNextInterrupt();
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
