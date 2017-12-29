/*============================================================================
 * Licencia:
 * Autor:
 * Fecha:
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

//#include "program.h"   // <= su propio archivo de cabecera (opcional)
#include "sapi.h"        // <= Biblioteca sAPI

//#include "c_i18n_es.h" // <= para traducir el codigo C al espaï¿½ol (opcional)
//#include "c_i18n_es.h" // <= para traducir la sAPI al espaï¿½ol (opcional)

/*==================[definiciones y macros]==================================*/

//#define UART_SELECTED   UART_GPIO
//#define UART_SELECTED   UART_485
#define UART_SELECTED   UART_USB
//#define UART_SELECTED   UART_ENET
//#define UART_SELECTED   UART_232

/*==================[definiciones de datos internos]=========================*/

CONSOLE_PRINT_ENABLE

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/


// Caracter personalizado carita feliz :)
const char smile[8] = {
	0b00000000,
	0b00001010,
	0b00001010,
	0b00001010,
	0b00000000,
	0b00010001,
	0b00001110,
	0b00000000,
};

const char e_char[8] = {
	0b01110,
	0b10000,
	0b10000,
	0b01100,
	0b01000,
	0b10000,
	0b10001,
	0b01110
};

const char r_char[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01110,
	0b01000,
	0b01000,
	0b01000,
	0b01000
};

const char i_char[8] = {
	0b00000,
	0b00100,
	0b00000,
	0b00100,
	0b00100,
	0b00100,
	0b00100,
	0b00110
};

const char c_char[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00110,
	0b01000,
	0b01000,
	0b01001,
	0b00110
};

const char tempChar[8] = { //Temperatura - Termometro
	0b01110,
	0b01010,
	0b01010,
	0b01110,
	0b01110,
	0b11111,
	0b11111,
	0b01110
};

const char humChar[8] = { // Humedad - Gota
	0b00100,
	0b00100,
	0b01110,
	0b10111,
	0b10111,
	0b10011,
	0b01110,
	0b00000
};

const char vieChar[8] = { // Viento
	0b01000,
	0b00100,
	0b10010,
	0b01001,
	0b00100,
	0b10010,
	0b01000,
	0b00100
};


void rxIsr( void ){
   uint8_t receivedByte = 0;
   uartReadByte( UART_SELECTED, &receivedByte );
   uartWriteByte( UART_SELECTED, receivedByte );
}

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------

   // Inicializar y configurar la plataforma
   boardConfig();

   // Inicializar UART_USB como salida de consola
   //consolePrintConfigUart( UART_USB, 115200 );
   
   uartConfig( UART_SELECTED, 115200 );
   uartRxInterruptCallbackSet( UART_SELECTED, rxIsr );
   uartRxInterruptSet( UART_SELECTED, ON );

//   lcdInit( 16, 2, 5, 8 );

   // Cargar el caracter a CGRAM
   // El primer parámetro es el código del caracter (0 a 7).
   // El segundo es el puntero donde se guarda el bitmap (el array declarado anteriormente)
   /*
   lcdCreateChar( 0, e_char );
   lcdCreateChar( 1, r_char );
   lcdCreateChar( 2, i_char );
   lcdCreateChar( 3, c_char );
   lcdCreateChar( 4, smile );
   */

/*
   lcdCreateChar( 0, tempChar );
   lcdCreateChar( 1, humChar );
   lcdCreateChar( 2, vieChar );


   char tempString[] = "1014";
   char humString[] = "0495";
   char vieString[] = "0763";
*/
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {
      //sleepUntilNextInterrupt();
/*    lcdClear(); // Borrar la pantalla

      lcdGoToXY( 1, 1 ); // Poner cursor en 1, 1

      lcdSendStringRaw( "Temp" );
      lcdData(0);
      lcdSendStringRaw( " Hum" );
      lcdData(1);
      lcdSendStringRaw( "  Vie" );
      lcdData(2);
      // lcdSendStringRaw( "Temp  Hum   Vie " );

      lcdGoToXY( 1, 2 );  // Poner cursor en 1, 2
      lcdSendStringRaw( tempString );
      lcdGoToXY( 7, 2 );  // Poner cursor en 7, 2
      lcdSendStringRaw( humString );
      lcdGoToXY( 13, 2 ); // Poner cursor en 13, 2
      lcdSendStringRaw( vieString );

      delay(2000);

      lcdClear(); // Borrar la pantalla

      lcdGoToXY( 1, 1 ); // Poner cursor en 1, 1
      lcdSendStringRaw( "Hola" );

      delay(1000);
*/
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
