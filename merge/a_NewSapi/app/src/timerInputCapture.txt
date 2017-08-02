/*============================================================================
 * Licencia: 
 * Autor: 
 * Fecha: 
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "programa.h"   // <= su propio archivo de cabecera
#include "sapi.h"       // <= Biblioteca sAPI

#define TICKRATE_HZ        10
#define TIMER0_PRESCALER   10

#define CAP_NUMB           2

CONSOLE_PRINT_ENABLE

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

volatile bool_t calculated = FALSE;
volatile uint32_t tCapturadoAnterior = 0;
volatile uint32_t tCapturadoActual = 0;
volatile uint32_t result = 0;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[Menejo de interrupciones]===============================*/

// Cuando se de el evento COMPARE_MATCH de TIMER1, ejecuta la siguiente funcion
bool_t tickCallback( void* ptr ){
   gpioToggle( GPIO0 );
}

// Cuando se de el evento INGRESA UN FLANCO ASCENDENTE DE UN PULSO en TIMER0, ejecuta la siguiente funcion
bool_t timer0CaptureCallback( void* ptr ){
   gpioToggle( LED1 );
   if( calculated == FALSE ){
      consolePrintString( "tCapturadoAnterior: " ); 
      tCapturadoAnterior = timerReadCapture( TIMER0, TIMER_CAPTURE2 );
      calculated = TRUE;
      consolePrintlnInt( tCapturadoAnterior );
   } else{
      tCapturadoActual = timerReadCapture( TIMER0, TIMER_CAPTURE2 );     
      consolePrintString( "tCapturadoActual: " ); 
      consolePrintlnInt( tCapturadoActual );
      
      // Prescale = 10, CPU_CLK = 204MHz, con *1000 da en ms
      result = ( tCapturadoActual - tCapturadoAnterior ) / (timerGetClock( TIMER0)/1000000);
      tCapturadoAnterior = tCapturadoActual;
      
      consolePrintString( "F_CPU[MHz]: " );
      consolePrintlnInt( 204 );
      
      consolePrintString( "Prescale_Timer: " );
      consolePrintlnInt( timerGetPrescale( TIMER0) );
      
      consolePrintString( "F_Timer: " );
      consolePrintlnInt( timerGetClock( TIMER0) );
      
      consolePrintString( "T[us] medido: " );
      consolePrintlnInt( result );
      
      consolePrintString( "F[Hz*10] medido: " );
      consolePrintlnInt( 10000000/result );
      consolePrintEnter();
   }
}

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   boardConfig(); // Configurar plataforma

   gpioConfig( GPIO0, GPIO_OUTPUT ); // Configurar GPIO0 como Salida
   // El GPIO0 se toglea a una frecuencia de 10Hz generada con TIMER1 en modo TICKER
   // GPIO0 se debe coenctar a la entrada de captura 2 del TIMER0 (pin ENET_TXD1 en EDU-CIAA-NXP)
   // ENET_TXD1 --> P1.20, FUNC4 --> T0_CAP2

   // Configurar Timer 1 en modo TICKER 
   timerTickerConfig( TIMER1, TICKRATE_HZ );
   timerTickerSetTickEvent( TIMER1, tickCallback );

   // Configurar Timer 0 en modo INPUT_CAPTURE (solo en flanco de subida)
   timerInputCaptureConfig( TIMER0, TIMER_CAPTURE2,
                            TIMER0_PRESCALER,
                            TRUE, FALSE );
   timerInputCaptureSetCaptureEvent( TIMER0, TIMER_CAPTURE2,
                                     timer0CaptureCallback );
   
   // Hailitar TIMER0 y TIMER1
   timerSetPower( TIMER0, ON );
   timerSetPower( TIMER1, ON );
   
   
   // Print
   consolePrintConfigUart( UART_USB, 115200 ); // Configurar UART

   consolePrintlnString( "Blinky example using TIMER1" ); // \r\n
   
   consolePrintString( "Timer 1 clock     = " );
   consolePrintInt( TICKRATE_HZ );
   consolePrintlnString( "Hz" );
   // DEBUGOUT("Timer 1 clock     = %d Hz\r\n", timerFreq);
   
   consolePrintString( "Timer 1 tick rate = " );
   consolePrintInt( TICKRATE_HZ );
   consolePrintlnString( "Hz" );
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {  
      //sleepUntilNextInterrupt();
   } 

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta 
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[Interrupt handlers]=====================================*/

/*==================[fin del archivo]========================================*/