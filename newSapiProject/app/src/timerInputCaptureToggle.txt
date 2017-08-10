/*============================================================================
 * Licencia: 
 * Autor: 
 * Fecha: 
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "board.h"
#include <stdio.h>

#include "programa.h"   // <= su propio archivo de cabecera
#include "sapi.h"       // <= Biblioteca sAPI

#define TICKRATE_HZ        10

#define CAP_NUMB           2
#define TIMER0_PRESCALER   10

CONSOLE_PRINT_ENABLE

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   
   uint32_t timerFreq;

   SystemCoreClockUpdate();
   Board_Init();
   
   
// Configurar UART y GPIOs
   
   consolePrintConfigUart( UART_USB, 115200 );
  
   gpioConfig( 0, GPIO_ENABLE ); // Inicializar GPIOs

   // Configuracion de pines de entrada para Teclas de la EDU-CIAA-NXP
   gpioConfig( TEC1, GPIO_INPUT );
   gpioConfig( TEC2, GPIO_INPUT );
   gpioConfig( TEC3, GPIO_INPUT );
   gpioConfig( TEC4, GPIO_INPUT );

   // Configuracion de pines de salida para Leds de la EDU-CIAA-NXP
   gpioConfig( LEDR, GPIO_OUTPUT );
   gpioConfig( LEDG, GPIO_OUTPUT );
   gpioConfig( LEDB, GPIO_OUTPUT );
   gpioConfig( LED1, GPIO_OUTPUT );
   gpioConfig( LED2, GPIO_OUTPUT );
   
   gpioConfig( GPIO0, GPIO_OUTPUT ); // Salida de frecuencia de 10Hz a coenctar a la entrada de captura ENET_TXD1 --> P1.20, FUNC4 --> T0_CAP2
   
   
// Configurar pin P1.5 como GPIO1[8] como salida

   Chip_SCU_PinMuxSet( 0x1, 5, (SCU_MODE_FUNC0) );         // P1.5 whith SCU_MODE_FUNC0 is GPIO1[8]
   Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 0x1, 8 );     // Set GPIO1[8] as Output
   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0x1, 8, false );  // GPIO1[8] = FALSE


// Configurar TIMER1 modo comparacion para generar una interrupcion periodica a TICKRATE_HZ

   // Enable timer 1 clock and reset it
   Chip_TIMER_Init( LPC_TIMER1 );
   Chip_RGU_TriggerReset( RGU_TIMER1_RST );
   while( Chip_RGU_InReset( RGU_TIMER1_RST ) ){}

   // Get timer 1 peripheral clock rate
   timerFreq = Chip_Clock_GetRate( CLK_MX_TIMER1 );

   // Timer setup for match and interrupt at TICKRATE_HZ
   Chip_TIMER_Reset( LPC_TIMER1 );
   Chip_TIMER_MatchEnableInt( LPC_TIMER1, 1 );
   Chip_TIMER_SetMatch( LPC_TIMER1, 1, (timerFreq/TICKRATE_HZ) );
   Chip_TIMER_ResetOnMatchEnable( LPC_TIMER1, 1 );


// Configurar TIMER0 en modo entrada de captura 1 
   
   // Configurar pin P1.20 como T0_CAP2
   // ENET_TXD1 --> P1.20, FUNC4 --> T0_CAP2
   Chip_SCU_PinMux( 0x1,20, MD_PLN_FAST, SCU_MODE_FUNC4 ); // P1.20 whith SCU_MODE_FUNC4 is T0_CAP2
                // T0_CAP2
   LPC_GIMA->CAP0_IN[0][2]|= 0x20;

   // Enable timer 0 clock and reset it
   Chip_TIMER_Init( LPC_TIMER0 );
   Chip_TIMER_Reset( LPC_TIMER0 );
   Chip_TIMER_TIMER_SetCountClockSrc( LPC_TIMER0, TIMER_CAPSRC_RISING_PCLK, CAP_NUMB );
   Chip_TIMER_PrescaleSet( LPC_TIMER0, TIMER0_PRESCALER );
   Chip_TIMER_ClearCapture( LPC_TIMER0, CAP_NUMB );
   Chip_TIMER_CaptureRisingEdgeEnable( LPC_TIMER0, CAP_NUMB );
   Chip_TIMER_CaptureFallingEdgeEnable( LPC_TIMER0, CAP_NUMB );
   Chip_TIMER_CaptureEnableInt( LPC_TIMER0, CAP_NUMB );

// Hailitar interrupcion de TIMER1

   // Enable timer interrupt
   NVIC_EnableIRQ( TIMER1_IRQn );
   NVIC_ClearPendingIRQ( TIMER1_IRQn );

// Hailitar interrupcion de TIMER0

   // Enable timer interrupt
   NVIC_EnableIRQ( TIMER0_IRQn );
   NVIC_ClearPendingIRQ( TIMER0_IRQn );

// Hailitar TIMER0 y TIMER1

   Chip_TIMER_Enable( LPC_TIMER0 );
   Chip_TIMER_Enable( LPC_TIMER1 );
   
// Print

   consolePrintlnString( "Blinky example using TIMER1" ); // \r\n
   
   consolePrintString( "Timer 1 clock     = " );
   consolePrintInt( timerFreq );
   consolePrintlnString( "Hz" );
   // DEBUGOUT("Timer 1 clock     = %d Hz\r\n", timerFreq);
   
   consolePrintString( "Timer 1 tick rate = " );
   consolePrintInt( TICKRATE_HZ );
   consolePrintlnString( "Hz" );
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {       
		//__WFI();
   } 

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta 
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/**
 * @brief  Reads a capture register
 * @param  pTMR    : Pointer to timer IP register address
 * @param  capnum  : Capture register to read
 * @return The selected capture register value
 * @note   Returns the selected capture register value.

STATIC INLINE uint32_t Chip_TIMER_ReadCapture(LPC_TIMER_T *pTMR, int8_t capnum)
{
   return pTMR->CR[capnum];
}
 */

/*==================[definiciones de funciones externas]=====================*/

/*==================[Interrupt handlers]=====================================*/

/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing
 */
void TIMER1_IRQHandler(void)
{
	static bool On = false;

	if( Chip_TIMER_MatchPending( LPC_TIMER1, 1 ) ){
		Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
		On = (bool) !On;
		//Board_LED_Set(0, On);
      //Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0x1, 8, On );
      gpioWrite( GPIO0, On );
	}
}


/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing
 */
void TIMER0_IRQHandler(void)
{
	static bool On = false;
	if( Chip_TIMER_CapturePending( LPC_TIMER0, 2 ) ){
		Chip_TIMER_ClearCapture( LPC_TIMER0, 2 );
		On = (bool) !On;
		//Board_LED_Set( 0, On );
      gpioWrite( LED1, On );
	}
}

/*==================[fin del archivo]========================================*/