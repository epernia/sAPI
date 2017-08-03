/*============================================================================
 * Licencia: 
 * Autor: 
 * Fecha: 
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "programa.h"   // <= su propio archivo de cabecera
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

// Handle interrupt from GPIO pin or GPIO pin mapped to PININT
void PININT_IRQ_HANDLER(void)
{
   Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   gpioToggle(LEDB);
}

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();   
   
   /* Configuración de GPIO0 de la EDU-CIAA-NXP como entrada con pull-up */
   gpioConfig( GPIO0, GPIO_INPUT_PULLUP );

   
   // Comienzo de funciones LPCOpen para configurar la interrupción

   // Configure interrupt channel for the GPIO pin in SysCon block
   Chip_SCU_GPIOIntPinSel( PININT_INDEX, GPIO0_GPIO_PORT, GPIO0_GPIO_PIN );

   // Configure channel interrupt as edge sensitive and falling edge interrupt
   Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );
   Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX) );

   // Enable interrupt in the NVIC
   NVIC_ClearPendingIRQ( PININT_NVIC_NAME );
   NVIC_EnableIRQ( PININT_NVIC_NAME );

   // Fin de funciones LPCOpen
   
   
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
