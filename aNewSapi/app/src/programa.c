/*============================================================================
 * Licencia: 
 * Autor: 
 * Fecha: 
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "programa.h"   // <= su propio archivo de cabecera
#include "sapi.h"       // <= Biblioteca sAPI

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();   
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {      
      // Si se presiona TEC1, enciende el LEDR
      gpioWrite( LEDR, !(gpioRead(TEC1)) );
      
      // Si se presiona TEC2, enciende el LED1
      gpioWrite( LED1, !(gpioRead(TEC2)) );
      
      // Si se presiona TEC3, enciende el LED2
      gpioWrite( LED2, !(gpioRead(TEC3)) );
      
      // Si se presiona TEC4, enciende el LED3
      gpioWrite( LED3, !(gpioRead(TEC4)) );

      // Intercambia el valor del LEDB
      gpioToggle( LEDB );
      
      // Retardo bloqueante durante 100ms
      delay( 100 );
   } 

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta 
   // directamenteno sobre un microcontroladore y no es llamado/ por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
