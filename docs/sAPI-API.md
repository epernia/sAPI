Biblioteca sAPI en github:

https://github.com/epernia/sAPI

Uso:

Para usarla bajar la release y copiar la carpeta sapi_bm en Firmware/Modules.
Luego copiar el proyecto bare_metal de Firmware/examples y renombrarlo a, por
ejemplo, miProyectoConSapi y modificar el Makefile del mismo dentro de la
carpeta mak cambiando la última linea de:

MODS                         += externals$(DS)drivers

por:

MODS                         += modules$(DS)sapi_bm

Para usarla en el programa incluir

#include "sAPI.h"

en el archivo .c donde se requiera. Ya no es necesario incluir "chip.h" ya que la
propia biblioteca sAPI lo incluye.

La sAPI también maneja el vector de interrupción, es por esto que es necesario
eliminar el archivo "vector.c" que trae el ejemplo "bare_metal".




Documentación (en progreso) en:

https://github.com/epernia/sAPI/tree/master/docs/assets/pdf

--------------------------------------------------------------


## Motivación

Esta biblioteca de C surge de la necesidad de manejar los periféricos
directamente desde una VM de Java para el desarrollo de Java sobre la CIAA y
corresponde a la parte de bajo nivel de las clases de periféricos en Java que
básicamente bindea a funciones escritas en C.

Luego se extendió la misma para facilitar el uso de la EDU-CIAA-NXP a personas
no expertas en la arquitectura del LPC4337 facilitando el uso de esta plataforma.


## Principios de diseño

Se basa en definir periféricos abstractos y lo más genéricos posibles, logrando
independencia de la arquitectura. Esta independecia es deseada para que actúe
como HAL.

Se diseña además una API sencilla en la que la mayoría de las funciones son
del tipo:

- nombreModuloConfig();
- nombreModuloRead();
- nombreModuloWrite();

Con nombres sencillos al estilo de la biblioteca *Wiring*. Pero, a diferencia de
*Wiring*, toma el mapeo por periféricos y no por pines para lograr una API
unificada.





--------------------------------------------------------------

### ej6_sAPI_secuencias

Con TEC1 y TEC 4 maneja la dirección de movimiento de los leds, mientras que con
TEC2 y TEC4 cabia la velocidad de movimiento de lenta a rápida en la
EDU-CIAA-NXP. Utiliza retardo no bloqueante y cambia su valor en ejecución.

--------------------------------------------------------------


sAPI_AnalogIO.h

   void analogConfig( uint8_t config );

      Configuraciones:
         ENEABLE_ANALOG_INPUTS,  DISABLE_ANALOG_INPUTS,
         ENEABLE_ANALOG_OUTPUTS, DISABLE_ANALOG_OUTPUTS

   uint16_t analogRead( uint8_t analogInput );

   void analogWrite( uint8_t analogOutput, uint16_t value );

sAPI_Uart.h

   void uartConfig( uint8_t uart, uint32_t baudRate );

      baudRate:
         9600, 115200, etc.

   uint8_t uartReadByte( uint8_t uart );
   void uartWriteByte( uint8_t uart, uint8_t byte );

   void uartWriteString( uint8_t uart, uint8_t * str );

   void uartWriteByteArray( uint8_t uart, uint8_t * byteArray, uint16_t byteArraySize );




----------------------------------------------------------------------------------------

NOTAAA: MEJORAR DIGITAL IO

   Orden:

   sAPI_Config.h
   sAPI_DataTypes.h
   sAPI_IsrVector.h
   sAPI_Board.h
   sAPI_PeripheralMap.h
   sAPI_DigitalIO.h
   sAPI_Tick.h
   sAPI_Delay.h
   sAPI_AnalogIO.h
   sAPI_Uart.h
