Biblioteca sAPI en github:

https://github.com/epernia/sAPI

Uso:

Para usarla bajar la release y copiar la carpeta sapi_bm en Firmware/Modules. Luego copiar el proyecto bare_metal de Firmware/examples y renombrarlo a, por ejemplo, miProyectoConSapi y modificar el Makefile del mismo dentro de la carpeta mak cambiando la última linea de:

MODS                         += externals$(DS)drivers

por:

MODS                         += modules$(DS)sapi_bm

Para usarla en el programa incluir "sapi.h" donde se requiera. No es necesario incluir chip.h ya que la propia biblioteca sAPI lo incluye.

La sAPI también maneja el vector de interrupción, es por esto que es necesario eliminar vector.c


Documentación (en progreso) en:

https://github.com/epernia/sAPI/tree/master/docs/index.md

https://github.com/epernia/sAPI/tree/master/docs/assets/pdf

NOTA: En la documentación aparecen también módulos que estamos terminando y otros comenzando.


Por que se hizo:

Salio de la necesidad de manejar los periféricos directamente desde Java, es la parte de bajo nivel de las clases de periféricos en Java que básicamente bindea a funciones escritas en C. La idea es tener periféricos abstractos y lo más genéricos posibles. Que sea bien independiente de la arquitectura y en lo posible que las funciones sean todas del tipo:

   moduloConfig();
   moduloRead();
   moduloWrite();

Con nombres sencillos al estilo de la biblioteca Wiring. Por último, a diferencia de Wiring los toma por perièrico y no por pines para lograr una API unificada. Porque si miran Wiring con los perifèricos de màs de un pin hace otra cosa distinta y se manejan nada que ver.





Archivos que componen la biblioteca:

   sAPI.h
   sAPI_AnalogIO.h
   sAPI_Board.h
   sAPI_Config.h
   sAPI_DataTypes.h
   sAPI_Delay.h
   sAPI_DigitalIO.h
   sAPI_IsrVector.h
   sAPI_PeripheralMap.h
   sAPI_Tick.h
   sAPI_Uart.h

   sAPI_AnalogIO.c
   sAPI_Board.c
   sAPI_Delay.c
   sAPI_DigitalIO.c
   sAPI_IsrVector.c
   sAPI_Tick.c
   sAPI_Uart.c

Uso:

   #include "sAPI.h"

API:

sAPI_Config.h

	Configuraciones actuales:
	   SAPI_USE_TICK_HOOK (TRUE o FALSE)

sAPI_DataTypes.h

Define las siguientes constantes.

   Estados lógicos:
      TRUE  = 1
      FALSE = 0

   Estados funcionales:
      ON  = 1
      OFF = 0

   Estados eléctricos:
      HIGH = 1
      LOW  = 0

Además define los tipos de datos:
      Booleano          - bool_t
      Enteros sin signo - uint8_t, uint16_t, uint32_t, uint64_t
      Enteros con signo - int8_ntt, int16_t, int32_t, int64_t

      (ver, no funcionan) Reales - real32_t, real64_t

sAPI_IsrVector.h

Contiene la tabla de vectores de interrupción.

sAPI_Board.h

Contiene la función de configuración para inicialización de la plataforma de hardware:

   void boardConfig( void );

sAPI_PeripheralMap.h

Contiene las definiciones de los nombres de los periféricos.

   DigitalIO Map
      
      DIO0,  DIO1,  DIO2,  DIO3,  DIO4,  DIO5,  DIO6,  DIO7,
      DIO8,  DOI9,  DIO10, DIO11, DIO12, DIO13, DIO14, DIO15,
      DIO16, DIO17, DIO18, DIO19, DIO20, DIO21, DIO22, DIO23,
      DIO24, DIO25, DIO26, DIO27, DIO28, DIO29, DIO30, DIO31,
      DIO32, DIO33, DIO34, DIO35,
      TEC1,  TEC2,  TEC3,  TEC4,
      LED1,  LED2,  LED3,  LEDR,  LEDG,  LEDB

   AnalogIO Map
      AI0, AI1, AI2, AO

   Uart Map
      UART_USB, UART_232, UART_485


sAPI_DigitalIO.h

Manejo de Entradas y Salidas digitales.

Configuración inicial y modo de una entrada o salida

   bool_t digitalConfig( int8_t pin, int8_t config);

      Configuraciones:
         INITIALIZE
         INPUT, INPUT_PULLUP, INPUT_PULLDOWN, INPUT_REPEATER
         OUTPUT

Lectura de Entrada digital.
         
   bool_t digitalRead( int8_t pin );

Escriturade Salida Digital.

   bool_t digitalWrite( int8_t pin, bool_t value );

sAPI_Tick.h

Configuración de interrupción periódica de temporizador cada tickRateMSvalue milisegundos para utilizar de base de tiempo del sistema. Se dice que ocurre un tick del sistema cada tickRateMSvalue milisegundos.

   bool_t tickConfig( tick_t tickRateMSvalue );

      La tasa de ticks en ms, tickRateMS, es un parámetro
      con rango de 1 a 50 ms.

Leer la variable del conteo actual de ticks (se incrementa en 1 cada tickRateMSvalue milisegundos.
      
   tick_t tickRead( void );

Escribir la variable del conteo actual de ticks.
   
   void tickWrite( tick_t ticks );

Función que se ejecuta en cada vez que ocurre 
   
   tick_t tickHook( void );

sAPI_Delay.h

   Retardo inexacto. Un for bloqueante que tiene una constante calculada a ojo.
      void delayInaccurate(tick_t delay_ms);
         INACCURATE_TO_MS = 20400

   Retardo bloqueante. Utiliza el conteo de ticks para determinarel tiempo transcurrido con lo cual resulta en un retardo exacto.
      void delay (tick_t time);

   Retardo no bloqueante. Este tipo de retardo permite realizar otras tareas mientras simplemente se chequea si el tiempo se ha arribado en lugar de quedarse bloqueado esperando a que se complete el tiempo como en los casos anteriores.
   
   Crear una variable de estructura del tipo delay_t (estructura).
      delay_t delay;
   
   Configura inicialmente el delay no bloqueante pasando como parámetro la variable de estructura del tipo delay_t
      void delayConfig( delay_t * delay, tick_t duration );
   
   La primera vez que leo si se cumplió el delay lo comienza a ejecutar. Devuelve 1 (true) cuando se completo y se vuelve a relanzar automáticamente.
      bool_t delayRead( delay_t * delay );
      
   Con esta función se puede cambiar la duración de un delay en tiempo de ejecución.
      void delayWrite( delay_t * delay, tick_t duration );

--------------------------------------------------------------

Ejemplos

ej1_sAPI_tecsLeds. Este ejemplo maneja un led con cada tecla de la EDU-CIAA-NXP.

ej2_sAPI_blinky. Este ejemplo hace titilar un led (blink) de la EDU-CIAA-NXP utilizando un retardo bloqueante.

ej3_sAPI_semaforo. Este ejemplo se emula un semáforo con los leds de la EDU-CIAA-NXP utilizando un retardo bloqueante.

ej4_sAPI_blinky_tecla. Este ejemplo hace titilar un led (blink) de la EDU-CIAA-NXP utilizando un retardo bloqueante. Esto permite manejar otro led con la TEC4 al mismo tiempo que parpadea.

ej5_sAPI_semaforo_tecla. Este ejemplo se emula un semáforo con los leds de la EDU-CIAA-NXP utilizando un retardo no bloqueante. Esto permite manejar otro led con la TEC4 al mismo tiempo que funciona el semáforo.

ej6_sAPI_secuencias. Con TEC1 y TEC 4 maneja la dirección de movimiento de los leds, mientras que con TEC2 y TEC4 cabia la velocidad de movimiento de lenta a rápida en la EDU-CIAA-NXP. Utiliza retardo no bloqueante y cambia su valor en ejecución.
      
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
