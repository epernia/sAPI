# API de la biblioteca sAPI

## Módulos

### sAPI_DataTypes

Define las siguientes constantes:

Estados lógicos

``#define FALSE 0``

``#define TRUE !FALSE``

Estados funcionales

``#define ON 1``

``#define OFF 0``

Estados eléctricos

``#define HIGH 1``

``#define LOW 0``

Además define los tipos de datos:

- **Booleano** ``bool_t``
- **Enteros sin signo** ``uint8_t, uint16_t, uint32_t, uint64_t``
- **Enteros con signo** ``int8_t, int16_t, int32_t, int64_t``

El tipo de datos para el conteo de tiempo en la unidad Tick

``typedef uint64_t tick_t;``

Un tipo de datos para puntero a función:

``typedef bool_t (*sAPI_FuncPtr_t)(void *);``

- Parámetro: ``void *`` Para poder pasar cualquier argumento.
- Retorna: ``bool_t`` Para reportar errores (TRUE si todo está bien).

Utilizando este tipo de datos define la finción Nula que no hace nada y Retorna
siempre TRUE, esta se utiliza para evitar errores de NULL POINTER.

``bool_t sAPI_NullFuncPtr(void *);``

- Parámetro: ``void *`` No usado.
- Retorna: ``bool_t``Retorna siempre TRUE.


### sAPI_PeripheralMap

Contiene el mapa de periféricos.

DigitalIO Map

      EDU-CIAA-NXP

      DIO0,  DIO1,  DIO2,  DIO3,  DIO4,  DIO5,  DIO6,  DIO7,
      DIO8,  DOI9,  DIO10, DIO11, DIO12, DIO13, DIO14, DIO15,
      DIO16, DIO17, DIO18, DIO19, DIO20, DIO21, DIO22, DIO23,
      DIO24, DIO25, DIO26, DIO27, DIO28, DIO29, DIO30, DIO31,
      DIO32, DIO33, DIO34, DIO35,
      TEC1,  TEC2,  TEC3,  TEC4,
      LED1,  LED2,  LED3,  LEDR,  LEDG,  LEDB

      CIAA-NXP

      DI0,   DI1,   DI2,   DI3,   DI4,   DI5,   DI6,   DI7,
      DO0,   DO1,   DO2,   DO3,   DO4,   DO5,   DO6,   DO7

AnalogIO Map

      EDU-CIAA-NXP

      AI0, AI1, AI2, AO

      CIAA-NXP

      AI0, AI1, AI2, AI3, AO

Uart Map

      UART_USB, UART_232, UART_485

Pwm Map

      PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10

Servo Map

      SERVO0, SERVO1, SERVO2, SERVO3, SERVO4, SERVO5, SERVO6, SERVO7, SERVO8


### sAPI_IsrVector

Contiene la tabla de vectores de interrupción.

### sAPI_Board

Contiene la función de configuración para inicialización de la plataforma de
hardware:

``void boardConfig( void );``

- Parámetro: ``void``
- Retorna: ``void``

### sAPI_Tick

**Configuración de interrupción periódica**

``bool_t tickConfig(tick_t tickRateMSvalue, sAPI_FuncPtr_t tickHook );``

- Parámetro: ``tick_t tickRateMSvalue`` cada cuantos ms ocurre un tick.
- Parámetro: ``sAPI_FuncPtr_t tickHook`` función a ejecutar en cada tick.
- Retorna: ``bool_t`` TRUE en en caso correcto o FALSE en caso de errores.

Configura una interrupción periódica de temporizador cada tickRateMSvalue
milisegundos para utilizar de base de tiempo del sistema. Una vez ejecutada
esta función se dice que ocurre un tick del sistema cada tickRateMSvalue
milisegundos.

La tasa de ticks en ms, tickRateMS, es un parámetro con rango de 1 a 50 ms.

Además de aumentar el conteo de ticks en cada interrupción, la función
tickConfig ejecuta la función pasada como parámero cada vez que ocurre un tick.
Si no se desea ejecutar ninguna función debe poner en cero este parámetro.

**Leer la variable del conteo actual de ticks**

``tick_t tickRead( void );``

- Parámetro: ``void`` sin parámetro.
- Retorna: ``tick_t`` el valor actual del contador de ticks.

La variable del conteo actual de ticks se incrementa en 1 cada tickRateMSvalue
milisegundos.

**Escribir la variable del conteo actual de ticks**

``void tickWrite( tick_t ticks );``

- Parámetro: ``tick_t ticks`` el nuevo valor a setear del contador de ticks.
- Retorna: ``void``

Se utiliza si se necesita cambiar el valor del contador de ticks, por ejemplo,
para resetearlo.

En la implementación para la CIAA utiliza internaente el peiférico temporizador
Systick para configurar una interrupción periódica.


### sAPI_Delay

Para utilizar los retardos (con excepción del retardo inexacto) se debe
configurar el Tick ya que utiliza estas interrupciones como base de tiempo.

Todos los tiempos de parámetros están en milisegundos.

Define la constante ``#define INACCURATE_TO_MS 20400`` y contiene las funciones:

**Retardo inexacto bloqueante** ``void delayInaccurate(tick_t delay_ms);``

- Parámetros: ``tick_t delay_ms`` tiempo de duración del retardo en ms.
- Retorna: ``void``

Utiliza un bloque for bloqueante que tiene una constante calculada "a ojo"
(INACCURATE_TO_MS) para perder muchos ciclos de reloj y lograr hacer un retado.

**Retardo bloqueante** ``void delay (tick_t time);``

- Parámetros: ``tick_t time``
- Retorna: ``void``

Utiliza el conteo de ticks para determinar el tiempo transcurrido resultando en
un retardo exacto. Es bloqueante pues se queda en un bucle while hasta que se
cuentan los ticks necesarios para lograr el tiempo especificado.

**Retardo no bloqueante**

Este tipo de retardo permite realizar otras tareas mientras se ejecuta ya que
simplemente se chequea si el tiempo de retardo se ha arribado en lugar de
quedarse bloqueado esperando a que se complete el tiempo como en los casos
anteriores.

Define el tipo de datos estructurado ``delay_t``

Contiene las funciones:

``void delayConfig( delay_t * delay, tick_t duration );``

- Parámetro: ``delay_t * delay`` dirección de memoria de una variable del tipo delay_t.
- Parámetro: ``tick_t duration`` tiempo de duración del retardo en ms.
- Retorna: ``void``

``bool_t delayRead( delay_t * delay );``

- Parámetro: ``delay_t * delay`` dirección de memoria de una variable del tipo delay_t.
- Retorna: ``bool_t`` TRUE cuando el delay se cumplió, FALSE en caso contrario.

``void delayWrite( delay_t * delay, tick_t duration );``

- Parámetro: ``delay_t * delay`` dirección de memoria de una variable del tipo delay_t.
- Parámetro: ``tick_t duration`` tiempo de duración del retardo en ms.
- Retorna: ``void``


Uso:

Se utiliza declarando una variable de estructura del tipo delay_t, por ejemplo:

``delay_t myDelay;``

Luego, se configura inicialmente pasando como parámetro la variable recién
declarada

``delayConfig( &myDelay, 500 );``

Se detecta con un bloque if si se cumplió el delay leyéndolo con

``delayRead( &myDelay );``

La primera vez que se ejecuta delayRead activa el mismo. delayRead devuelve TRUE
cuando se completo y se vuelve a relanzar automáticamente.

Con ``delayWrite( &myDelay, 1000 );`` se puede cambiar la duración de un delay
en tiempo de ejecución.


### sAPI_DigitalIO

Manejo de Entradas y Salidas digitales.

**Configuración inicial y modo de una entrada o salida**

``bool_t digitalConfig( int8_t pin, int8_t config);``

- Parámetro: ``int8_t pin`` pin a configurar (ver Digital IO Map).
- Parámetro: ``int8_t config`` configuración.
- Retorna: ``bool_t`` TRUE si la configuración es correcta.

Posibles configuraciones:

``ENABLE_DIGITAL_IO`` Habilita las entradas y salidas digitales.

``INPUT, INPUT_PULLUP, INPUT_PULLDOWN, INPUT_REPEATER`` Pin configurado como
entrada digital en sus distintas variantes.

``OUTPUT`` Pin configurado como salida digital.

**Lectura de Entrada digital**

``bool_t digitalRead( int8_t pin );``

- Parámetro: ``int8_t pin`` pin a leer (ver Digital IO Map).
- Retorna: ``bool_t`` valor de la entrada digital.


**Escritura de Salida Digital**

``bool_t digitalWrite( int8_t pin, bool_t value );``

- Parámetro: ``int8_t pin`` pin a escribir (ver Digital IO Map).
- Parámetro: ``bool_t value`` valor a escribir en el pin.
- Retorna: ``bool_t`` FALSE en caso de errores.

### sAPI_AnalogIO

Manejo de Entradas y Salidas analógicas.

**Configuración inicial de entradas o salidas analógicas**

``void analogConfig( uint8_t config );``

- Parámetro: ``uint8_t config`` configuración.
- Retorna: ``void``.

Posibles configuraciones:
- ``ENEABLE_ANALOG_INPUTS`` Habilita las entradas analógicas.
- ``DISABLE_ANALOG_INPUTS`` Deshabilita las entradas analógicas.
- ``ENEABLE_ANALOG_OUTPUTS`` Habilita las salidas analógicas.
- ``DISABLE_ANALOG_OUTPUTS`` Deshabilita las salidas analógicas.

**Lectura de Entrada analógica**

``uint16_t analogRead( uint8_t analogInput );``

- Parámetro: ``uint8_t analogInput`` pin a leer (ver Analog IO Map).
- Retorna: ``uint16_t`` el valor actual de la entrada analógica.

**Escritura de Salida analógica**

``void analogWrite( uint8_t , uint16_t value );``

- Parámetro: ``uint8_t analogOutput`` pin a escribir (ver Analog IO Map).
- Parámetro: ``uint16_t value`` valor del pin a escribir.
- Retorna: ``void``.


>>>>>>>>>FALTA REVISAR LA API DESDE UART>>>>>>>>>>>>>>>>>>>>>>>><

### sAPI_Uart

``void uartConfig( uint8_t uart, uint32_t baudRate );``

   baudRate:
      9600, 115200, etc.

``uint8_t uartReadByte( uint8_t uart );``
``void uartWriteByte( uint8_t uart, uint8_t byte );``

``void uartWriteString( uint8_t uart, uint8_t * str );``

``void uartWriteByteArray( uint8_t uart, uint8_t * byteArray, uint16_t byteArraySize );``

### sAPI_I2c


### sAPI_Rtc


### sAPI_Pwm


### sAPI_Servo


### sAPI_Hmc5883l


## Archivos que componen la biblioteca

**src** (.c):

- sAPI_AnalogIO.c
- sAPI_Board.c
- sAPI_Delay.c
- sAPI_DigitalIO.c
- sAPI_IsrVector.c
- sAPI_Tick.c
- sAPI_Uart.c

**inc** (.h):

- sAPI.h
- sAPI_AnalogIO.h
- sAPI_Board.h
- sAPI_Config.h
- sAPI_DataTypes.h
- sAPI_Delay.h
- sAPI_DigitalIO.h
- sAPI_IsrVector.h
- sAPI_PeripheralMap.h
- sAPI_Tick.h
- sAPI_Uart.h
