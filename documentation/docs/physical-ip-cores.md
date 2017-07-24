# Periféricos físicos de la biblioteca sAPI

Modelan peiféricos con existencia física en el SoC. Estos periféricos no son
"creables", solamente son "inicializables", además, tienen definidos nombres
prefijados que no se pueden destruir o crear dos veces. Es decir, no se puede,
por ejemplo, tener 10 SPI apuntando al mismo elemento fisico; SPI0, es SPI0 
desde que el software arranca hasta que termina o se destrulle el chip.

Los periféricos modelados son:

- CORE0 a CORE8.
- GPIO0 a GPIO63.
- ADC0 a ADC7. Tienen 8 canales cada uno.
- DAC0 a DAC7.
- TIMER0 a TIMER7.
- RCT0 a RCT3.
- UART0 a UART7.
- SPI0 a SPI7.
- I2C0 a I2C7.
- POWER_MANAGEMENT.

## CORE

Modela un núcleo de CPU.

### Propiedades de CORE

**Propiedades de configuración**

- Nunguno.

**Propiedades de valor**

- Nunguno.

**Propiedades de eventos por polling**

- Nunguno.

**Propiedades de eventos por interrupción**

- ``coreInterrupts`` (tipo ``coreConfig_t``). Valores posibles:
    - ``CORE_INTERRUPTS_DISABLE`` *(valor por defecto)*
    - ``CORE_INTERRUPTS_ENABLE``

### Métodos de CORE

**Getters y Setters de todas sus propiedades**

...


## GPIO

GPIO modela un único pin de entrada/salida de porpósito general. El valor de un
pin es booleano.

### Propiedades de GPIO

**Propiedades de configuración**

- ``mode`` (tipo ``gpioConfig_t``). Valores posibles:
    - ``GPIO_INPUT`` *(valor por defecto)*. Posibles flags de modificación:
        - ``GPIO_NOPULL`` *(valor por defecto)*
        - ``GPIO_PULLUP``
        - ``GPIO_PULLDOWN``
        - ``GPIO_PULLUP | GPIO_PULLDOWN``
        - ``GPIO_PULLUPDOWN``
    - ``GPIO_OUTPUT``. Posibles flags de modificación:
        - ``GPIO_PUSHPULL`` *(valor por defecto)*
        - ``GPIO_PUSHPULL | GPIO_STRENGTH(i)`` (i = 0,...,7)
        - ``GPIO_OPENCOLLECTOR`` ( es equivalente también ``GPIO_OPENDRAIN``)
        - ``GPIO_OPENCOLLECTOR | GPIO_PULLUP``
- ``speed`` (tipo ``gpioConfig_t``). Valores posibles:
    - ``GPIO_SPEED(i)`` (i = 0,...,7)
- ``power`` (tipo ``gpioConfig_t``). Valores posibles:
    - ``ON`` *(valor por defecto)*, ``OFF``, ``ENABLE`` o ``DISABLE``

**Propiedades de valor**

- ``value`` (tipo ``bool_t``). Valores posibles:
    - ``ON``, ``OFF``, ``HIGH``, ``LOW``, ``TRUE`` o ``FALSE``

**Propiedades de eventos por polling**

- Nunguno.

**Propiedades de eventos por interrupción**

- ``inputInterrupt`` (tipo ``gpioConfig_t``). Valores posibles:
    - ``GPIO_INTERRUPT_DISABLE`` *(valor por defecto)*
    - ``GPIO_LEVEL`` Level-sensitive (high/low). Posibles flags de modificación:
        - ``GPIO_LEVEL_HIGH`` *(valor por defecto)*
        - ``GPIO_LEVEL_LOW``
        - ``GPIO_LEVEL_HIGH | GPIO_LEVEL_LOW``
        - ``GPIO_LEVEL_BOTH``
    - ``GPIO_EDGE`` Edge (Rising/falling). Posibles flags de modificación:
        - ``GPIO_EDGE_RISING`` *(valor por defecto)*
        - ``GPIO_EDGE_FALLING``
        - ``GPIO_EDGE_RISING | GPIO_EDGE_FALLING``
        - ``GPIO_EDGE_BOTH``
    - ``GPIO_ASYNCHRONOUS_EDGE`` Asynchronous Edge (Rising/falling). Posibles flags de modificación:
        - ``GPIO_EDGE_RISING`` *(valor por defecto)*
        - ``GPIO_EDGE_FALLING``
        - ``GPIO_EDGE_RISING | GPIO_EDGE_FALLING``
        - ``GPIO_EDGE_BOTH``
- ``inputInterruptCallback`` (tipo ``Callback_t``). Valores posibles:
    - Una estructura con el puentero a función y el puntero al parámetro que le pueda pasar el usuario a dicha función. *(valor por defecto)* un null pointer con un while(1) *(TODO)*


### Métodos de GPIO

**Getters y Setters de todas sus propiedades**

- Getters y Setters de configuración:
    - ``mode``
        - ``gpioSetMode( GPIO<i>, GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_STRENGTH(7) );``
        - ``gpioMode = gpioGetMode( GPIO<i> );``
    - ``speed``
        - ``gpioSetSpeed( GPIO<i>, GPIO_SPEED(7) );``
        - ``gpioSpeed = gpioGetSpeed( GPIO<i> );``
    - ``power``
        - ``gpioSetPower( GPIO<i>, ON );``
        - ``booleanValue = gpioGetPower( GPIO<i> );``
- Getters y Setters de valor:
    - ``value``
        - ``gpioSetValue( GPIO<i>, HIGH );``
        - ``booleanValue = gpioGetValue( GPIO<i> );``
- Getters y Setters de eventos por interrupción:
    - ``inputInterrupt``
        - ``gpioSetInputInterrupt( GPIO<i>, GPIO_EDGE | GPIO_EDGE_RISING );``
        - ``gpioInputInterrupt = gpioGetInputInterrupt( GPIO<i> );``
    - ``inputInterruptCallback``

**Configuración de una entrada o salida**

``bool_t gpioConfig( gpioMap_t pin, gpio_t mode );``

- Parámetro: ``gpioMap_t pin`` pin a configurar (ver GPIO Map).
- Parámetro: ``gpioConfig_t config`` configuración.
- Retorna: ``bool_t`` TRUE si la configuración es correcta.

Alias de ``gpioSetMode();`

**Lectura de Entrada**

``bool_t gpioRead( gpioMap_t pin );``

- Parámetro: ``gpioMap_t pin`` pin a leer (ver GPIO Map).
- Retorna: ``bool_t`` estado del pin.

Alias de ``gpioGetValue();``

**Escritura de Salida**

``void gpioWrite( gpioMap_t pin, bool_t value );``

- Parámetro: ``gpioMap_t pin`` pin a escribir (ver GPIO Map).
- Parámetro: ``bool_t value`` valor a escribir en el pin.
- Retorna: nada.

Alias de ``gpioSetValue();``





## ADC

Modela periférico de Conversor Analógico-Digital (ADC en ingés).

### Propiedades de ADC

**Propiedades de configuración**

- ``conversionMode`` (tipo ``adcConfig_t``). Valores posibles:
    - ADC_SOFTWARE_TRIGGERED_CONVERSION *(valor por defecto)*: Modo de coversión disparada por software. En este modo hay que ejecutar el método ``adcStartConversion( ADC0, channel0 );`` o ``adcRead( ADC0, channel0 );`` para realizar una única conversión *(bloqueante)*.
    - ADC_HARDWARE_TRIGGERED_CONVERSION
        - ADC_CONTINUOUS_CONVERSION: Conversion continua o en ráfaga (brust). Se puede aplicar a una o múltiples entradas. Conversion periódica disparada a tasa samplingRate.
            - ``samplingRate`` (solo para conversionMode == ADC_CONTINUOUS_CONVERSION) *(TODO: ver si agregamos: clockSource, prescaler. En base a estos se calcula el samplingRate)*
        - ADC_TIMER_TRIGGERED: Inicia una conversión disparada por un timer. Puede ser Timer Match signal.
            - ``timerSource``
        - ADC_GPIO_TRIGGERED: Inicia una conversión mediante la transición de un GPIO.
            - ``gpioSource``

- ``channelMode`` (Modos de los canales analógicos). Valores posibles:
    - ADC_SINGLE_ENDED_INPUTS
        - ``channel<i>`` (con i=0,...,7)
    - ADC_DIFERENTIAL_INPUTS
        - ``differentialChannel<i>`` (con i=0,...,3) (usa 2 channels comunes, por ejemplo channel0 conectado a - en el amplificador operacional y channel1 conectado a + en el amplificador operacional)
        - gain (1x, 10x, 100x, 200x, ... )


- ``voltageRefereceHighSource`` (ADC_EXTERNAL_REF_HIGH_PIN, ANALOG_VCC, INTERNAL (ej: 2560 mV) )
    - ``voltageRefereceHigh`` (uint16_t representando el voltaje en mV)
- ``voltageRefereceLowSource`` (ADC_EXTERNAL_REF_LOW_PIN, GND)
    - ``voltageRefereceLow`` (uint16_t representando el voltaje en mV), 0 por defecto.
- converterResolution: resolution = 2^converterResolution (converterResolution=8, 10, 12, 16, 24, 32)
    - Entre estos se calcula el paso minimo de conversión en volts.
        - stepSize = (voltageRefereceHigh - voltageRefereceLow) / resolution
- ``power``

**Propiedades de valor**

- ``channel0``
- ``channel1``
- ``channel2``
- ``channel3``
- ``channel4``
- ``channel5``
- ``channel6``
- ``channel7``

**Propiedades de eventos por polling**

- Ninguno.

**Propiedades de eventos por interrupción**

- ``conversionCompleteInterrupt``
- ``conversionCompleteInterruptCallback``
- ``adcDmaTransferCompleteInterrupt``
- ``adcDmaTransferCompleteInterruptCallback``
- ``analogComparatorInterrupt``: Comparación del valor convertido contra un valor programado, para mayor que, igual que o menor que. 
    - ADC_GREATER_THAN: Modo comparación por mayor que.
    - ADC_LESS_THAN: Modo comparación por menor que.
- ``analogComparatorInterruptCallback``

### Métodos de ADC

**Getters y Setters de todas sus propiedades**


adcInitialize( adc, conversionMode, channelMode )

adcConfig()

adcRead()

Read the value on the analog pin and return it. The returned value will be between 0 and 4095.

adcReadTimed(buf, time)





## DAC

Manejo de conversor digital-analógico.

### Propiedades de DAC

**Propiedades de configuración**

- ``mode`` (tipo ``dacConfig_t``). Valores posibles:
    - ``DAC_NORMAL``
    - ``DAC_DMA_TRANSFER``
- ``conversionRate``
- ``resolution``
- ``power``


**Propiedades de valor**

**Propiedades de eventos por polling**

- Nunguno.

**Propiedades de eventos por interrupción**


### Métodos de DAC

**Getters y Setters de todas sus propiedades**

dacInitialize(bits=8) Reinitialise the DAC. bits can be 8 or 12.

dacDeinitialize() De-initialise the DAC making its pin available for other uses.

dacWrite(value) Direct access to the DAC output. The minimum value is 0. The maximum value is 2**``bits``-1, where bits is set when creating the DAC object or by using the init method.

dacWriteTimed(data, freq, *, mode=DAC.NORMAL)

- Initiates a burst of RAM to DAC using a DMA transfer. The input data is treated as an array of bytes in 8-bit mode, and an array of unsigned half-words (array typecode ‘H’) in 12-bit mode.
- freq can be an integer specifying the frequency to write the DAC samples at, using Timer(6). Or it can be an already-initialised Timer object which is used to trigger the DAC sample. Valid timers are 2, 4, 5, 6, 7 and 8.
- mode can be DAC.NORMAL or DAC.CIRCULAR.
- Example using both DACs at the same time:
- Modos:
    - DAC_NORMAL: saca muestras de un buffer y las manda a cierta tasa, enviando la secuencia una única vez.
    - DAC_CIRCULAR: saca muestras de un buffer y las manda a cierta tasa, repitiendo la secuencia en forma periodica.

dacNoise( freq ) Generate a pseudo-random noise signal. A new random sample is written to the DAC output at the given frequency.

dacTriangle( freq ) Generate a triangle wave. The value on the DAC output changes at the given frequency, and the frequency of the repeating triangle wave itself is 2048 times smaller.

dacSine( freq ) Generate a sine wave. The value on the DAC output changes at the given frequency, and the frequency of the repeating sine wave itself is 2048 times smaller.

**Configuración inicial de conversor digital-analógico**

``void dacConfig( dacConfig_t config );``

- Parámetro: ``dacConfig_t config`` configuración.
- Retorna: ``void``.

Posibles configuraciones:

- ``DAC_ENABLE`` Habilita el periférico DAC.
- ``DAC_DISABLE`` Deshabilita el periférico DAC.

**Escritura de Salida analógica**

``void dacWrite( dacMap_t analogOutput, uint32_t value );``

- Parámetro: ``dacMap_t analogOutput`` pin a escribir (ver DAC Map).
- Parámetro: ``uint32_t value`` valor del pin a escribir.
- Retorna: ``void``.




## TIMER

Modela un periférico Timer/Counter.

Modos de funcionamiento:


     
- TIMER_TICKER. A Ticker is used to call a function at a recurring interval 

- TIMER_OVERFLOW =? COUNT_TO_OVERFLOW =? TIMER_NORMAL. Modo temporizador de propósito general. Este modo es utilizado cuando se requiere de temporizaciones precisas. Cuenta hasta overflow.
    - Continuous normal operation (Reset timer counter on overflow) /|/|/|
    - Continuous inverter operation (invert timer counter mode on overflow) /\/\/\ --> Solo si soporta conteo up-down
    - Stop timer counter on overflow  /|___

- TIMER_MATCH. Modo temporizador de propósito general. Este modo es utilizado cuando se requiere de temporizaciones precisas. Cuenta hasta llegar a cierto valor de comparación.
    - Reset timer counter on match /|/|/|
    - Invert timer counter mode on match /\/\/\ --> Solo si soporta conteo up-down
    - Stop timer counter on match  /|___

   - TIMER_MATCH_OUTPUT =? TIMER_OUTPUT_SIGNAL_GENERATOR =? TIMER_WAVEFORM_GENERATOR: Modo de generación de pulsos de un determinado ancho o señal periódica de una determinada frecuencia.
       - TIMER_SET_OUTPUT_ON_MATCH (Set high on match)
       - TIMER_CLEAR_OUTPUT_ON_MATCH (Set low on match)
       - TIMER_TOGGLESET_OUTPUT_ON_MATCH (Toggle on match)

   - TIMER_PWM: Generación de señal de salida PWM (modulación de ancho de pulso). En modo PWM hay que usar los 2 valores, el match y el overflow para cambiar el pin.
       - TIMER_PWM_EDGE (alineado al flanco =? Fast PWM. Inicia con un flanco de subida o termia con un flanco de bajada, desde el punto de vista del ciclo de trabajo (Duty Cycle).
           - TIMER_PWM_LOW_ON_COMPARE (comienza en alto y cae cuando llega a COMPARE)
                 - Non-inverted Clear OCO on compare match, set OCO at TOP
           - TIMER_PWM_HIGH_ON_COMPARE (comienza en bajo y sube cuando llega a COMPARE)
                 - Inverted PWM Set OCO on compare match, clear OCO at TOP
       - TIMER_PWM_CENTER (alineado al centro) =? Fase correcta. Alineado al centro del período T. Esta última característica es muy popular para el control de servo motores de 3 fases en CA y sin escobillas (brushless) en CD, en donde son necesarios  varios  canales  de  PWM.
           - TIMER_PWM_LOW_ON_COMPARE (comienza en alto y cae cuando llega a COMPARE)
           - TIMER_PWM_HIGH_ON_COMPARE (comienza en bajo y sube cuando llega a COMPARE)


- TIMER_INPUT_CAPTURE: Modo de captura de eventos externos. En este modo el timer mide eventos temporales externos, aplicados a ciertos pines. Estos eventos pueden ser: medición de ancho de un pulso o la frecuencia de una señal.
    - CAPTURE IN RISING EDGE
    - CAPTURE IN FALLING EDGE
    - CAPTURE IN BOTH EDGES

    - Continuous normal operation (Reset timer counter on overflow) /|/|/|
    - Continuous inverter operation (invert timer counter mode on overflow) /\/\/\ --> Solo si soporta conteo up-down
    - Reset timer on capture /|/|/| 
    - Invert timer on capture /\/\/\ --> Solo si soporta conteo up-down
    - Stop timer on capture  /|___



timer capture input
timer match outputs



Propiedades:

- clockSource (external pin, F_CPU)
- prescaler (clockSource/1, /2, /4, /8, /16, /32, /64, /128, /256, /512, /1024 )
    - Entre estos 2 se calcula:
        - frequency
        - period

- counterSize (tamaño del contador del timer, 8, 16, 24 o 32 bits)
- counterMode (up, down o up-down(lo usa modo PWM alineado al centro) )
- counterState (estado, running o stop)
- counterValue (valor actual de conteo)

- counterCompareValue (valor de comparacion, puede indicar hasta donde cuenta o desde donde cuenta).
    - Con este valor se calcula el dutyCycle (cambia la formula segun el tipo de PWM)

Eventos:

- timerOverflowEvent (marca evento de rebalse a 0, ejecuta un callback, ojo que en modo up-down no marca cuando llega al máximo).
- timerCompareMatchEvent (marca evento de alcanzar el valor de comaración, ejecuta un callback). Puede tener más de un evento de match por timer (4 en LPC4337 TIMER, 16 en el SCT).
- timerInputCaptureEvent (marca el evento del pin, guarda el valor actual de counter y ejecuta un callback). Puede tener más de un evento de capture por timer (4 en LPC4337 TIMER, 8 en el SCT).

- Cuando ocurre un evento además de lo que hace se puede:
    - Continuous normal operation (no hace nada, NO VALIDO PARA EVENTO OVERFLOW)
    - Reset timer counter on event /|/|/| 
    - Invert timer counter mode on event /\/\/\ --> Solo si soporta conteo up-down
    - Stop timer on event  /|___




Timeout:

The Timeout interface is used to setup an interrupt to call a function after a specified delay.

**Métodos:**

Timer:

- start
- stop
- reset
- readCounter



### PWM


- frequency
- dutyCycle

Manejo de salidas PWM (modulación por ancho de pulso). En la EDU-CIAA-NXP se
utiliza internamente el periférico SCT para generar los PWM.

**Configuración**

``bool_t pwmConfig( pwmMap_t pwmNumber, pwmConfig_t config);``

- Parámetro: ``pwmMap_t pwmNumber`` pin a configurar como salida PWM (ver PWM Map).
- Parámetro: ``uint8_t config`` configuración.
- Retorna: ``bool_t`` TRUE si la configuración es correcta.

Posibles configuraciones:

- ``PWM_ENABLE`` habilita el o los Timers en modo PWM.
- ``PWM_DISABLES`` deshabilita el o los Timers en modo PWM.
- ``PWM_ENABLE_OUTPUT`` habilita la salida PWM particular.
- ``PWM_DISABLE_OUTPUT`` deshabilita la salida PWM particular.

**Lectura del ciclo de trabajo (duty cycle) de la salida PWM**

``uint8_t pwmRead( pwmMap_t pwmNumber );``

- Parámetro: ``pwmMap_t pwmNumber`` salida PWM a leer el ciclo de trabajo.
- Retorna: ``uint8_t`` el ciclo de trabajo de la salida PWM.

 **Establecer el ciclo de trabajo de la salida PWM**

``bool_t pwmWrite( pwmMap_t pwmNumber, uint8_t percent );``

- Parámetro: ``pwmMap_t pwmNumber`` salida PWM a leer el ciclo de trabajo.
- Parámetro: ``uint8_t percent`` valor de ciclo de trabajo a setear en la salida PWM.
- Retorna: ``bool_t`` TRUE.




## RTC

Manejo del periférico RTC (reloj de tiempo real).

**Configuración**

``bool_t rtcConfig( rtc_t * rtc );``

- Parámetro: ``rtc_t * rtc`` Puntero a estructura de configuración del tipo RTC_t.
- Retorna: ``bool_t`` TRUE si la configuración es correcta.

La estructura del tipo ``RTC_t`` contiene los parámetros:

- ``uint16_t year`` año, con valores desde 1 a 4095.
- ``uint8_t month`` mes, con valores desde 1 a 12.
- ``uint8_t mday`` día, con valores desde 1 a 31.
- ``uint8_t wday`` día de la semana, con valores desde 1 a 7.
- ``uint8_t hour`` horas, con valores desde 0 a 23.
- ``uint8_t min`` minutos, con valores desde 0 a 59.
- ``uint8_t sec`` segundos, con valores desde 0 a 59.

**Lectura de fecha y hora**

``bool_t rtcRead( rtc_t * rtc );``

- Parámetro: ``rtc_t * rtc`` Puntero a estructura del tipo RTC_t donde se guarda la fecha y hora.
- Retorna: ``bool_t`` TRUE.

**Establecer la fecha y hora**

``bool_t rtcWrite( rtc_t * rtc );``

- Parámetro: ``rtc_t * rtc`` Puntero a estructura del tipo RTC_t con la nueva fecha y hora a setear.
- Retorna: ``bool_t`` TRUE.



## UART

Modela periférico de comunicación UART (transmisor/receptor asincrónico universal), coumnmente llamado *puerto serie*.

Modos de utilización de UART

- Bloqueante (esperar trasmisor libre o recepción por polling).
- No bloqueante (interrupción).


### Propiedades de UART

**Propiedades de configuración**

- ``baudRate`` (tipo ``uartConfig_t``). Valores posibles:
    - ``1200``, ``2400``, ``4800``, ``9600`` *(valor por defecto)*, ``19200``, ``38400``, ``57600`` o ``115200``.
- ``dataBits`` (tipo ``uartConfig_t``). Valores posibles:
    - ``5``, ``6``, ``7``, ``8`` *(valor por defecto)* o ``9``.
- ``stopBits`` (tipo ``uartConfig_t``). Valores posibles:
    - ``1`` *(valor por defecto)* o ``2``.
- ``parity`` (tipo ``uartConfig_t``). Valores posibles:
    - ``UART_PARITY_NONE`` *(valor por defecto)*, ``UART_PARITY_ODD`` o ``UART_PARITY_EVEN``.
- ``flowControl`` (tipo ``uartConfig_t``). Valores posibles:
    - ``UART_FLOWCONTROL_NONE`` *(valor por defecto)*, ``UART_FLOWCONTROL_CTS``, ``UART_FLOWCONTROL_RTS``, ``UART_FLOWCONTROL_RTS_CTS``.
- ``power`` (tipo ``uartConfig_t``). Valores posibles:
    - ``ON``, ``OFF`` *(valor por defecto)*, ``ENABLE`` o ``DISABLE``

**Propiedades de valor**

- ``txValue`` (tipo ``uint8_t``). Valores posibles:
    - Un valor del tipo ``uint8_t`` que representa el un byte a transmitir.
- ``rxValue``*{solo lectura}* (tipo ``uint8_t``). Valores posibles:
    - Un valor del tipo ``uint8_t`` que representa el último byte recibido.
- ``timeout`` (tipo ``tick_t``). Valores posibles:
    - Un valor del tipo ``tick_t`` que representa milisegundos. Este parámetro es para transmisión y recepción bloqueante. Se ignora para transmisión y recepción por interrupción.

**Propiedades de eventos por polling o interrupción**

*(TODO: Hacer ejemplos de uso de eventos por polling e interrupción para ver cuales deben tenerse como propiedades)*

Los siguientes son del tipo ``uartConfig_t``:

- ``byteReceive``
- ``byteSendComplete``
- ``stringSendComplete``
- ``stringSendingingTimeout``
- ``stringReceivedComplete``
- ``stringReceivingTimeout``

Valores posibles:

- ``DISABLE`` (valor por defecto) o ``ENABLE``

Los siguientes son del tipo ``uartEventFlag_t``:

- ``byteReceive``
- ``byteSendComplete``
- ``stringSendComplete``
- ``stringSendingingTimeout``
- ``stringReceivedComplete``
- ``stringReceivingTimeout``

Valores posibles:

- ``DISABLE`` (valor por defecto) o ``ENABLE``

Los siguientes son del tipo ``Callback_t``.

- ``byteSendCompleteInterruptCallback`` 
- ``byteReceiveInterruptCallback``
- ``stringReceivedCompleteCallback``
- ``stringSendingTimeoutCallback``
- ``stringSendCompleteCallback``
- ``stringReceivingTimeoutCallback``

Valores posibles:

- Una estructura con el puentero a función y el puntero al parámetro que le pueda pasar el usuario a dicha función. *(valor por defecto)* un null pointer con un while(1) *(TODO)*


### Métodos de UART

- Getters y Setters de sus propiedades.

- Initialize: 

```c
void uartInitialize( 
                     UART<i>, 
                     UART_BAUDRATE(b) | UART_DATABITS(d) | UART_STOPBITS(s) | 
                     UART_PARITY(p) | UART_FLOWCONTROL(f) | UART_TIMEOUT(t) 
                   );

/*
Con:
 - b = un valor de baudRate.
 - d = un valor de dataBits.
 - s = un valor de stopBits.
 - p = un valor de parity.
 - f = un valor de flowControl.
 - t = un valor de timeout en ms.
*/
```

- Métodos  para utilizando buffers:
    - ``uartSend( UART<i>, uint8_t* buffer, bufferSize );``
    - ``uartReceive( UART<i>, uint8_t* buffer, bufferSize );``

- Métodos tipados:

    - ``uartConfig();`` alias de ``uartInitialize();``
    - ``uartSendByte();`` usa ``uartGetRxValue();``
    - ``uartReceiveByte();`` usa ``gpioSetTxValue();``
    - ``uartSendString();``
    - ``uartReceiveString();``

*(TODO: Agregar métodos en UART para chequear explicitamente eventos por pooling), si se dan, ejecutar el callback*



## SPI






## I2C

Manejo del periférico bus comunicación I2C (Inter Integrated Circuits).

**Configuración**

``bool_t i2cConfig( i2cMap_t i2cNumber, uint32_t clockRateHz );``

- Parámetro: ``i2cMap_t i2cNumber`` I2C a configurar (ver I2C Map).
- Parámetro: ``uint32_t clockRateHz`` configuración de velocidad del bus I2C.
- Retorna: ``bool_t`` TRUE si la configuración es correcta.

Posibles configuraciones de clockRateHz: 100000, etc.

**Lectura**
```c
bool_t i2cRead( i2cMap_t  i2cNumber,
                uint8_t  i2cSlaveAddress,
                uint8_t* dataToReadBuffer,
                uint16_t dataToReadBufferSize,
                bool_t   sendWriteStop,
                uint8_t* receiveDataBuffer,
                uint16_t receiveDataBufferSize,
                bool_t   sendReadStop );
```

- Parámetro: ``i2cMap_t i2cNumber`` I2C a leer (ver I2C Map).
- Parámetro: ``uint8_t i2cSlaveAddress`` Dirección del sensor conectado por I2C a leer.
- Parámetro: ``uint8_t* dataToReadBuffer`` puntero al buffer con los bytes a escribir para indicar que se debe leer.
- Parámetro: ``uint16_t dataToReadBufferSize`` tamaño del buffer con los bytes a escribir.
- Parámetro: ``bool_t sendWriteStop`` setear en 1 para enviar stop al finalizar el comando de escritura, con 0 no se envía. Algunos periféricos pueden no necesitar el stop.
- Parámetro: ``uint8_t* receiveDataBuffer`` puntero al buffer donde se almacenarán los datos leídos.
- Parámetro: ``uint16_t receiveDataBufferSize`` tamaño del buffer donde se almacenarán los datos leídos.
- Parámetro: ``bool_t sendReadStop`` setear en 1 para enviar stop al finalizar el comando de lectura, con 0 no se envía. Algunos periféricos pueden no necesitar el stop.
- Retorna: ``bool_t`` TRUE si se pudo leer correctamente.

**Escritura**

```c
bool_t i2cWrite( i2cMap_t  i2cNumber,
                 uint8_t  i2cSlaveAddress,
                 uint8_t* transmitDataBuffer,
                 uint16_t transmitDataBufferSize,
                 bool_t   sendWriteStop );
```

- Parámetro: ``i2cMap_t i2cNumber`` ID de periférico I2C a escribir (ver I2C Map). Actualmente funciona únicamente el I2C0.
- Parámetro: ``uint8_t i2cSlaveAddress`` Dirección del sensor conectado por I2C a escribir.
- Parámetro: ``uint8_t* transmitDataBuffer`` puntero al buffer donde se encuentran los datos a escribir.
- Parámetro: ``uint16_t transmitDataBufferSize`` tamaño del buffer donde se encuentran los datos a escribir.
- Parámetro: ``bool_t sendWriteStop`` setear en 1 para enviar stop al finalizar el comando de escritura, con 0 no se envía. Algunos periféricos pueden no necesitar el stop.
- Retorna: ``bool_t`` TRUE si se pudo escribir correctamente.




## Power management

Manejo de modos de consumo del microcontrolador.

**Dormir hasta que ocurra la próxima interrupción**

``void sleepUntilNextInterrupt( void );``

- Parámetro: ninguno.
- Retorna: nada.e

Ver: http://docs.micropython.org/en/latest/pyboard/library/pyb.html#reset-related-functions




## Archivos que contienen estos módulos

**inc** (.h):

- sapi_core.h
- sapi_gpio.h
- sapi_adc.h
- sapi_dac.h
- sapi_timer.h
- sapi_rtc.h
- sapi_uart.h
- sapi_spi.h
- sapi_i2c.h
- sapi_power.h

**src** (.c):

- sapi_core.c
- sapi_gpio.c
- sapi_adc.c
- sapi_dac.c
- sapi_timer.c
- sapi_rtc.c
- sapi_uart.c
- sapi_spi.c
- sapi_i2c.c
- sapi_power.c

