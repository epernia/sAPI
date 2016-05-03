
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
