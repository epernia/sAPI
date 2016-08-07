# Introducción a la biblioteca para microcontroladores sAPI

Esta biblioteca implementa una API simple para la programación de
microcontroladores.

## Motivación

La misma surge de la necesidad de manejar los periféricos directamente desde una
VM de Java para el desarrollo de Java sobre la CIAA y corresponde a la parte de
bajo nivel de las clases de periféricos en Java que básicamente bindea a
funciones escritas en C.

Luego se extendió la misma para facilitar el uso de la EDU-CIAA-NXP a personas
no expertas en la arquitectura del LPC4337 facilitando el uso de esta plataforma.

## Módulos incluidos

**Periféricos internos**

- Tipos de datos.
- Mapa de periféricos.
- Plataforma.
- Tick.
- Vector de ISRs.
- Retardo.
- E/S Digital.
- E/S Analógica.
- UART.
- I2C.
- RTC.
- PWM.

**Periféricos externos**

- Servo angular (0 a 180°).
- Sensor magnetómetro (compass) HMC5883L.
- Módulo WiFi ESP8266.

Cada módulo incluye un ejemplo.

## Plataformas

Actualmente disponible para las plataformas:

- EDU-CIAA-NXP (microcontrolador NXP LPC4337). [Descargar mapeo de periféricos.](assets/pdf/EDU-CIAA-NXP_sAPI_bm_A4_v1r0_ES.pdf)
- CIAA-NXP (microcontrolador NXP LPC4337).
