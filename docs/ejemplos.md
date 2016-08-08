# Ejemplos

## EDU-CIAA-NXP

### Periféricos internos

**ej1_sapi_bm_tecs_leds**

Ejemplo de utilización de DigitalIO. Este ejemplo enciende un led con cada tecla
de la EDU-CIAA-NXP.

**ej2_sapi_bm_blinky**

Ejemplo de utilización de DigitalIO, Tick y Delay. Este ejemplo hace titilar un
led (blink) de la EDU-CIAA-NXP utilizando un retardo bloqueante (función ``delay()``). Un retardo bloqueante no permite que se ejecuten otras tareas
mientras se está ejecutando.

**ej3_sapi_bm_blinky_tecla**

Ejemplo de utilización de DigitalIO, Tick y Delay. Este ejemplo hace titilar un
led (blink) de la EDU-CIAA-NXP utilizando un retardo NO bloqueante (funciónes
``delayConfig()``, ``delayRead()``). Esto permite
manejar otro led con la TEC4 al mismo tiempo que parpadea.

**ej4_sapi_bm_secuencias**

Ejemplo de utilización de DigitalIO, Tick y Delay. Con TEC1 y TEC 4 maneja la
dirección de movimiento de los leds (secuencia de leds a izquierda o derecha),
mientras que con TEC2 y TEC4 cabia la velocidad de movimiento de lenta a rápida
en la EDU-CIAA-NXP. Utiliza retardo no bloqueante (funciónes ``delayConfig()``,
``delayRead()``) y cambia su valor en ejecución (función ``delayWrite()``).

**ej5_sapi_bm_tick**

Ejemplo de utilización de DigitalIO y Tick. Este ejemplo utiliza una función
que se ejecuta cada vez que ocurre un tick para cambiar el valor del LED azul
de la EDU-CIAA-NXP. Confgura la tasa de ticks a 1 tick cada 50ms (velocidad más
lenta soportada).



- UART.
- E/S Analógica.
- RTC.
- PWM.
- I2C.

### Periféricos externos


- Servo angular (0 a 180°).
- Sensor magnetómetro (compass) HMC5883L.
