# Ejemplos

## EDU-CIAA-NXP

### Periféricos internos

**ej01_sapi_bm_tecs_leds**

Ejemplo de utilización de DigitalIO. Este ejemplo enciende un led con cada tecla
de la EDU-CIAA-NXP.

**ej02_sapi_bm_blinky**

Ejemplo de utilización de DigitalIO, Tick y Delay. Este ejemplo hace titilar un
led (blink) de la EDU-CIAA-NXP utilizando un retardo bloqueante (función ``delay()``). Un retardo bloqueante no permite que se ejecuten otras tareas
mientras se está ejecutando.

**ej03_sapi_bm_blinky_tecla**

Ejemplo de utilización de DigitalIO, Tick y Delay. Este ejemplo hace titilar un
led (blink) de la EDU-CIAA-NXP utilizando un retardo NO bloqueante (funciónes
``delayConfig()``, ``delayRead()``). Esto permite
manejar otro led con la TEC4 al mismo tiempo que parpadea.

**ej04_sapi_bm_secuencias**

Ejemplo de utilización de DigitalIO, Tick y Delay. Con TEC1 y TEC 4 maneja la
dirección de movimiento de los leds (secuencia de leds a izquierda o derecha),
mientras que con TEC2 y TEC4 cabia la velocidad de movimiento de lenta a rápida
en la EDU-CIAA-NXP. Utiliza retardo no bloqueante (funciónes ``delayConfig()``,
``delayRead()``) y cambia su valor en ejecución (función ``delayWrite()``).

**ej05_sapi_bm_tickHook**

Ejemplo de utilización de DigitalIO y Tick. Este ejemplo utiliza una función
que se ejecuta cada vez que ocurre un tick para cambiar el valor del LED azul
de la EDU-CIAA-NXP. Confgura la tasa de ticks a 1 tick cada 50ms (velocidad más
lenta soportada).

**ej06_sapi_uart_eco**

Ejemplo de utilización de UART. Al iniciar envía varios caracteres y Strings y
luego realiza el eco de todo lo que recibe enviándolo nuevamente por la
UART_USB.

**ej07_sapi_adc_dac**

Ejemplo de utilización de ADC y DAC. También utiliza DigitalIO, Tick , Delay y
UART. Utiliza dos retardos NO bloqueantes para realizar dos tareas periódicas
sin interferencias. La primera lee el ADC CH1 (AI0), escribe el valor en el DAC
(AO) y envía el dato por la UART_USB. La segunda hace titilar el LED1 10 veces
rápidamente y después cambia el valor del retardo para que titile lentamente.

**ej08_sapi_rtc**

Ejemplo de utilización de RTC. También utiliza Delay y UART. Establece la fecha
y hora del reloj de tiempo real y la muestra por UART_USB.

**ej09_sapi_pwm**

Ejemplo de utilización de PWM. <<<<<<<<<<<<<<<

**ej10_sapi_servo**

Ejemplo de utilización de Servomotor angular (0 a 180°). <<<<<<<<<<<<<<<

**ej11_sapi_hmc5883l**

Ejemplo de utilización de sensor Magnetómetro (compass) HMC5883L, I2C y Uart.
Este sensor se conecta a la EDU-CIAA-NXP mediante I2C para leer los valores de
campo magnético en (x,y,z) y los muestra por UART.  <<<<<<<<<<<<<<<
