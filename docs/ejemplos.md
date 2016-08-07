# Ejemplos

## EDU-CIAA-NXP

**Periféricos internos**

- Tipos de datos.
- Mapa de periféricos.
- Vector de ISRs.
- Plataforma.
- Tick.
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

### ej1_sAPI_tecsLeds

Este ejemplo maneja un led con cada tecla de la EDU-CIAA-NXP.

### ej2_sAPI_blinky

Este ejemplo hace titilar un led (blink) de la EDU-CIAA-NXP utilizando un
retardo bloqueante.

### ej3_sAPI_semaforo

Este ejemplo se emula un semáforo con los leds de la EDU-CIAA-NXP utilizando un
retardo bloqueante.

### ej4_sAPI_blinky_tecla

Este ejemplo hace titilar un led (blink) de la EDU-CIAA-NXP utilizando un
retardo no bloqueante. Esto permite manejar otro led con la TEC4 al mismo tiempo
que parpadea.

### ej5_sAPI_semaforo_tecla

Este ejemplo se emula un semáforo con los leds de la EDU-CIAA-NXP utilizando un
retardo no bloqueante. Esto permite manejar otro led con la TEC4 al mismo tiempo
que funciona el semáforo.

### ej6_sAPI_secuencias

Con TEC1 y TEC 4 maneja la dirección de movimiento de los leds, mientras que con
TEC2 y TEC4 cabia la velocidad de movimiento de lenta a rápida en la
EDU-CIAA-NXP. Utiliza retardo no bloqueante y cambia su valor en ejecución.
