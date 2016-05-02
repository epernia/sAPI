# Biblioteca para microcontroladores sAPI

Esta biblioteca implementa una API simple para la programación de
microcontroladores que actúa como HAL (Hardware Abstraction
Layer).

Toma ideas de la biblioteca *Wiring*, pero en lugar del concepto de *pin*
utiliza el concepto *periférico*, logrando una API unificada sin importar el
número de pines que tenga un cierto perifécico.

### Módulos incluidos

- Tipos de datos.
- Mapa de periféricos.
- Plataforma.
- Tick.
- Retardo.
- E/S Digital.
- E/S Analógica.
- Uart.

### Capas de software

![Imagen "sapi-modulos-capas.png" no encontrada](assets/img/sapi-modulos-capas.png "Módulos y capas de la biblioteca sAPI")

### Plataformas

Actualmente disponible para las plataformas:

- EDU-CIAA-NXP (microcontrolador NXP LPC4337). [Descargar documentación.](assets/pdf/EDU-CIAA-NXP_sAPI_bm_A4_v1r0_ES.pdf)
- CIAA-NXP (microcontrolador NXP LPC4337).
