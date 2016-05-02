# sAPI library for microcontrollers

This library implements a simple API that acts as a HAL (Hardware Abstraction
Layer) for microcontrollers. T

Takes ideas from *Wiring library*, but use the concept of *peripheral* instead
the concept of *pin*, making the API regardless of the number of pins that use
certain peripheral.

## Documentation

### Included modules

- Data types.
- Peripheral Map.
- Board.
- Tick.
- Delay.
- Digital I/O.
- Analog I/O.
- Uart.

### Software layers

![ "sapi-modulos-capas.png" image not found](docs/assets/img/sapi-modulos-capas.png "Modules an layers of sAPI library")

### Boards

Now available for boards:

- EDU-CIAA-NXP (NXP LPC4337 microcontroller). [Download documentation.](docs/assets/pdf/EDU-CIAA-NXP_sAPI_bm_A4_v1r0_ES.pdf)
- CIAA-NXP (NXP LPC4337 microcontroller).
