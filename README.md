# sAPI library for microcontrollers

This library implements a simple API that acts as a HAL (Hardware Abstraction
Layer) for microcontrollers.

# NOTE: OLD VERSION!!! USE [FIRMWARE V3](https://github.com/epernia/firmware_v3)!!!

## Included modules

**Internal Peripherals**

- Data types.
- Peripheral Map.
- ISR Vector.
- Board.
- Tick.
- GPIO.
- UART.
- ADC.
- DAC.
- I2C.
- SPI.
- RTC.
- SCT.
- Timer.
- Sleep.

**High Level modules**

- Delay. Use Tick module.
- PWM. Use SCT and GPIO modules.
- Circular Buffer.
- Convert.
- Print. Use UART and Convert modules.
- Debug Print. Use Print module.
- Console Print. Use Print module.

**External Peripherals using sAPI**

- 7-segment display. Use GPIO and Delay modules.
- Keypad. Use GPIO and Delay modules.
- Angular Servo (0 to 180°). Use Timer and GPIO modules.
- Magnetometer (compass) sensor HMC5883L. Use I2C module.

Every module includes an example.

## Boards

Available for boards:

- EDU-CIAA-NXP (NXP LPC4337 microcontroller). [Download documentation.](docs/assets/pdf/EDU-CIAA-NXP_sAPI_bm_A4_v1r0_ES.pdf)
- CIAA-NXP (NXP LPC4337 microcontroller).
