/* Copyright 2015-2017, Eric Pernia.
 * All rights reserved.
 *
* This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2015-09-23 */

/*==================[inclusions]=============================================*/

#include "sapi_gpio.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/
/*
#define GPIO_MODE_MASK    0x7F
#define GPIO_SPEED_MASK   0x380
#define GPIO_EVENT_MASK   0x7C00
#define GPIO_POWER_MASK   0x8000

#define GPIO_SET_CONFIG_PROPERTY( gpioName, property , mask ) gpioInstances[(gpioName)].config = (gpioInstances[(gpioName)].config & ~(mask)) | ((property) & (mask)) 

#define GPIO_GET_CONFIG_PROPERTY( gpioName, mask ) gpioInstances[(gpioName)].config & (mask)
*/
/* ----- Begin Pin Config Structs NXP LPC4337 ----- */

#define LPC4337_GPIO_INPUT    0
#define LPC4337_GPIO_OUTPUT   1

typedef struct{
   int8_t port;
   int8_t pin;
} gpioConfigLpc4337_t;

typedef struct{
    pinConfigLpc4337_t pinName;
                int8_t func;
   gpioConfigLpc4337_t gpio;
} pinConfigGpioLpc4337_t;

/* ------ End Pin Config Structs NXP LPC4337 ------ */

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

const pinConfigGpioLpc4337_t gpioPinsConfig[] = {

   /*{ {PinNamePortN ,PinNamePinN}, PinFUNC, {GpioPortN, GpioPinN} }*/

   /* --------------------------------------------------------------- */
   /*                           EDU-CIAA-NXP                          */
   /* --------------------------------------------------------------- */
   /*                             Snap  sAPI   Connector  Serigraphy  */
   /* --------------------------------------------------------------- */

   // { {1,15}, FUNC0, {0, 2} },   /*  0   DIO0    CON2_09   ENET_RXD0   */

   // { {1, 4}, FUNC0, {0,11} },   /*  1   DIO1    CON2_21   SPI_MOSI    */
   // { {4, 9}, FUNC4, {5,13} },   /*  2   DIO2    CON2_23   LCD_EN      */

   // { {6, 1}, FUNC0, {3, 0} },   /*  3   DIO3    CON2_29   GPIO0       */
   // { {6, 5}, FUNC0, {3, 4} },   /*  4   DIO4    CON2_31   GPIO2       */
   // { {6, 8}, FUNC4, {5,16} },   /*  5   DIO5    CON2_33   GPIO4       */
   // { {6,10}, FUNC0, {3, 6} },   /*  6   DIO6    CON2_35   GPIO6       */

   // { {0, 0}, FUNC0, {0, 0} },   /*  7   DIO7    CON2_04   ENET_RXD1   */
   // { {0, 1}, FUNC0, {0, 1} },   /*  8   DIO8    CON2_06   ENET_TXEN   */
   // { {7, 7}, FUNC0, {3,15} },   /*  9   DIO9    CON2_08   ENET_MDC    */
   // { {1,16}, FUNC0, {0, 3} },   /* 10   DIO10   CON2_10   ENET_CRS_DV */
   // { {1,17}, FUNC0, {0,12} },   /* 11   DIO11   CON2_12   ENET_MDIO   */
   // { {1,18}, FUNC0, {0,13} },   /* 12   DIO12   CON2_14   ENET_TXD0   */
   // { {1,20}, FUNC0, {0,15} },   /* 13   DIO13   CON2_16   ENET_TXD1   */
   // { {1, 3}, FUNC0, {0,10} },   /* 14   DIO14   CON2_18   SPI_MISO    */

   // { {4,10}, FUNC4, {5,14} },   /* 15   DIO15   CON2_22   LCD4        */
   // { {4, 8}, FUNC4, {5,12} },   /* 16   DIO16   CON2_24   LCDRS       */
   // { {4, 6}, FUNC0, {2, 6} },   /* 17   DIO17   CON2_26   LCD3        */
   // { {4, 5}, FUNC0, {2, 5} },   /* 18   DIO18   CON2_28   LCD2        */
   // { {4, 4}, FUNC0, {2, 4} },   /* 19   DIO19   CON2_30   LCD1        */

   // { {6, 4}, FUNC0, {3, 3} },   /* 20   DIO20   CON2_32   GPIO1       */
   // { {6, 7}, FUNC4, {5,15} },   /* 21   DIO21   CON2_34   GPIO3       */
   // { {6, 9}, FUNC0, {3, 5} },   /* 22   DIO22   CON2_36   GPIO5       */
   // { {6,11}, FUNC0, {3, 7} },   /* 23   DIO23   CON2_38   GPIO7       */
   // { {6,12}, FUNC0, {2, 8} },   /* 24   DIO24   CON2_40   GPIO8       */

   // { {2, 4}, FUNC4, {5, 4} },   /* 25   DIO25   CON1_23   RS232_RXD   */
   // { {2, 3}, FUNC4, {5, 3} },   /* 26   DIO26   CON1_25   RS232_TXD   */
   // { {3, 1}, FUNC4, {5, 8} },   /* 27   DIO27   CON1_27   CAN_RD      */
   // { {3, 2}, FUNC4, {5, 9} },   /* 28   DIO28   CON1_29   CAN_TD      */
   // { {7, 4}, FUNC0, {3,12} },   /* 29   DIO29   CON1_31   T_COL1      */
   // { {4, 0}, FUNC0, {2, 0} },   /* 30   DIO30   CON1_33   T_FIL0      */
   // { {4, 3}, FUNC0, {2, 3} },   /* 31   DIO31   CON1_35   T_FIL3      */
   // { {4, 2}, FUNC0, {2, 2} },   /* 32   DIO32   CON1_37   T_FIL2      */
   // { {1, 5}, FUNC0, {1, 8} },   /* 33   DIO33   CON1_39   T_COL0      */

   // { {7, 5}, FUNC0, {3,13} },   /* 34   DIO34   CON1_34   T_COL2      */
   // { {4, 1}, FUNC0, {2, 1} },   /* 35   DIO35   CON1_36   T_FIL1      */


   { {4, 1}, FUNC0, {2, 1} },   /*   0   CON1_36   T_FIL1           */
   { {7, 5}, FUNC0, {3,13} },   /*   1   CON1_34   T_COL2           */

   { {1, 5}, FUNC0, {1, 8} },   /*   2   CON1_39   T_COL0           */
   { {4, 2}, FUNC0, {2, 2} },   /*   3   CON1_37   T_FIL2           */
   { {4, 3}, FUNC0, {2, 3} },   /*   4   CON1_35   T_FIL3           */
   { {4, 0}, FUNC0, {2, 0} },   /*   5   CON1_33   T_FIL0           */
   { {7, 4}, FUNC0, {3,12} },   /*   6   CON1_31   T_COL1           */

   { {3, 2}, FUNC4, {5, 9} },   /*   7   CON1_29   CAN_TD           */
   { {3, 1}, FUNC4, {5, 8} },   /*   8   CON1_27   CAN_RD           */

   { {2, 3}, FUNC4, {5, 3} },   /*   9   CON1_25   RS232_TXD        */
   { {2, 4}, FUNC4, {5, 4} },   /*  10   CON1_23   RS232_RXD        */

   { {6,12}, FUNC0, {2, 8} },   /*  11   CON2_40   GPIO8            */
   { {6,11}, FUNC0, {3, 7} },   /*  12   CON2_38   GPIO7            */
   { {6, 9}, FUNC0, {3, 5} },   /*  13   CON2_36   GPIO5            */
   { {6, 7}, FUNC4, {5,15} },   /*  14   CON2_34   GPIO3            */
   { {6, 4}, FUNC0, {3, 3} },   /*  15   CON2_32   GPIO1            */

   { {4, 4}, FUNC0, {2, 4} },   /*  16   CON2_30   LCD1             */
   { {4, 5}, FUNC0, {2, 5} },   /*  17   CON2_28   LCD2             */
   { {4, 6}, FUNC0, {2, 6} },   /*  18   CON2_26   LCD3             */
   { {4, 8}, FUNC4, {5,12} },   /*  19   CON2_24   LCDRS            */
   { {4,10}, FUNC4, {5,14} },   /*  20   CON2_22   LCD4             */

   { {1, 3}, FUNC0, {0,10} },   /*  21   CON2_18   SPI_MISO         */

   { {1,20}, FUNC0, {0,15} },   /*  22   CON2_16   ENET_TXD1        */
   { {1,18}, FUNC0, {0,13} },   /*  23   CON2_14   ENET_TXD0        */
   { {1,17}, FUNC0, {0,12} },   /*  24   CON2_12   ENET_MDIO        */
   { {1,16}, FUNC0, {0, 3} },   /*  25   CON2_10   ENET_CRS_DV      */
   { {7, 7}, FUNC0, {3,15} },   /*  26   CON2_08   ENET_MDC         */
   { {0, 1}, FUNC0, {0, 1} },   /*  27   CON2_06   ENET_TXEN        */
   { {0, 0}, FUNC0, {0, 0} },   /*  28   CON2_04   ENET_RXD1        */

   { {6,10}, FUNC0, {3, 6} },   /*  29   CON2_35   GPIO6            */
   { {6, 8}, FUNC4, {5,16} },   /*  30   CON2_33   GPIO4            */
   { {6, 5}, FUNC0, {3, 4} },   /*  31   CON2_31   GPIO2            */
   { {6, 1}, FUNC0, {3, 0} },   /*  32   CON2_29   GPIO0            */

   { {4, 9}, FUNC4, {5,13} },   /*  33   CON2_23   LCDEN            */

   { {1, 4}, FUNC0, {0,11} },   /*  34   CON2_21   SPI_MOSI         */

   { {1,15}, FUNC0, {0, 2} },   /*  35   CON2_09   ENET_RXD0        */


   { {1, 0}, FUNC0, {0, 4} },   /* 36   TEC1    TEC_1                 */
   { {1, 1}, FUNC0, {0, 8} },   /* 37   TEC2    TEC_2                 */
   { {1, 2}, FUNC0, {0, 9} },   /* 38   TEC3    TEC_3                 */
   { {1, 6}, FUNC0, {1, 9} },   /* 39   TEC4    TEC_4                 */

   { {2, 0}, FUNC4, {5, 0} },   /* 43   LEDR    LED0_R                */
   { {2, 1}, FUNC4, {5, 1} },   /* 44   LEDG    LED0_G                */
   { {2, 2}, FUNC4, {5, 2} },   /* 45   LEDB    LED0_B                */
   { {2,10}, FUNC0, {0,14} },   /* 40   LED1    LED1                  */
   { {2,11}, FUNC0, {1,11} },   /* 41   LED2    LED2                  */
   { {2,12}, FUNC0, {1,12} },   /* 42   LED3    LED3                  */

   /* --------------------------------------------------------------- */
	/*                             CIAA-NXP                            */
   /* --------------------------------------------------------------- */
   /*                             Snap  sAPI   Connector  Serigraphy  */
   /* --------------------------------------------------------------- */

   { {4, 0}, FUNC0, {2, 0} },   /* 46   DI0     BORN_24   DIN0        */
   { {4, 1}, FUNC0, {2, 1} },   /* 47   DI1     BORN_25   DIN1        */
   { {4, 2}, FUNC0, {2, 2} },   /* 48   DI2     BORN_26   DIN2        */
   { {4, 3}, FUNC0, {2, 3} },   /* 49   DI3     BORN_27   DIN3        */
   { {7, 3}, FUNC0, {3,11} },   /* 50   DI4     BORN_28   DIN4        */
   { {7, 4}, FUNC0, {3,12} },   /* 51   DI5     BORN_29   DIN5        */
   { {7, 5}, FUNC0, {3,13} },   /* 52   DI6     BORN_30   DIN6        */
   { {7, 6}, FUNC0, {3,14} },   /* 53   DI7     BORN_31   DIN7        */

   { {2, 1}, FUNC4, {5, 1} },   /* 54   DO0     BORN_14   DOUT0       */
   { {4, 6}, FUNC0, {2, 6} },   /* 55   DO1     BORN_06   DOUT1       */
   { {4, 5}, FUNC0, {2, 5} },   /* 56   DO2     BORN_08   DOUT2       */
   { {4, 4}, FUNC0, {2, 4} },   /* 57   DO3     BORN_10   DOUT3       */
   { {4, 8}, FUNC4, {5,12} },   /* 58   DO4     BORN_14   DOUT4       */
   { {4, 9}, FUNC4, {5,13} },   /* 59   DO5     BORN_15   DOUT5       */
   { {4,10}, FUNC4, {5,14} },   /* 60   DO6     BORN_16   DOUT6       */
   { {1, 5}, FUNC0, {1, 8} }    /* 61   DO7     BORN_17   DOUT7       */
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void gpioObtainPinConfig( uint32_t pin,
                                int8_t *pinNamePort, int8_t *pinNamePin,
																int8_t *func, int8_t *gpioPort,
																int8_t *gpioPin ){

   *pinNamePort = gpioPinsConfig[pin].pinName.port;
   *pinNamePin  = gpioPinsConfig[pin].pinName.pin;
   *func        = gpioPinsConfig[pin].func;
   *gpioPort    = gpioPinsConfig[pin].gpio.port;
   *gpioPin     = gpioPinsConfig[pin].gpio.pin;
}

/*==================[external functions definition]==========================*/

/* --------- Peripheral configutation methods ------------------------------ */

// Initialize
void gpioInit( void ){  
   /* Initializes GPIO */
   Chip_GPIO_Init(LPC_GPIO_PORT); 
}
   
// power. Enable or disable the peripheral energy and clock
void gpioPowerSet( bool_t power ){   
}

bool_t gpioPowerGet( void ){   
}
   
/* -- Single Pin property getters and setters methods - */
   
// mode
void gpioModeSet( int32_t gpioName, gpioMode_t mode ){

   bool_t ret_val     = 1;

   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;

   gpioObtainPinConfig( gpioName, &pinNamePort, &pinNamePin, &func,
                           &gpioPort, &gpioPin );

   switch( mode ){

      case GPIO_DISABLE:
      break;

      case GPIO_INPUT:
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), LPC4337_GPIO_INPUT );
      break;

      case GPIO_OUTPUT:
      //case GPIO_PUSHPULL:
      case GPIO_OPENDRAIN:
      //case GPIO_OPENCOLLECTOR:         
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_INACT | SCU_MODE_ZIF_DIS | SCU_MODE_INBUFF_EN,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), LPC4337_GPIO_OUTPUT );
         Chip_GPIO_SetPinState( LPC_GPIO_PORT, gpioPort, gpioPin, OFF);
      break;

      default:
         ret_val = 0;
      break;
   }
}

gpioMode_t gpioModeGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}

// pull
void gpioPullSet( int32_t gpioName, gpioPull_t pull ){
   
   bool_t ret_val     = 1;

   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;

   gpioObtainPinConfig( gpioName, &pinNamePort, &pinNamePin, &func,
                           &gpioPort, &gpioPin );

   switch( pull ){

      case GPIO_PULL_DISABLE:
      break;

      case GPIO_PULL_UP:
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
      break;

      case GPIO_PULL_DOWN:
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
      break;
      /*
      case GPIO_INPUT_PULLUP_PULLDOWN:
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_REPEATER | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
      break;
      */

      default:
         ret_val = 0;
      break;
   }
}

gpioPull_t gpioPullGet( int32_t gpioName ){
}
   
// value
void gpioValueSet( int32_t gpioName, bool_t value ){

   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;

   gpioObtainPinConfig( gpioName, &pinNamePort, &pinNamePin, &func,
                           &gpioPort, &gpioPin );

   Chip_GPIO_SetPinState( LPC_GPIO_PORT, gpioPort, gpioPin, value);
}

bool_t gpioValueGet( int32_t gpioName ){

   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;

   gpioObtainPinConfig( gpioName, &pinNamePort, &pinNamePin, &func,
                           &gpioPort, &gpioPin );

   return (bool_t) Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, gpioPort, gpioPin );
}

// speed
void gpioSpeedSet( int32_t gpioName, gpioSpeed_t speed ){
   // TODO: Implement
}

gpioSpeed_t gpioSpeedGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}
   
// stength
void gpioStengthSet( int32_t gpioName, gpioStrenght_t speed ){
   // TODO: Implement
}

gpioStrenght_t gpioStengthGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}

/* ------- Single Pin multiple property getters and setters methods -------- */

// config  is an uint32_t with "an OR" of direction, value, speed, stength
void gpioConfig( int32_t gpioName, int32_t config ){   
   
   // TODO: Implement
   
   gpioMode_t mode        = config & 0x0000000F;
   gpioPull_t pull        = config & 0x000000F0;
   gpioSpeed_t speed      = config & 0x00000F00;
   gpioStrenght_t stength = config & 0x0000F000;
   int32_t interrupt      = config & 0x00FF0000;
   
   // Mode
   gpioModeSet( gpioName, mode );

   // Pull
   gpioPullSet( gpioName, pull );

   // Speed
   gpioSpeedSet( gpioName, speed );
   
   // Stength
   gpioStengthSet( gpioName, stength );
   
   // Input interrupt
   gpioInterruptSet( gpioName, interrupt );
}

/* ------------ Interrupt properties methods ----------- */   
   
// Input Interrupt Callback
void gpioInterruptCallbackSet( int32_t gpioName,
                               interruptCallback_t interruptCallback ){
   // TODO: Implement
}

interruptCallback_t gpioInterruptCallbackGet( int32_t pin ){
   // TODO: Implement
   return 0;
}
   
// Input Interrupt Mode
void gpioInterruptModeSet( int32_t gpioName, gpioInterruptMode_t interruptMode ){
   // TODO: Implement
}

gpioInterruptMode_t gpioInterruptModeGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}

// Input Interrupt Type
void gpioInterruptTypeSet( int32_t gpioName, gpioInterruptType_t interruptType ){
   // TODO: Implement
}

gpioInterruptType_t gpioInterruptTypeGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}

// Input Interrupt
void gpioInterruptSet( int32_t gpioName, int32_t interruptMode ){
   // TODO: Implement
}

int32_t gpioInterruptGet( int32_t gpioName ){
   // TODO: Implement
   return 0;
}

/* -------------- Especific modes methods -------------- */
   
// Toggle a GPIO output --> Only for output mode
void gpioOutputToggle( int32_t gpioName ){
   return gpioValueSet( gpioName, !gpioValueGet(gpioName) );
}

// Read a GPIO input with a debounce time --> Only for input mode
bool_t gpioInputReadDebounced( int32_t gpioName, tick_t debounceTime ){
   // TODO: Implement
   return 0;
}
   
/* ----------------- GPIO PORT method ------------------ */

// Config Group of GPIOs
void gpioGroupModeSet( int32_t* gpios, 
                       uint8_t gpiosSize, 
                       int32_t config ){
   // TODO: Implement


   uint32_t i;

   for( i=0; i<gpiosSize; i++ )
      gpioConfig( gpios[i], config );
}

/*==================[end of file]============================================*/
