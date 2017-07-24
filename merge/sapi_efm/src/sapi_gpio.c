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
 *
 */

/* Date: 2015-09-23 */
/* Date: 2017-06-02 EFM32HG support */

/*==================[inclusions]=============================================*/

#include "sapi_gpio.h"

// emlib includes
#include "em_system.h"
//#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
//#include "em_assert.h"
#include "em_gpio.h"

#include "moveIntVectorToRAM.h" // @Important!!

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void gpioObtainPin( gpioMap_t pin,
                          GPIO_Port_TypeDef* gpioPort,
                          unsigned int *gpioPin );

static void gpioObtainEM4Pin( EM4_pin_t em4Pin,
                             GPIO_Port_TypeDef* gpioPort,
                             unsigned int *gpioPin );

/*==================[internal data definition]===============================*/

/*
const pinConfigGpioLpc4337_t gpioPinsConfig[] = {
*/
	//{ {PinNamePortN ,PinNamePinN}, PinFUNC, {GpioPortN, GpioPinN} }

   /* --------------------------------------------------------------- */
   /*                           EDU-CIAA-NXP                          */
   /* --------------------------------------------------------------- */
   /*                             Snap  sAPI   Connector  Serigraphy  */
   /* --------------------------------------------------------------- */

   // { {1,15}, FUNC0, {0, 2} },   //  0   DIO0    CON2_09   ENET_RXD0

   // { {1, 4}, FUNC0, {0,11} },   //  1   DIO1    CON2_21   SPI_MOSI
   // { {4, 9}, FUNC4, {5,13} },   //  2   DIO2    CON2_23   LCD_EN

   // { {6, 1}, FUNC0, {3, 0} },   //  3   DIO3    CON2_29   GPIO0
   // { {6, 5}, FUNC0, {3, 4} },   //  4   DIO4    CON2_31   GPIO2
   // { {6, 8}, FUNC4, {5,16} },   //  5   DIO5    CON2_33   GPIO4
   // { {6,10}, FUNC0, {3, 6} },   //  6   DIO6    CON2_35   GPIO6

   // { {0, 0}, FUNC0, {0, 0} },   //  7   DIO7    CON2_04   ENET_RXD1
   // { {0, 1}, FUNC0, {0, 1} },   //  8   DIO8    CON2_06   ENET_TXEN
   // { {7, 7}, FUNC0, {3,15} },   //  9   DIO9    CON2_08   ENET_MDC
   // { {1,16}, FUNC0, {0, 3} },   // 10   DIO10   CON2_10   ENET_CRS_DV
   // { {1,17}, FUNC0, {0,12} },   // 11   DIO11   CON2_12   ENET_MDIO
   // { {1,18}, FUNC0, {0,13} },   // 12   DIO12   CON2_14   ENET_TXD0
   // { {1,20}, FUNC0, {0,15} },   // 13   DIO13   CON2_16   ENET_TXD1
   // { {1, 3}, FUNC0, {0,10} },   // 14   DIO14   CON2_18   SPI_MISO

   // { {4,10}, FUNC4, {5,14} },   // 15   DIO15   CON2_22   LCD4
   // { {4, 8}, FUNC4, {5,12} },   // 16   DIO16   CON2_24   LCDRS
   // { {4, 6}, FUNC0, {2, 6} },   // 17   DIO17   CON2_26   LCD3
   // { {4, 5}, FUNC0, {2, 5} },   // 18   DIO18   CON2_28   LCD2
   // { {4, 4}, FUNC0, {2, 4} },   // 19   DIO19   CON2_30   LCD1

   // { {6, 4}, FUNC0, {3, 3} },   // 20   DIO20   CON2_32   GPIO1
   // { {6, 7}, FUNC4, {5,15} },   // 21   DIO21   CON2_34   GPIO3
   // { {6, 9}, FUNC0, {3, 5} },   // 22   DIO22   CON2_36   GPIO5
   // { {6,11}, FUNC0, {3, 7} },   // 23   DIO23   CON2_38   GPIO7
   // { {6,12}, FUNC0, {2, 8} },   // 24   DIO24   CON2_40   GPIO8

   // { {2, 4}, FUNC4, {5, 4} },   // 25   DIO25   CON1_23   RS232_RXD
   // { {2, 3}, FUNC4, {5, 3} },   // 26   DIO26   CON1_25   RS232_TXD
   // { {3, 1}, FUNC4, {5, 8} },   // 27   DIO27   CON1_27   CAN_RD
   // { {3, 2}, FUNC4, {5, 9} },   // 28   DIO28   CON1_29   CAN_TD
   // { {7, 4}, FUNC0, {3,12} },   // 29   DIO29   CON1_31   T_COL1
   // { {4, 0}, FUNC0, {2, 0} },   // 30   DIO30   CON1_33   T_FIL0
   // { {4, 3}, FUNC0, {2, 3} },   // 31   DIO31   CON1_35   T_FIL3
   // { {4, 2}, FUNC0, {2, 2} },   // 32   DIO32   CON1_37   T_FIL2
   // { {1, 5}, FUNC0, {1, 8} },   // 33   DIO33   CON1_39   T_COL0

   // { {7, 5}, FUNC0, {3,13} },   // 34   DIO34   CON1_34   T_COL2
   // { {4, 1}, FUNC0, {2, 1} },   // 35   DIO35   CON1_36   T_FIL1
/*
   { {4, 1}, FUNC0, {2, 1} },   //   0   CON1_36   T_FIL1
   { {7, 5}, FUNC0, {3,13} },   //   1   CON1_34   T_COL2

   { {1, 5}, FUNC0, {1, 8} },   //   2   CON1_39   T_COL0
   { {4, 2}, FUNC0, {2, 2} },   //   3   CON1_37   T_FIL2
   { {4, 3}, FUNC0, {2, 3} },   //   4   CON1_35   T_FIL3
   { {4, 0}, FUNC0, {2, 0} },   //   5   CON1_33   T_FIL0
   { {7, 4}, FUNC0, {3,12} },   //   6   CON1_31   T_COL1

   { {3, 2}, FUNC4, {5, 9} },   //   7   CON1_29   CAN_TD
   { {3, 1}, FUNC4, {5, 8} },   //   8   CON1_27   CAN_RD

   { {2, 3}, FUNC4, {5, 3} },   //   9   CON1_25   RS232_TXD
   { {2, 4}, FUNC4, {5, 4} },   //  10   CON1_23   RS232_RXD

   { {6,12}, FUNC0, {2, 8} },   //  11   CON2_40   GPIO8
   { {6,11}, FUNC0, {3, 7} },   //  12   CON2_38   GPIO7
   { {6, 9}, FUNC0, {3, 5} },   //  13   CON2_36   GPIO5
   { {6, 7}, FUNC4, {5,15} },   //  14   CON2_34   GPIO3
   { {6, 4}, FUNC0, {3, 3} },   //  15   CON2_32   GPIO1

   { {4, 4}, FUNC0, {2, 4} },   //  16   CON2_30   LCD1
   { {4, 5}, FUNC0, {2, 5} },   //  17   CON2_28   LCD2
   { {4, 6}, FUNC0, {2, 6} },   //  18   CON2_26   LCD3
   { {4, 8}, FUNC4, {5,12} },   //  19   CON2_24   LCDRS
   { {4,10}, FUNC4, {5,14} },   //  20   CON2_22   LCD4

   { {1, 3}, FUNC0, {0,10} },   //  21   CON2_18   SPI_MISO

   { {1,20}, FUNC0, {0,15} },   //  22   CON2_16   ENET_TXD1
   { {1,18}, FUNC0, {0,13} },   //  23   CON2_14   ENET_TXD0
   { {1,17}, FUNC0, {0,12} },   //  24   CON2_12   ENET_MDIO
   { {1,16}, FUNC0, {0, 3} },   //  25   CON2_10   ENET_CRS_DV
   { {7, 7}, FUNC0, {3,15} },   //  26   CON2_08   ENET_MDC
   { {0, 1}, FUNC0, {0, 1} },   //  27   CON2_06   ENET_TXEN
   { {0, 0}, FUNC0, {0, 0} },   //  28   CON2_04   ENET_RXD1

   { {6,10}, FUNC0, {3, 6} },   //  29   CON2_35   GPIO6
   { {6, 8}, FUNC4, {5,16} },   //  30   CON2_33   GPIO4
   { {6, 5}, FUNC0, {3, 4} },   //  31   CON2_31   GPIO2
   { {6, 1}, FUNC0, {3, 0} },   //  32   CON2_29   GPIO0

   { {4, 9}, FUNC4, {5,13} },   //  33   CON2_23   LCDEN

   { {1, 4}, FUNC0, {0,11} },   //  34   CON2_21   SPI_MOSI

   { {1,15}, FUNC0, {0, 2} },   //  35   CON2_09   ENET_RXD0

   { {1, 0}, FUNC0, {0, 4} },   // 36   TEC1    TEC_1
   { {1, 1}, FUNC0, {0, 8} },   // 37   TEC2    TEC_2
   { {1, 2}, FUNC0, {0, 9} },   // 38   TEC3    TEC_3
   { {1, 6}, FUNC0, {1, 9} },   // 39   TEC4    TEC_4

   { {2, 0}, FUNC4, {5, 0} },   // 43   LEDR    LED0_R
   { {2, 1}, FUNC4, {5, 1} },   // 44   LEDG    LED0_G
   { {2, 2}, FUNC4, {5, 2} },   // 45   LEDB    LED0_B
   { {2,10}, FUNC0, {0,14} },   // 40   LED1    LED1
   { {2,11}, FUNC0, {1,11} },   // 41   LED2    LED2
   { {2,12}, FUNC0, {1,12} },   // 42   LED3    LED3
*/
   /* --------------------------------------------------------------- */
   /*                             CIAA-NXP                            */
   /* --------------------------------------------------------------- */
   /*                             Snap  sAPI   Connector  Serigraphy  */
   /* --------------------------------------------------------------- */
/*
   { {4, 0}, FUNC0, {2, 0} },   // 46   DI0     BORN_24   DIN0
   { {4, 1}, FUNC0, {2, 1} },   // 47   DI1     BORN_25   DIN1
   { {4, 2}, FUNC0, {2, 2} },   // 48   DI2     BORN_26   DIN2
   { {4, 3}, FUNC0, {2, 3} },   // 49   DI3     BORN_27   DIN3
   { {7, 3}, FUNC0, {3,11} },   // 50   DI4     BORN_28   DIN4
   { {7, 4}, FUNC0, {3,12} },   // 51   DI5     BORN_29   DIN5
   { {7, 5}, FUNC0, {3,13} },   // 52   DI6     BORN_30   DIN6
   { {7, 6}, FUNC0, {3,14} },   // 53   DI7     BORN_31   DIN7

   { {2, 1}, FUNC4, {5, 1} },   // 54   DO0     BORN_14   DOUT0
   { {4, 6}, FUNC0, {2, 6} },   // 55   DO1     BORN_06   DOUT1
   { {4, 5}, FUNC0, {2, 5} },   // 56   DO2     BORN_08   DOUT2
   { {4, 4}, FUNC0, {2, 4} },   // 57   DO3     BORN_10   DOUT3
   { {4, 8}, FUNC4, {5,12} },   // 58   DO4     BORN_14   DOUT4
   { {4, 9}, FUNC4, {5,13} },   // 59   DO5     BORN_15   DOUT5
   { {4,10}, FUNC4, {5,14} },   // 60   DO6     BORN_16   DOUT6
   { {1, 5}, FUNC0, {1, 8} }    // 61   DO7     BORN_17   DOUT7
};
*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/*
static void gpioObtainPinConfig( gpioMap_t pin,
                                int8_t *pinNamePort, int8_t *pinNamePin,
																int8_t *func, int8_t *gpioPort,
																int8_t *gpioPin ){

   *pinNamePort = gpioPinsConfig[pin].pinName.port;
   *pinNamePin  = gpioPinsConfig[pin].pinName.pin;
   *func        = gpioPinsConfig[pin].func;
   *gpioPort    = gpioPinsConfig[pin].gpio.port;
   *gpioPin     = gpioPinsConfig[pin].gpio.pin;
}*/


static void gpioObtainPin( gpioMap_t pin,
                           GPIO_Port_TypeDef* gpioPort,
                           unsigned int *gpioPin ){

   switch(pin){
      case PA0:
         *gpioPort = gpioPortA;
         *gpioPin = 0;
      break;
      case PA1:
         *gpioPort = gpioPortA;
         *gpioPin = 1;
      break;
      case PA2:
         *gpioPort = gpioPortA;
         *gpioPin = 2;
      break;
      case PC0:
         *gpioPort = gpioPortC;
         *gpioPin = 0;
      break;
      case PC4:
         *gpioPort = gpioPortC;
         *gpioPin = 4;
      break;
      case PC9:
         *gpioPort = gpioPortC;
         *gpioPin = 9;
      break;
      case PC10:
         *gpioPort = gpioPortC;
         *gpioPin = 10;
      break;
      case PD4:
         *gpioPort = gpioPortD;
         *gpioPin = 4;
      break;
      case PD5:
         *gpioPort = gpioPortD;
         *gpioPin = 5;
      break;
      case PD6:
         *gpioPort = gpioPortD;
         *gpioPin = 6;
      break;
      case PD7:
         *gpioPort = gpioPortD;
         *gpioPin = 7;
      break;
      case PE10:
         *gpioPort = gpioPortE;
         *gpioPin = 10;
      break;
      case PE11:
         *gpioPort = gpioPortE;
         *gpioPin = 11;
      break;
      case PE12:
         *gpioPort = gpioPortE;
         *gpioPin = 12;
      break;
      case PE13:
         *gpioPort = gpioPortE;
         *gpioPin = 13;
      break;
      case PF2:
         *gpioPort = gpioPortF;
         *gpioPin = 2;
      break;
      case PF1:
         *gpioPort = gpioPortF;
         *gpioPin = 1;
      break;
      case PF4:
         *gpioPort = gpioPortF;
         *gpioPin = 4;
      break;
      case PF5:
         *gpioPort = gpioPortF;
         *gpioPin = 5;
      break;
      default:
      break;
   }

}


static void gpioObtainEM4Pin( EM4_pin_t em4Pin,
                              GPIO_Port_TypeDef* gpioPort,
                              unsigned int *gpioPin ){

   switch(em4Pin){
      case EM4_PA0:
         *gpioPort = gpioPortA;
         *gpioPin = 0;
      break;
      case EM4_PC9:
         *gpioPort = gpioPortC;
         *gpioPin = 9;
      break;
      case EM4_PE13:
         *gpioPort = gpioPortE;
         *gpioPin = 13;
      break;
      case EM4_PF1:
         *gpioPort = gpioPortF;
         *gpioPin = 1;
      break;
      case EM4_PF2:
         *gpioPort = gpioPortF;
         *gpioPin = 2;
      break;
      default:
      break;
   }

}


/*==================[external functions definition]==========================*/

bool_t gpioConfig( gpioMap_t pin, gpioConfig_t config ){

   bool_t ret_val     = 1;

   /*
   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;
   */


   //GPIO_Port_TypeDef gpioPort = 0;
   //unsigned int gpioPin = 0;
   //gpioObtainPinConfig( pin, &pinNamePort, &pinNamePin, &func,
   //                        &gpioPort, &gpioPin );

   switch(config){

      case GPIO_ENABLE:
         /* Initializes GPIO */
         // Chip_GPIO_Init(LPC_GPIO_PORT);

         gpioPowerSet( ON );

         // Move interrupt vectors to RAM
         moveInterruptVectorToRam();

/*
         // GPIO Default configurations

         // Port A Configuration
         // Pin PA0 is configured to Input enabled
         GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
               | GPIO_P_MODEL_MODE0_INPUT;
         // Pin PA1 is configured to Push-pull
         GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
               | GPIO_P_MODEL_MODE1_PUSHPULL;

         // Port C Configuration
         // Pin PC4 is configured to Input enabled
         GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
               | GPIO_P_MODEL_MODE4_INPUT;
         // Pin PC9 is configured to Input enabled
         GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
               | GPIO_P_MODEH_MODE9_INPUT;
         // Pin PC10 is configured to Input enabled
         GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
               | GPIO_P_MODEH_MODE10_INPUT;

         // Port F Configuration
         // Pin PF2 is configured to Input enabled
         GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
               | GPIO_P_MODEL_MODE2_INPUT;
*/
      break;

      case GPIO_DISABLE:

         switch(pin){
            case PA0:
               /* Pin PA0 is configured to DISABLED */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_DISABLED;
            break;
            case PA1:
               /* Pin PA1 is configured to DISABLED */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
                     | GPIO_P_MODEL_MODE1_DISABLED;
            break;
            case PA2:
               /* Pin PA2 is configured to DISABLED */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_DISABLED;
            break;
            case PB7:
               /* Pin PB7 is configured to DISABLED */
               GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_DISABLED;
            break;
            case PB8:
               /* Pin PB8 is configured to DISABLED*/
               GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
                     | GPIO_P_MODEH_MODE8_DISABLED;
            break;
            case PC0:
               /* Pin PC0 is configured to DISABLED */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_DISABLED;
            break;
            case PC4:
               /* Pin PC4 is configured to DISABLED */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_DISABLED;
            break;
            case PC9:
               /* Pin PC9 is configured to DISABLED */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
                     | GPIO_P_MODEH_MODE9_DISABLED;
            break;
            case PC10:
               /* Pin PC10 is configured to DISABLED */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_DISABLED;
            break;
            case PC14:
               /* Pin PC14 is configured to DISABLED */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                     | GPIO_P_MODEH_MODE14_DISABLED;
            break;
            case PC15:
               /* Pin PC15 is configured to DISABLED */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                     | GPIO_P_MODEH_MODE15_DISABLED;
            break;
            case PD4:
               /* Pin PD4 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_DISABLED;
            break;
            case PD5:
               /* Pin PD5 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_DISABLED;
            break;
            case PD6:
               /* Pin PD6 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                     | GPIO_P_MODEL_MODE6_DISABLED;
            break;
            case PD7:
               /* Pin PD7 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_DISABLED;
            break;
            case PE10:
               /* Pin PE10 is configured to DISABLED */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_DISABLED;
            break;
            case PE11:
               /* Pin PE11 is configured to DISABLED */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                     | GPIO_P_MODEH_MODE11_DISABLED;
            break;
            case PE12:
               /* Pin PE12 is configured to DISABLED */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
                     | GPIO_P_MODEH_MODE12_DISABLED;
            break;
            case PE13:
               /* Pin PE13 is configured to DISABLED */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
                     | GPIO_P_MODEH_MODE13_DISABLED;
            break;
            case PF0:
               /* Pin PF0 is configured to DISABLED */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_DISABLED;
            break;
            case PF2:
               /* Pin PF2 is configured to DISABLED */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_DISABLED;
            break;
            case PF4:
               /* Pin PF4 is configured to DISABLED */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_DISABLED;
            break;
            case PF5:
               /* Pin PF5 is configured to DISABLED */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_DISABLED;
            break;
            default:
            break;
         }
      break;

      case GPIO_INPUT:
         /*
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
         */


         switch(pin){
            case PA0:
               /* Pin PA0 is configured to Input enabled */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUT;
            break;
            case PA1:
               /* Pin PA1 is configured to Input enabled */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
                     | GPIO_P_MODEL_MODE1_INPUT;
            break;
            case PA2:
               /* Pin PA2 is configured to Input enabled */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUT;
            break;
            case PB7:
               /* Pin PB7 is configured to Input enabled */
               GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUT;
            break;
            case PB8:
               /* Pin PB8 is configured to Input enabled */
               GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
                     | GPIO_P_MODEH_MODE8_INPUT;
            break;
            case PC0:
               /* Pin PC0 is configured to Input enabled */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUT;
            break;
            case PC4:
               /* Pin PC4 is configured to Input enabled */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUT;
            break;
            case PC9:
               /* Pin PC9 is configured to Input enabled */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
                     | GPIO_P_MODEH_MODE9_INPUT;
            break;
            case PC10:
               /* Pin PC10 is configured to Input enabled */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUT;
            break;
            case PC14:
               /* Pin PC14 is configured to Input enabled */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                     | GPIO_P_MODEH_MODE14_INPUT;
            break;
            case PC15:
               /* Pin PC15 is configured to Input enabled */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                     | GPIO_P_MODEH_MODE15_INPUT;
            break;
            case PD4:
               /* Pin PD4 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUT;
            break;
            case PD5:
               /* Pin PD5 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUT;
            break;
            case PD6:
               /* Pin PD6 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                     | GPIO_P_MODEL_MODE6_INPUT;
            break;
            case PD7:
               /* Pin PD7 is configured to Input enabled */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUT;
            break;
            case PE10:
               /* Pin PE10 is configured to Input enabled */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUT;
            break;
            case PE11:
               /* Pin PE11 is configured to Input enabled */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                     | GPIO_P_MODEH_MODE11_INPUT;
            break;
            case PE12:
               /* Pin PE12 is configured to Input enabled */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
                     | GPIO_P_MODEH_MODE12_INPUT;
            break;
            case PE13:
               /* Pin PE13 is configured to Input enabled */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
                     | GPIO_P_MODEH_MODE13_INPUT;
            break;
            case PF0:
               /* Pin PF0 is configured to Input enabled */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUT;
            break;
            case PF2:
               /* Pin PF2 is configured to Input enabled */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUT;
            break;
            case PF4:
               /* Pin PF4 is configured to Input enabled */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUT;
            break;
            case PF5:
               /* Pin PF5 is configured to Input enabled */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUT;
            break;
            default:
            break;
         }

      break;

      case GPIO_INPUT_PULLUP:
         /*
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
         */


         switch(pin){
            case PA0:
               /* Pin PA0 is configured to Input enabled with pull-up */
               GPIO->P[0].DOUT |= (1 << 0);
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PA1:
               /* Pin PA1 is configured to Input enabled with pull-up */
               GPIO->P[0].DOUT |= (1 << 1);
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
                     | GPIO_P_MODEL_MODE1_INPUTPULL;
            break;
            case PA2:
               /* Pin PA2 is configured to Input enabled with pull-up */
               GPIO->P[0].DOUT |= (1 << 2);
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUTPULL;
            break;
            case PB7:
               /* Pin PB7 is configured to Input enabled with pull-up */
               GPIO->P[1].DOUT |= (1 << 7);
               GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUTPULL;
            break;
            case PB8:
               /* Pin PB8 is configured to Input enabled with pull-up */
               GPIO->P[1].DOUT |= (1 << 8);
               GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
                     | GPIO_P_MODEH_MODE8_INPUTPULL;
            break;
            case PC0:
               /* Pin PC0 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 0);
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PC4:
               /* Pin PC4 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 4);
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PC9:
               /* Pin PC9 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 9);
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
                     | GPIO_P_MODEH_MODE9_INPUTPULL;
            break;
            case PC10:
               /* Pin PC10 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 10);
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUTPULL;
            break;
            case PC14:
               /* Pin PC14 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 14);
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                     | GPIO_P_MODEH_MODE14_INPUTPULL;
            break;
            case PC15:
               /* Pin PC15 is configured to Input enabled with pull-up */
               GPIO->P[2].DOUT |= (1 << 15);
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                     | GPIO_P_MODEH_MODE15_INPUTPULL;
            break;
            case PD4:
               /* Pin PD4 is configured to Input enabled with pull-up */
               GPIO->P[3].DOUT |= (1 << 4);
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PD5:
               /* Pin PD5 is configured to Input enabled with pull-up */
               GPIO->P[3].DOUT |= (1 << 5);
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUTPULL;
            break;
            case PD6:
               /* Pin PD6 is configured to Input enabled with pull-up */
               GPIO->P[3].DOUT |= (1 << 6);
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                     | GPIO_P_MODEL_MODE6_INPUTPULL;
            break;
            case PD7:
               /* Pin PD7 is configured to Input enabled with pull-up */
               GPIO->P[3].DOUT |= (1 << 7);
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUTPULL;
            break;
            case PE10:
               /* Pin PE10 is configured to Input enabled with pull-up */
               GPIO->P[4].DOUT |= (1 << 10);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUTPULL;
            break;
            case PE11:
               /* Pin PE11 is configured to Input enabled with pull-up */
               GPIO->P[4].DOUT |= (1 << 11);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                     | GPIO_P_MODEH_MODE11_INPUTPULL;
            break;
            case PE12:
               /* Pin PE12 is configured to Input enabled with pull-up */
               GPIO->P[4].DOUT |= (1 << 12);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
                     | GPIO_P_MODEH_MODE12_INPUTPULL;
            break;
            case PE13:
               /* Pin PE13 is configured to Input enabled with pull-up */
               GPIO->P[4].DOUT |= (1 << 13);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
                     | GPIO_P_MODEH_MODE13_INPUTPULL;
            break;
            case PF0:
               /* Pin PF0 is configured to Input enabled with pull-up */
               GPIO->P[5].DOUT |= (1 << 0);
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PF2:
               /* Pin PF2 is configured to Input enabled with pull-up */
               GPIO->P[5].DOUT |= (1 << 2);
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUTPULL;
            break;
            case PF4:
               /* Pin PF4 is configured to Input enabled with pull-up */
               GPIO->P[5].DOUT |= (1 << 4);
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PF5:
               /* Pin PF5 is configured to Input enabled with pull-up */
               GPIO->P[5].DOUT |= (1 << 5);
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUTPULL;
            break;
            default:
            break;
         }

      break;

      case GPIO_INPUT_PULLDOWN:
         /*
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
         */


         switch(pin){
            case PA0:
               /* Pin PA0 is configured to Input enabled with pull-down */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PA1:
               /* Pin PA1 is configured to Input enabled with pull-down */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
                     | GPIO_P_MODEL_MODE1_INPUTPULL;
            break;
            case PA2:
               /* Pin PA2 is configured to Input enabled with pull-down */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUTPULL;
            break;
            case PB7:
               /* Pin PB7 is configured to Input enabled with pull-down */
               GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUTPULL;
            break;
            case PB8:
               /* Pin PB8 is configured to Input enabled with pull-down */
               GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
                     | GPIO_P_MODEH_MODE8_INPUTPULL;
            break;
            case PC0:
               /* Pin PC0 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PC4:
               /* Pin PC4 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PC9:
               /* Pin PC9 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
                     | GPIO_P_MODEH_MODE9_INPUTPULL;
            break;
            case PC10:
               /* Pin PC10 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUTPULL;
            break;
            case PC14:
               /* Pin PC14 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                     | GPIO_P_MODEH_MODE14_INPUTPULL;
            break;
            case PC15:
               /* Pin PC15 is configured to Input enabled with pull-down */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                     | GPIO_P_MODEH_MODE15_INPUTPULL;
            break;
            case PD4:
               /* Pin PD4 is configured to Input enabled with pull-down */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PD5:
               /* Pin PD5 is configured to Input enabled with pull-down */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUTPULL;
            break;
            case PD6:
               /* Pin PD6 is configured to Input enabled with pull-down */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                     | GPIO_P_MODEL_MODE6_INPUTPULL;
            break;
            case PD7:
               /* Pin PD7 is configured to Input enabled with pull-down */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_INPUTPULL;
            break;
            case PE10:
               /* Pin PE10 is configured to Input enabled with pull-down */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_INPUTPULL;
            break;
            case PE11:
               /* Pin PE11 is configured to Input enabled with pull-down */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                     | GPIO_P_MODEH_MODE11_INPUTPULL;
            break;
            case PE12:
               /* Pin PE12 is configured to Input enabled with pull-down */
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
                     | GPIO_P_MODEH_MODE12_INPUTPULL;
            break;
            case PE13:
               /* Pin PE13 is configured to Input enabled with pull-down */
               GPIO->P[4].DOUT |= (1 << 13);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
                     | GPIO_P_MODEH_MODE13_INPUTPULL;
            break;
            case PF0:
               /* Pin PF0 is configured to Input enabled with pull-down */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_INPUTPULL;
            break;
            case PF2:
               /* Pin PF2 is configured to Input enabled with pull-down */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_INPUTPULL;
            break;
            case PF4:
               /* Pin PF4 is configured to Input enabled with pull-down */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_INPUTPULL;
            break;
            case PF5:
               /* Pin PF5 is configured to Input enabled with pull-down */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_INPUTPULL;
            break;
            default:
            break;
         }

      break;
      case GPIO_INPUT_PULLUP_PULLDOWN:
         /*
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_REPEATER | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_INPUT );
         */
      break;

      case GPIO_OUTPUT:
         /*
         Chip_SCU_PinMux(
            pinNamePort,
            pinNamePin,
            SCU_MODE_INACT | SCU_MODE_ZIF_DIS | SCU_MODE_INBUFF_EN,
            func
         );
         Chip_GPIO_SetDir( LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), GPIO_OUTPUT );
         Chip_GPIO_SetPinState( LPC_GPIO_PORT, gpioPort, gpioPin, 0);
         */

         switch(pin){
            case PA0:
               /* Pin PA0 is configured to Push-pull */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_PUSHPULL;
            break;
            case PA1:
               /* Pin PA1 is configured to Push-pull */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK)
                     | GPIO_P_MODEL_MODE1_PUSHPULL;
            break;
            case PA2:
               /* Pin PA2 is configured to Push-pull */
               GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_PUSHPULL;
            break;
            case PB7:
               /* Pin PB7 is configured to Push-pull */
               GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_PUSHPULL;
            break;
            case PB8:
               /* Pin PB8 is configured to Push-pull */
               GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
                     | GPIO_P_MODEH_MODE8_PUSHPULL;
            break;
            case PC0:
               /* Pin PC0 is configured to Push-pull */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_PUSHPULL;
            break;
            case PC4:
               /* Pin PC4 is configured to Push-pull */
               GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_PUSHPULL;
            break;
            case PC9:
               /* Pin PC9 is configured to Push-pull */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
                     | GPIO_P_MODEH_MODE9_PUSHPULL;
            break;
            case PC10:
               /* Pin PC10 is configured to Push-pull */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_PUSHPULL;
            break;
            case PC14:
               /* Pin PC14 is configured to Push-pull */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                     | GPIO_P_MODEH_MODE14_PUSHPULL;
            break;
            case PC15:
               /* Pin PC15 is configured to Push-pull */
               GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                     | GPIO_P_MODEH_MODE15_PUSHPULL;
            break;
            case PD4:
               /* Pin PD4 is configured to Push-pull */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_PUSHPULL;
            break;
            case PD5:
               /* Pin PD5 is configured to Push-pull */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_PUSHPULL;
            break;
            case PD6:
               /* Pin PD6 is configured to Push-pull */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
                     | GPIO_P_MODEL_MODE6_PUSHPULL;
            break;
            case PD7:
               /* Pin PD7 is configured to Push-pull */
               GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
                     | GPIO_P_MODEL_MODE7_PUSHPULL;
            break;
            case PE10:
               /* Pin PE10 is configured to Push-pull */
               GPIO->P[4].DOUT |= (1 << 10);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                     | GPIO_P_MODEH_MODE10_PUSHPULL;
            break;
            case PE11:
               /* Pin PE11 is configured to Push-pull */
               GPIO->P[4].DOUT |= (1 << 11);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                     | GPIO_P_MODEH_MODE11_PUSHPULL;
            break;
            case PE12:
               /* Pin PE12 is configured to Push-pull */
               //GPIO->P[4].DOUT |= (1 << 12);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
                     | GPIO_P_MODEH_MODE12_PUSHPULL;
            break;
            case PE13:
               /* Pin PE13 is configured to Push-pull */
               //GPIO->P[4].DOUT |= (1 << 13);
               GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
                     | GPIO_P_MODEH_MODE13_PUSHPULL;
            break;
            case PF0:
               /* Pin PF0 is configured to Push-pull */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
                     | GPIO_P_MODEL_MODE0_PUSHPULL;
            break;
            case PF2:
               /* Pin PF2 is configured to Push-pull */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
                     | GPIO_P_MODEL_MODE2_PUSHPULL;
            break;
            case PF4:
               /* Pin PF4 is configured to Push-pull */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
                     | GPIO_P_MODEL_MODE4_PUSHPULL;
            break;
            case PF5:
               /* Pin PF5 is configured to Push-pull */
               GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
                     | GPIO_P_MODEL_MODE5_PUSHPULL;
            break;
            default:
            break;
         }

      break;

      default:
         ret_val = 0;
      break;
   }

   return ret_val;

}

// Enable or disable the peripheral energy and clock
bool_t gpioPowerSet( bool_t power ){

   if( power ){

      // Enable clock for GPIO
      CMU_ClockEnable(cmuClock_GPIO, true);

   } else{

      // Disable clock for GPIO
      CMU_ClockEnable(cmuClock_GPIO, false);

   }

   return TRUE;
}

bool_t gpioWrite( gpioMap_t pin, bool_t value ){

   bool_t ret_val = 1;

   GPIO_Port_TypeDef gpioPort = 0;
   unsigned int gpioPin = 0;

   // Obtengo el pin
   gpioObtainPin( pin, &gpioPort, &gpioPin );

   // Escribo el pin
   GPIO_PinModeSet( gpioPort, gpioPin, gpioModePushPull, value );

   /*
   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;
   */

   //gpioObtainPinConfig( pin, &pinNamePort, &pinNamePin, &func,
   //                        &gpioPort, &gpioPin );

   //Chip_GPIO_SetPinState( LPC_GPIO_PORT, gpioPort, gpioPin, value);

   return ret_val;
}


bool_t gpioToggle( gpioMap_t pin ){

   return gpioWrite( pin, !gpioRead(pin) );
}


bool_t gpioRead( gpioMap_t pin ){

   bool_t ret_val = OFF;

   /*
   int8_t pinNamePort = 0;
   int8_t pinNamePin  = 0;

   int8_t func        = 0;

   int8_t gpioPort    = 0;
   int8_t gpioPin     = 0;
   */
   //gpioObtainPinConfig( pin, &pinNamePort, &pinNamePin, &func,
   //                        &gpioPort, &gpioPin );

   //ret_val = (bool_t) Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, gpioPort, gpioPin );


   GPIO_Port_TypeDef gpioPort = 0;
   unsigned int gpioPin = 0;

   // Obtengo el pin
   gpioObtainPin( pin, &gpioPort, &gpioPin );

   // Leo el pin
   ret_val = (bool_t)GPIO_PinInGet( gpioPort, gpioPin );

   return ret_val;
}



// Interrupt ------------------------------------

volatile gpioInterruptCallback_t gpioInterruptCallbacks[10] = {
   sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr,
   sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr, sAPI_NullFuncPtr
};


bool_t gpioInterruptCallbackSet( gpioMap_t pin,
                                 gpioInterruptCallback_t interruptCallback )
{
   bool_t ret_val = TRUE;

   if( interruptCallback != NULL ){
      gpioInterruptCallbacks[pin] = interruptCallback;
   } else {
      ret_val = FALSE;
   }

   return ret_val;
}


bool_t gpioInterruptSet( gpioMap_t pin, gpioInterrupt_t interruptMode )
{
   bool_t ret_val = TRUE;

   GPIO_Port_TypeDef gpioPort = 0;
   unsigned int gpioPin = 0;

   // GPIO Interrupt input
//   gpioConfig( pin, GPIO_INPUT );

   /*
      // GPIO Interrupt input
      GPIO_PinModeSet(gpioPortF, 2, gpioModeInput, 0);
      // Select port F as IRQ 2 rising edge
      GPIO_IntConfig(gpioPortF, 2, true, false, true);
      GPIO->IFC = 0x04;
   */
   //GPIO_ExtIntConfig( PANIC_BUTTON_PORT, PANIC_BUTTON_PIN, PANIC_BUTTON_PIN, true, false, true );

   // Obtengo el pin
   gpioObtainPin( pin, &gpioPort, &gpioPin );

   switch( interruptMode ){

      // INTERRUPT DISABLE
      case GPIO_INTERRUPT_DISABLE:
         // Select port as IRQ
         GPIO_IntConfig( gpioPort, gpioPin, false, false, false );
      break;

      // INTERRUPT LEVEL
      case GPIO_INTERRUPT_LEVEL_LOW:
      break;

      case GPIO_INTERRUPT_LEVEL_HIGH:
      break;

      case GPIO_INTERRUPT_LEVEL_BOTH:
      break;

      // INTERRUPT EDGE
      case GPIO_INTERRUPT_EDGE_FALLING:
         // Select port as IRQ
         GPIO_IntConfig( gpioPort, gpioPin, false, true, true );
      break;

      case GPIO_INTERRUPT_EDGE_RISING:
         // Select port as IRQ
         GPIO_IntConfig( gpioPort, gpioPin, true, false, true );
      break;

      case GPIO_INTERRUPT_EDGE_BOTH:
         // Select port as IRQ
         GPIO_IntConfig( gpioPort, gpioPin, true, true, true );
      break;


      default:
         ret_val = FALSE;
      break;

   }

   if( interruptMode == GPIO_INTERRUPT_DISABLE ){

      // config GPIO IRQ
      NVIC_ClearPendingIRQ( GPIO_EVEN_IRQn );
      NVIC_DisableIRQ( GPIO_EVEN_IRQn );

      NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
      NVIC_DisableIRQ(GPIO_ODD_IRQn);

   } else {

      // config GPIO IRQ
      NVIC_ClearPendingIRQ( GPIO_EVEN_IRQn );
      NVIC_EnableIRQ( GPIO_EVEN_IRQn );

      NVIC_ClearPendingIRQ(GPIO_ODD_IRQn );
      NVIC_EnableIRQ( GPIO_ODD_IRQn );
   }

   return ret_val;
}

/* ------ EM4 ------------------------------- */

// Configure/deconfigure pin for Wake-up from EM4
void gpioConfigEM4Wakeup( EM4_pin_t em4Pin, bool_t enable,
                          bool_t wakupCondition ){

   GPIO_Port_TypeDef gpioPort = 0;
   unsigned int gpioPin = 0;

   gpioObtainEM4Pin( em4Pin, &gpioPort, &gpioPin );

   // Set pin as an input, used to wake the system
   GPIO_PinModeSet(gpioPort, gpioPin, gpioModeInputPull, 1);

   if(enable){

      // Retain GPIO modes while in EM4, to wake it up with button press
      GPIO->CTRL = 1;

      // EM4 Wake-up Polarity, EM4WUPOL (reset value 0x00, RW)
      // Write bit n to 1 for high wake-up request. Write bit n to 0 for low wake-up request
      if( wakupCondition == HIGH ){
         GPIO->EM4WUPOL |= em4Pin;    // High signal is button pushed state
      } else {
         GPIO->EM4WUPOL &= ~em4Pin;   // Low signal is button pushed state
      }

      GPIO->EM4WUEN |= em4Pin;
   }
   else{
      GPIO->EM4WUEN &= ~em4Pin;
   }

   GPIO->CMD = 1;   // EM4WUCLR = 1, to clear all previous events
}

/*==================[Interrupts]=============================================*/

static void GPIO_Unified_IRQ( void ){
   uint32_t iflags;
   // Get all interrupts.
   iflags = GPIO_IntGetEnabled(); // GPIO_IntGet();
   // Clear interrupts.
   GPIO_IntClear( iflags );
}

void GPIO_EVEN_IRQHandler( void ){
   GPIO_Unified_IRQ();
}

void GPIO_ODD_IRQHandler( void ){
   GPIO_Unified_IRQ();
}

/*==================[end of file]============================================*/
