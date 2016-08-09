/* Copyright 2016, Eric Pernia
 * Copyright 2016, Alejandro Permingeat.
 * Copyright 2016, Eric Pernia
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

 /*
  * Date:
  * 2016-05-02 Eric Pernia - Only define API
  * 2016-06-23 Alejandro Permingeat - First functional version
  * 2016-08-07 Eric Pernia - Improve names
  */

/*==================[inclusions]=============================================*/

#include "chip.h"

#include "sAPI_I2c.h"

/*==================[macros and definitions]=================================*/

#define I2C_MAX_TX_BUFF  16

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t i2cConfig( uint8_t i2cNumber, uint32_t clockRateHz ){

   if( i2cNumber != I2C0 ){
      return FALSE;
   }

   /* Configuracion de las lineas de SDA y SCL de la placa */
   Chip_SCU_I2C0PinConfig( I2C0_STANDARD_FAST_MODE );

   /* Inicializacion del periferico */
   Chip_I2C_Init( i2cNumber );
   /* Seleccion de velocidad del bus */
   Chip_I2C_SetClockRate( i2cNumber, clockRateHz );
   /* Configuracion para que los eventos se resuelvan por polliong
    * (la otra opcion es por interrupcion) */
   Chip_I2C_SetMasterEventHandler( i2cNumber, Chip_I2C_EventHandlerPolling );

   return TRUE;
}


bool_t i2cRead( uint8_t i2cNumber, 
                uint8_t addr, 
                uint8_t record, 
                uint8_t * buf, 
                uint16_t len ){

   I2CM_XFER_T i2cData;

   if( i2cNumber != I2C0 ){
      return FALSE;
   }

   i2cData.slaveAddr = addr;
   i2cData.options = 0;
   i2cData.status = 0;
   i2cData.txBuff = &record;
   i2cData.txSz = 1;
   i2cData.rxBuff = buf;
   i2cData.rxSz = len;

   if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ) {
      return FALSE;
   }

   return TRUE;
}


bool_t i2cWrite( uint8_t i2cNumber, 
                 uint8_t addr, 
                 uint8_t record, 
                 uint8_t * buf, 
                 uint16_t len ){

   I2CM_XFER_T i2cData;
   uint8_t txBuff[I2C_MAX_TX_BUFF];
   uint8_t i;

   if( i2cNumber != I2C0 ){
      return FALSE;
   }

   /* First, the record to be write must be sent */
   txBuff[0] = record;

   /* If length of data to be sent is greater than (I2C_MAX_TX_BUFF-1),
    * modify the length to be sent. Remember that the firs byte of
    * the txBuff is reserved for the "record id"
    */
   if( (I2C_MAX_TX_BUFF-1) < len ){
      len = I2C_MAX_TX_BUFF - 1;
   }

   for( i=0; i<len; i++ ){
      txBuff[i+1] = buf[i];
   }

   /* Prepare the i2cData register */
   i2cData.slaveAddr = addr;
   i2cData.options = 0;
   i2cData.status = 0;
   i2cData.txBuff = &txBuff;
   i2cData.txSz = len + 1;
   i2cData.rxBuff = NULL;
   i2cData.rxSz = 0;

   /* Send the i2c data */
   if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ){
      return FALSE;
   }

   return TRUE;
}

/*==================[ISR external functions definition]======================*/

/*==================[end of file]============================================*/
