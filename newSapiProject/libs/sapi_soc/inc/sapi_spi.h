/* Copyright 2016, Eric Pernia.
 * Copyright 2017, Marcos Ziegler.
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

/* Date: 2016-05-02 */

#ifndef _SAPI_SPI_H_
#define _SAPI_SPI_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"
#include "sapi_board_map.h"
#include "sapi_soc_map.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/* SPI Config int32_t */
/*

spiConfig[31:0] = spiBitOrder_t[4:4],
                  spiPolarity_t[3:3],
                  spiPhase_t[2:2],
                  spiBlockmode_t[1:1],
                  spiMode_t[0:0]

               order      polar      ph         xfer          mode
    31  5      4  4       3  3      2  2        1  1          0  0
     0x00         0          0         0           0             0
                  0  MSB     0  HI     0 first     0 BLOCK       0 MASTER
                  1  LSB     1  LO     1 second    1 NONBLOCK    1 SLAVE
                                                                                             
                                                                                            
                                                                                               
                                                                                             

*/

/* SPI Properties */  
/*
 SPI module:
 - Power
 - Mode (master/slave)
 - Transfer mode (blocking/nonblocking)
 - Bits per frame
 - Bit order (LSB first/MSB first)
 - Clock phase (sample on first edge/sample on second edge)
 - Clock polarity (active high/active low)
 - Frequency
*/
   
/* SPI Properties values */   
   
typedef enum{
   // Master mode
   SPI_MODE_MASTER = (0 << 0),
   // Slave mode
   SPI_MODE_SLAVE  = (1 << 0)
} spiMode_t;
 
typedef enum{
   // Transfer mode
   SPI_BLOCK     = (0 << 1), // blocking transfer
   SPI_NONBLOCK  = (1 << 1), // non-blocking transfer
} spiXferMode_t;

typedef enum{
   // Data sampled on the first clock edge of SCK. A transfer starts and ends with activation and deactivation of the SSEL signal
   SPI_PHASE_FIRST   = (0 << 2),
   // Data is sampled on the second clock edge of the SCK. A transfer starts with the first clock edge, and ends with the last sampling edge when SSEL signal is active  
   SPI_PHASE_SECOND  = (1 << 2)
} spiClockPhase_t;

typedef enum{
   // Clock is active high
   SPI_POLARITY_HIGH = (0 << 3),
   // Clock is active low
   SPI_POLARITY_LOW  = (1 << 3)
} spiPolarity_t;

typedef enum{
   // SPI data is transferred MSB (bit 7) first
   SPI_ORDER_MSB = (0 << 4),
   // SPI data is transferred LSB (bit 0) first
   SPI_ORDER_LSB = (1 << 4)
} spiBitOrder_t;

typedef enum{
   SPI_ERROR,
   SPI_READY,
   SPI_BUSY
} spiStatus_t;

typedef enum{
   SPI_AFTER_XFER,
   SPI_AFTER_FRAME,
   SPI_XFER_ERROR
} spiEvent_t;

typedef void (spiCallback_t)( void );

/*==================[external data declaration]==============================*/

/*==================[ISR external functions declaration]=====================*/

/*==================[external functions declaration]=========================*/

//Multiple frames transfer
bool_t spiXferStart( int32_t spi, const spi_data_t* bufferout, spi_data_t* bufferin, size_t count );

void spiXferEnd( int32_t spi );

//Single frame transfer
bool_t spiReadSingle( int32_t spi, spi_data_t* datain );
  
bool_t spiWriteSingle( int32_t spi, spi_data_t* dataout );

bool_t spiXferSingle( int32_t spi, spi_data_t* dataout , spi_data_t* datain );

/* --------- Peripheral configutation methods ------------------------------ */

// Initialize
void spiInit( int32_t spi );
   
// power. Enable or disable the peripheral energy and clock
void spiPowerSet( int32_t spi, bool_t power );
bool_t spiPowerGet( int32_t spi );
   
/* -- spi properties getters and setters methods - */
   
// SPI mode
void spiModeSet( int32_t spi, spiMode_t mode );
spiMode_t spiModeGet( int32_t spi );

// Transfer mode
void spiXferModeSet( int32_t spi, spiXferMode_t mode );
spiXferMode_t spiXferModeGet( int32_t spi );

// Bits per frame
void spiBitsSet( int32_t spi, uint8_t bits );
uint8_t spiBitsGet( int32_t spi );
   
// Clock phase
void spiClockPhaseSet( int32_t spi, spiClockPhase_t phase );
bool_t spiClockPhaseGet( int32_t spi );

// Bit transfer order
void spiBitOrderSet( int32_t spi, spiBitOrder_t order );
spiBitOrder_t spiBitOrderGet( int32_t spi );

// Clock polarity
void spiPolaritySet( int32_t spi, spiPolarity_t order );
spiPolarity_t spiPolarityGet( int32_t spi );

// Frequency
void spiFreqSet( int32_t spi, uint32_t freq );
uint32_t spiFreqGet( int32_t spi );

spiStatus_t spiStatusGet( int32_t spi );

/* ------- spi multiple property getters and setters methods -------- */

// config  is an uint32_t with "an OR" of Frequency, Clock polarity, Bit transfer order, etc.
void spiConfig( int32_t spi, uint32_t config );

// Set the callback method for a specified event that occurs while transferring data
void spiConfigCallback( int32_t spi, spiEvent_t event_mask, spiCallback_t *callback);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[module example]=========================================*/

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_SPI_H_ */
