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

spiConfig[31:0] = spiInterruptMode_t[31:28], 
                  spiInterrupt_t[27:24], 
                  spiBitrate_t[23:20], 
                  spiBitOrder_t[19:16], 
                  spiPolarity_t[15:12], 
                  spiPhase_t[11:8],  
                  spiBits_t[7:4]
                  spiMode_t[3:0]

   intm   int      bitrat        order      polar       ph         bits    mode
  31  28 27  24    23  20        19  16     15  12    11  8        7  4    3  0
   0000   0000      0000          0000       0000      0000        0000    0000
                       0  110        0  MSB     0  HI     0 first     0  8    0 MASTER
                       1  300        1  LSB     1  LO     1 second    1  9    1 SLAVE
                      10  600                                        10 10
                      11  1200                                       11 11
                     100  2400                                      100 12
                     101  4800                                      101 13
                     110  9600                                      110 14
                     111  14400                                     111 15
                    1000  19200                                    1000 16
                    1001  38400                 
                    1010  57600  
                    1011  115200 
                    1100  230400 
                    1101  460800 
                    1110  921600 
*/

/* SPI Properties */  
/*
 SPI module:
 - Power
 - Mode (master/slave)
 - Bits per frame
 - Bit order (LSB first/MSB first)
 - Clock phase (sample on first edge/sample on second edge)
 - Clock polarity (active high/active low)
 - Bitrate
 - Interrupt (Ver cuales pueden tener)
*/
   
/* SPI Properties values */   
   
typedef enum{
   // Master mode
   SPI_MODE_MASTER = (0 << 0),
   // Slave mode
   SPI_MODE_SLAVE  = (1 << 0)
} spiMode_t;
   
typedef enum {
   // Select bits per frame
   SPI_BITS_8  = (0 << 4),
   SPI_BITS_9  = (1 << 4),
   SPI_BITS_10 = (2 << 4),
   SPI_BITS_11 = (3 << 4),
   SPI_BITS_12 = (4 << 4),
   SPI_BITS_13 = (5 << 4),
   SPI_BITS_14 = (6 << 4),
   SPI_BITS_15 = (7 << 4),
   SPI_BITS_16 = (8 << 4)
} spiBits_t;
   
typedef enum{
   // Data sampled on the first clock edge of SCK. A transfer starts and ends with activation and deactivation of the SSEL signal
   SPI_PHASE_FIRST   = (0 << 8),
   // Data is sampled on the second clock edge of the SCK. A transfer starts with the first clock edge, and ends with the last sampling edge when SSEL signal is active  
   SPI_PHASE_SECOND  = (1 << 8)
} spiClockPhase_t;

typedef enum{
   // Clock is active high
   SPI_POLARITY_HIGH = (0 << 12),
   // Clock is active low
   SPI_POLARITY_LOW  = (1 << 12)
} spiPolarity_t;

typedef enum{
   // SPI data is transferred MSB (bit 7) first
   SPI_ORDER_MSB = (0 << 16),
   // SPI data is transferred LSB (bit 0) first
   SPI_ORDER_LSB = (1 << 16)
} spiDataOrder_t;

typedef enum{
   // SPI bitrate
   SPI_BITRATE_110    = (0 << 20),
   SPI_BITRATE_300    = (1 << 20),
   SPI_BITRATE_600    = (2 << 20),
   SPI_BITRATE_1200   = (3 << 20),
   SPI_BITRATE_2400   = (4 << 20),
   SPI_BITRATE_4800   = (5 << 20),
   SPI_BITRATE_9600   = (6 << 20),
   SPI_BITRATE_14400  = (7 << 20),
   SPI_BITRATE_19200  = (8 << 20),
   SPI_BITRATE_38400  = (9 << 20),
   SPI_BITRATE_57600  = (10 << 20),
   SPI_BITRATE_115200 = (11 << 20),
   SPI_BITRATE_230400 = (12 << 20),
   SPI_BITRATE_460800 = (13 << 20),
   SPI_BITRATE_921600 = (14 << 20)
} spiBitrate_t;

/*==================[external data declaration]==============================*/

/*==================[ISR external functions declaration]=====================*/

/*==================[external functions declaration]=========================*/

//Multiple frames transfer
bool_t spiRead( int32_t spi, uint8_t* buffer, uint32_t bufferSize );

bool_t spiWrite( int32_t spi, uint8_t* buffer, uint32_t bufferSize );

bool_t spiXfer( int32_t spi, uint8_t* bufferin, uint8_t* bufferout, uint32_t count );

//Single frame transfer
uint16_t spiReadSingle( int32_t spi );
  
void spiWriteSingle( int32_t spi, uint16_t data );

uint16_t spiXferSingle( int32_t spi, uint16_t data );

void spiCSSet( int32_t spi, uint8_t slave, bool_t value );

bool_t spiCSGet( int32_t spi, uint8_t slave );

/* --------- Peripheral configutation methods ------------------------------ */

// Initialize
void spiInit( int32_t spi );
   
// power. Enable or disable the peripheral energy and clock
void spiPowerSet( int32_t spi, bool_t power );
bool_t spiPowerGet( int32_t spi );
   
/* -- Single Pin property getters and setters methods - */
   
// SPI mode
void spiModeSet( int32_t spi, spiMode_t mode );
spiMode_t spiModeGet( int32_t spi );

// Bits per frame
void spiBitsSet( int32_t spi, spiBits_t bits );
spiBits_t spiBitsGet( int32_t spi );
   
// Clock phase
void spiClockPhaseSet( int32_t spi, spiClockPhase_t phase );
bool_t spiClockPhaseGet( int32_t spi );

// Bit transfer order
void spiBitOrderSet( int32_t spi, spiBitOrder_t order );
spiBitOrder_t spiBitOrderGet( int32_t spi );

// Clock polarity
void spiPolaritySet( int32_t spi, spiPolarity_t order );
spiPolarity_t spiPolarityGet( int32_t spi );

// Bitrate
void spiBitrateSet( int32_t spi, spiBitrate_t bitrate );
spiBitrate_t spiBitrateGet( int32_t spi );

/* ------- Single Pin multiple property getters and setters methods -------- */

// config  is an uint32_t with "an OR" of Bitrate, Clock polarity, Bit transfer order, etc.
void spiConfig( int32_t spi, uint32_t config );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[module example]=========================================*/

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_SPI_H_ */
