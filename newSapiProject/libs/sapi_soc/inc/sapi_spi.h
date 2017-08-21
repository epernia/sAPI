/* Copyright 2016, Eric Pernia.
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

#define spiConfig spiInit

/*==================[typedef]================================================*/

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
   SAPI_SPI_MODE_MASTER  = 0,
   // Slave mode
   SAPI_SPI_MODE_SLAVE   = 1
} spiMode_t;
   
typedef enum {
   // Select bits per frame
   SAPI_SPI_BITS_8,
   SAPI_SPI_BITS_9,
   SAPI_SPI_BITS_10,
   SAPI_SPI_BITS_11,
   SAPI_SPI_BITS_12,
   SAPI_SPI_BITS_13,
   SAPI_SPI_BITS_14,
   SAPI_SPI_BITS_15,
   SAPI_SPI_BITS_16
} spiBits_t;
   
typedef enum{
   // Data sampled on the first clock edge of SCK. A transfer starts and ends with activation and deactivation of the SSEL signal
   SAPI_SPI_PHASE_FIRST   = 0,
   // Data is sampled on the second clock edge of the SCK. A transfer starts with the first clock edge, and ends with the last sampling edge when SSEL signal is active  
   SAPI_SPI_PHASE_SECOND  = 1
} spiClockPhase_t;

typedef enum{
   // Clock is active high
   SAPI_SPI_POLARITY_HIGH = 0,
   // Clock is active low
   SAPI_SPI_POLARITY_LOW = 1
} spiPolarity_t;

typedef enum{
   // SPI data is transferred MSB (bit 7) first
   SAPI_SPI_ORDER_MSB = 0,
   // SPI data is transferred LSB (bit 0) first
   SAPI_SPI_ORDER_LSB = 1
} spiDataOrder_t;

/*==================[external data declaration]==============================*/

/*==================[ISR external functions declaration]=====================*/

/*==================[external functions declaration]=========================*/

bool_t spiRead( int32_t spi, uint8_t* buffer, uint32_t bufferSize );

bool_t spiWrite( int32_t spi, uint8_t* buffer, uint32_t bufferSize);

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

// Bit trasnfer order
void spiDataOrderSet( int32_t spi, spiDataOrder_t order );
spiDataOrder_t spiDataOrderGet( int32_t spi );

// Clock polarity
void spiPolaritySet( int32_t spi, spiPolarity_t order );
spiPolarity_t spiPolarityGet( int32_t spi );

// Bitrate
void spiBitRateSet( int32_t spi, uint32_t bitrate );
uint32_t spiBitRateGet( int32_t spi );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_SPI_H_ */
