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

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 2016-05-02   v0.0.1   ENP   First version
 */

/*==================[inclusions]=============================================*/

#include "sapi_spi.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/

#define SPI_MODE_MASK     0x00000001
#define SPI_BLOCK_MASK    0x00000002
#define SPI_PHASE_MASK    0x00000004
#define SPI_POLARITY_MASK 0x00000008
#define SPI_ORDER_MASK    0x00000010

typedef struct{
   spi_data_t* bufferOut;
   spi_data_t* bufferIn;
   uint32_t count;
   uint32_t index;
   spiCallback_t afterFrameCallback;
   spiCallback_t afterXferCallback;
   spiStatus_t status;
   uint32_t config;
   uint32_t freq;
} spiInfo_t;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

spiInfo_t spi0Info;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t spiInit( int32_t spi ){
   
   bool_t retVal = TRUE;
   
   if( spi == SPI0 ){
      
  		/* Set up clock and power for SSP1 module */
		// Configure SSP SSP1 pins
		Chip_SCU_PinMuxSet(0xf, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CLK0
		Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); // MISO1
		Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5)); // MOSI1

		Chip_SCU_PinMuxSet(0x6, 1, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CS1 configured as GPIO
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 0);
      
      // Initialize SSP Peripheral
      Chip_SSP_Init( LPC_SSP1 );
      Chip_SSP_Enable( LPC_SSP1 );
      
   } else{
      retVal = FALSE;
   }
   
   return retVal;
}

bool_t spiXferStart( int32_t spi, const spi_data_t* bufferout, spi_data_t* bufferin, size_t count )
{
   bool_t retVal = FALSE;
   Chip_SSP_DATA_SETUP_T xferConfig;

   if(SPI_READY == spiStatusGet(spi))
   {
      if(SPI_IS_BLOCKING(spiConfigGet(spi)))
      {
         xferConfig.tx_data = bufferout;
         xferConfig.tx_cnt  = 0;
         xferConfig.rx_data = bufferin;
         xferConfig.rx_cnt  = 0;
         xferConfig.length  = count;

         if( spi == SPI0 ){
            Chip_SSP_RWFrames_Blocking( LPC_SSP1, &xferConfig );
            retVal = TRUE;
         } else{
            
         }
      }
      else if(SPI_IS_NONBLOCKING(spiConfigGet(spi)))
      {
         if( spi == SPI0 ){
            spi0Info.tx_data = bufferout;
            spi0Info.rx_data = bufferin;
            spi0Info.count  = count;
            spi0Info.index  = 0;
            /* TODO: Set beginning of transfer somehow. Set state? */
            spi0Info.status = SPI_BUSY;
            Chip_SSP_Int_FlushData(LPC_SSP1);
            Chip_SSP_SendFrame(LPC_SSP1, spi0Info.tx_data[spi0Info.index++]);
            retVal = TRUE;
         } else{

	 }
      }
      else
      {
         
      }
   }

   return retVal;
}

void spiXferEnd( int32_t spi )
{
   if(SPI_BUSY == spiStatusGet(spi))
   {
      if(SPI_IS_NONBLOCKING(spiConfigGet(spi)))
      {
         if( spi == SPI0 ){
            Chip_SSP_Int_FlushData(LPC_SSP1);
            spi0Info.status = READY;
         } else{
            
         }
      }
   }
}

void spiConfig( int32_t spi, uint32_t config ){   
   
   // TODO: Implement
   
   spiMode_t mode           = config & SPI_MODE_MASK;
   spiBlockmode_t blockMode = config & SPI_BLOCK_MASK;
   spiPhase_t phase         = config & SPI_PHASE_MASK;
   spiPolarity_t polarity   = config & SPI_POLARITY_MASK;
   spiBitOrder_t order      = config & SPI_ORDER_MASK;

   spiModeSet( spi, mode );

   spiXferModeSet( spi, pull );

   spiPhaseSet( spi, phase);

   spiPolaritySet( spi, stength );

   spiBitOrderSet( spi, speed );

   spiBitsSet( spi, SPI_BITS_DEFAULT );

   spiFreqSet( spi, SPI_FREQ_DEFAULT );

   //TODO
}

// power. Enable or disable the peripheral energy and clock
void spiPowerSet( int32_t spi, bool_t power )
{

}

bool_t spiPowerGet( int32_t spi )
{

}
   
/* -- spi properties getters and setters methods - */
   
// SPI mode
void spiModeSet( int32_t spi, spiMode_t mode )
{
   if( spi == SPI0 )
   {
      if( SPI_MASTER == mode )
      {
         Chip_SSP_Set_Mode(LPC_SSP1, SSP_MODE_MASTER);
	 spi0Info.config &= ~SPI_MODE_MASK;
         spi0Info.config |= mode;
      }
      else if( SPI_SLAVE == mode )
      {
         Chip_SSP_Set_Mode(LPC_SSP1, SSP_MODE_SLAVE);
         spi0Info.config &= ~SPI_MODE_MASK;
         spi0Info.config |= mode;
      }
      else
      {
      }
   }
}

spiMode_t spiModeGet( int32_t spi )
{

}

// Transfer mode
void spiXferModeSet( int32_t spi, spiXferMode_t mode )
{
   if( spi == SPI0 )
   {
      if( SPI_BLOCK == mode )
      {
         spi0Info.config &= ~SPI_BLOCK_MASK;
         spi0Info.config |= mode;
      }
      else if( SPI_NONBLOCK == mode )
      {
         spi0Info.config &= ~SPI_BLOCK_MASK;
         spi0Info.config |= mode;
      }
      else
      {
      }
   }
}

spiXferMode_t spiXferModeGet( int32_t spi )
{

}

// Bits per frame
void spiBitsSet( int32_t spi, uint8_t bits )
{
   if( spi == SPI0 )
   {
      if( bits >= 4 && bits <= 16)
      {
         LPC_SSP1->CR0 = (LPC_SSP1->CR0 & ~0x0F) | (bits - 1)
         spi0Info.config &= ~SPI_BITS_MASK;
         spi0Info.config |= bits << 5;
      }
      else
      {
      }
   }
}

uint8_t spiBitsGet( int32_t spi )
{

}
   
// Clock phase
void spiClockPhaseSet( int32_t spi, spiClockPhase_t phase )
{
   if( spi == SPI0 )
   {
      if( SPI_PHASE_FIRST == phase )
      {
         LPC_SSP1->CR0 = (LPC_SSP1->CR0 & (1<<7)) | SSP_CR0_CPHA_FIRST;
         spi0Info.config &= ~SPI_PHASE_MASK;
         spi0Info.config |= phase;
      }
      else if( SPI_PHASE_SECOND == phase )
      {
         LPC_SSP1->CR0 = (LPC_SSP1->CR0 & (1<<7)) | SSP_CR0_CPHA_SECOND;
         spi0Info.config &= ~SPI_PHASE_MASK;
         spi0Info.config |= phase;
      }
      else
      {
      }
   }
}

bool_t spiClockPhaseGet( int32_t spi )
{

}

// Bit transfer order
void spiBitOrderSet( int32_t spi, spiBitOrder_t order )
{
   if( spi == SPI0 )
   {
      /* Feature not available in SSP0/1 peripherals, only in SPI. MSB is always transferred first. */
      spi0Info.config &= ~SPI_ORDER_MASK;
      spi0Info.config |= SPI_ORDER_MSB; /* Default value */
   }
}

spiBitOrder_t spiBitOrderGet( int32_t spi )
{

}

// Clock polarity
void spiPolaritySet( int32_t spi, spiPolarity_t polarity )
{
   if( spi == SPI0 )
   {
      if( SPI_POLARITY_HIGH == polarity )
      {
         LPC_SSP1->CR0 = (LPC_SSP1->CR0 & ~(1<<6)) | SPI_CR_CPOL_HI;
         spi0Info.config &= ~SPI_POLARITY_MASK;
         spi0Info.config |= polarity;
      }
      else if( SPI_POLARITY_LOW == polarity )
      {
         LPC_SSP1->CR0 = (LPC_SSP1->CR0 & ~(1<<6)) | SPI_CR_CPOL_LO;
         spi0Info.config &= ~SPI_POLARITY_MASK;
         spi0Info.config |= polarity;
      }
      else
      {
      }
   }
}

spiPolarity_t spiPolarityGet( int32_t spi )
{

}

// Frequency
void spiFreqSet( int32_t spi, uint32_t freq )
{
   /* Clock 208MHz. Max theoretical bitrate is 208MHz/2 = 104MHz.  PCLK / (CPSDVSR * [SCR+1]). */
   /* TODO: Validate frequency? */
   if( spi == SPI0 )
   {
      Chip_SPI_SetBitRate(LPC_SSP1, freq);
      /* Now find out the exact frequency set */
      pClk = Chip_Clock_GetRate(Chip_SSP_GetPeriphClockIndex(LPC_SSP1));
      spi0Info.freq = pClk / (LPC_SSP1->CPSR & 0xF) * (((LPC_SSP1->CR0 >> 8) & 0xF) + 1);
   }
}

uint32_t spiFreqGet( int32_t spi )
{

}

spiStatus_t spiStatusGet( int32_t spi )
{

}

/*==================[ISR external functions definition]======================*/

void spi0_irqhandler(void)
{
   Chip_SSP_Int_Disable(LPC_SSP1);
   if (spi0Info.afterFrameCallback) {
      (spi0Info.afterFrameCallback)();
   }
   if (spi0Info.index < spi0Info.count) {
      /* check if RX FIFO contains data */
      *((uint16_t*)spi0Info.rx_data + spi0Info.index) = Chip_SSP_ReceiveFrame(LPC_SSP1);
      Chip_SSP_SendFrame(LPC_SSP1, *((uint16_t*)spi0Info.tx_data + spi0Info.index));
   }
   else {
      if (spi0Info.afterXferCallback) {
         (spi0Info.afterXferCallback)();
      }
      spi0Info.index = 0;
      spi0Info.status = SPI_READY;
   }
   Chip_SSP_Int_Enable(LPC_SSP1);
}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
