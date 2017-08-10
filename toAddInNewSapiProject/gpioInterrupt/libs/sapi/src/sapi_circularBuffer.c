/* Copyright 2016, Eric Pernia.
 * All rights reserved.
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

/* Date: 2016-10-06 */

/*==================[inclusions]=============================================*/

#include "sapi_circularBuffer.h"   /* <= own header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/


void circularBufferConfig( circularBuffer_t* buffer, uint8_t* bufferMemory,
      uint8_t bufferMemorySize, callBackFuncPtr_t emptyBufferCallback,
		callBackFuncPtr_t fullBufferCalback ){

   buffer->memoryAddress       = bufferMemory;
   buffer->size                = bufferMemorySize;
   buffer->readIndex           = 0;
   buffer->writeIndex          = 0;
   buffer->status = CIRCULAR_BUFFER_EMPTY;

   // Empty buffer callback
   if( emptyBufferCallback ){
      buffer->emptyBufferCallback = emptyBufferCallback;
   }

   // Full buffer callback
   if( fullBufferCalback ){
      buffer->fullBufferCalback = fullBufferCalback;
   }

}


circularBufferStatus_t circularBufferRead( circularBuffer_t* buffer,
                                           uint8_t *dataByte ){
	// Is Empty?
	if ( (buffer->readIndex) == (buffer->writeIndex) ){

	   // Error, empty buffer
	   buffer->status = CIRCULAR_BUFFER_EMPTY;

	   // Execute emptyBufferCallback
	   if( buffer->emptyBufferCallback ){
		   (* (buffer->emptyBufferCallback) )();
	   }

	} else {

	   buffer->status = CIRCULAR_BUFFER_NORMAL;

	   *dataByte = (buffer->memoryAddress)[ buffer->readIndex ];
	   buffer->readIndex = (buffer->readIndex + 1) % buffer->size;

	}

	return buffer->status;
}


circularBufferStatus_t circularBufferWrite( circularBuffer_t* buffer,
                                            uint8_t *dataByte ){

	// Is Full?
	if( ((buffer->writeIndex + 1) % (buffer->size) ) == (buffer->readIndex) ){

	   // Error, full buffer
	   buffer->status = CIRCULAR_BUFFER_FULL;

	   // Execute fullBufferCalback
	   if( buffer->fullBufferCalback ){
		   (* (buffer->fullBufferCalback) )();
	   }

	} else{

		buffer->status = CIRCULAR_BUFFER_NORMAL;

		(buffer->memoryAddress)[ buffer->writeIndex ] = *dataByte;
		buffer->writeIndex = (buffer->writeIndex + 1) % buffer->size;
	}

	return buffer->status;
}

/*==================[end of file]============================================*/
