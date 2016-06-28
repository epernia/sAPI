/* Copyright 2016, Eric Pernia
 * Copyright 2016, Alejandro Permingeat.
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

#ifndef _SAPI_I2C_H_
#define _SAPI_I2C_H_
/** \brief Bare Metal example header file
 **
 ** This is a mini example of the CIAA Firmware
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example header file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * ENP          Eric Pernia
 * AlPer		Alejandro Permingeat
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 2016-05-02   v0.0.1   ENP   First version
 * 2016-06-23   v0.1   	 AlPer First functional version
 */

/*==================[inclusions]=============================================*/
#include "sAPI_DataTypes.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
typedef enum {
	sapi_I2C0,				/**< ID I2C number 0 */
	sapi_I2C1				/**< ID I2C number 11 */
} sAPI_i2cID_t;

/*==================[external data declaration]==============================*/

/*==================[ISR external functions definition]======================*/



/*==================[external functions definition]==========================*/
bool_t i2cConfig(sAPI_i2cID_t i2cID, uint32_t clockRateHz);
bool_t i2cWrite(uint8_t addr, uint8_t record, uint8_t* buf, uint16_t len);
bool_t i2cRead(uint8_t addr, uint8_t record, uint8_t* buf, uint16_t len);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_I2C_H_ */
