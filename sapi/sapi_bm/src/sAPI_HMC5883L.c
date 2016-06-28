/* Copyright 2016, Alejandro Permingeat.
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

/** \brief Main example source file
 **
 ** This is a mini example of the CIAA Firmware using HMC5883L.
 **
 **/

#include "sAPI_HMC5883L.h"         /* <= sAPI HMC5883L header */
#include "sAPI_I2c.h"         	   /* <= sAPI I2C header */

bool_t HMC5883L_init()
{
	i2cConfig(sapi_I2C0,100000);
	return(HMC5883L_isAlive());
}

bool_t HMC5883L_isAlive()
{
	uint8_t idRegister[3];
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__ID_REG_A,&idRegister,3);

	if ((sapi_HMC5883L__VALUE_ID_REG_A == idRegister[0]) &&
		(sapi_HMC5883L__VALUE_ID_REG_B == idRegister[1]) &&
		(sapi_HMC5883L__VALUE_ID_REG_C == idRegister[2]))
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}

bool_t HMC5883L_prepareDefaultConfig(sAPI_HMC5883L_config_t *config)
{
	config->gain = sapi_HMC5883L__DEFAULT_gain;
	config->meassurement = sapi_HMC5883L__DEFAULT_messurement;
	config->rate = sapi_HMC5883L__DEFAULT_rate;
	config->samples = sapi_HMC5883L__DEFAULT_sample;
	config->mode = sapi_HMC5883L__DEFAULT_mode;
	return(TRUE);
}


bool_t HMC5883L_config(sAPI_HMC5883L_config_t config)
{
	uint8_t registerA, registerB, registerMode;

	registerA = config.samples;
	registerA = registerA<<3;
	registerA |= config.rate;
	registerA = registerA<<2;
	registerA |= config.meassurement;

	registerB = config.gain;
	registerB = registerB << 5;

	registerMode = config.mode;


	i2cWrite(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__CONFIG_A,&registerA,1);
	i2cWrite(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__CONFIG_B,&registerB,1);
	i2cWrite(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__MODE,&registerMode,1);

	return (TRUE);
}

bool_t HMC5883L_getXYZ_raw(uint16_t *x, uint16_t *y, uint16_t *z)
{
	bool_t result = TRUE;

	uint8_t x_MSB, x_LSB;
	uint8_t y_MSB, y_LSB;
	uint8_t z_MSB, z_LSB;

	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__X_MSB,&x_MSB,1);
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__X_LSB,&x_LSB,1);
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__Y_MSB,&y_MSB,1);
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__Y_LSB,&y_LSB,1);
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__Z_MSB,&z_MSB,1);
	i2cRead(sapi_HMC5883L_ADD,sapi_HMC5883L_REG__Z_LSB,&z_LSB,1);

	return(result);

}

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Main example source file
 ** @{ */





