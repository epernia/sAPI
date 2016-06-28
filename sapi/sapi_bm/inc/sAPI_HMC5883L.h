/* Copyright 2016, Alejandro Permingeat
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

#ifndef _SAPI_HMC5883L_H_
#define _SAPI_HMC5883L_H_
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
 * AlPer       Alejandro Permingeat
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160627   v0.0.1   First version
 */

/*==================[inclusions]=============================================*/
#include "sAPI_DataTypes.h"
#include "sAPI_I2c.h"
/*==================[macros]=================================================*/

#define sapi_HMC5883L_ADD	0x1E

#define sapi_HMC5883L_REG__CONFIG_A		0x00
#define sapi_HMC5883L_REG__CONFIG_B		0x01
#define sapi_HMC5883L_REG__MODE			0x02
#define sapi_HMC5883L_REG__X_MSB		0x03
#define sapi_HMC5883L_REG__X_LSB		0x04
#define sapi_HMC5883L_REG__Z_MSB		0x05
#define sapi_HMC5883L_REG__Z_LSB		0x06
#define sapi_HMC5883L_REG__Y_MSB		0x07
#define sapi_HMC5883L_REG__Y_LSB		0x08
#define sapi_HMC5883L_REG__STATUS		0x09
#define sapi_HMC5883L_REG__ID_REG_A		0x0A
#define sapi_HMC5883L_REG__ID_REG_B		0x0B
#define sapi_HMC5883L_REG__ID_REG_C		0x0C

#define sapi_HMC5883L__VALUE_ID_REG_A		0x48
#define sapi_HMC5883L__VALUE_ID_REG_B		0x34
#define sapi_HMC5883L__VALUE_ID_REG_C		0x33

/*==================[typedef]================================================*/

typedef enum {
	sapi_HMC5883L__1_sample = 0,
	sapi_HMC5883L__2_sample = 1,
	sapi_HMC5883L__4_sample = 2,
	sapi_HMC5883L__8_sample = 3,
	sapi_HMC5883L__DEFAULT_sample = sapi_HMC5883L__1_sample

} sAPI_HMC5883L_samples_t;

typedef enum {
	sapi_HMC5883L__0_75_Hz = 0,
	sapi_HMC5883L__1_50_Hz = 1,
	sapi_HMC5883L__3_Hz = 2,
	sapi_HMC5883L__7_50_Hz = 3,
	sapi_HMC5883L__15_Hz = 4,
	sapi_HMC5883L__30_Hz = 5,
	sapi_HMC5883L__75_Hz = 6,
	sapi_HMC5883L__DEFAULT_rate = sapi_HMC5883L__15_Hz
} sAPI_HMC5883L_rate_t;

typedef enum {
	sapi_HMC5883L__normal = 0,
	sapi_HMC5883L__positive = 1,
	sapi_HMC5883L__regative = 2,
	sapi_HMC5883L__DEFAULT_messurement = sapi_HMC5883L__normal
} sAPI_HMC5883L_messurement_t;

typedef enum {
	sapi_HMC5883L__1370 = 0, /* ± 0.88 Ga*/
	sapi_HMC5883L__1090 = 1, /* ± 1.3 Ga */
	sapi_HMC5883L__820 = 2,  /* ± 1.9 Ga */
	sapi_HMC5883L__660 = 3,  /* ± 2.5 Ga */
	sapi_HMC5883L__440 = 4,  /* ± 4.0 Ga */
	sapi_HMC5883L__390 = 5, /* ± 4.7 Ga */
	sapi_HMC5883L__330 = 6, /* ± 5.6 Ga */
	sapi_HMC5883L__230 = 7, /* ± 8.1 Ga */
	sapi_HMC5883L__DEFAULT_gain = sapi_HMC5883L__1090
} sAPI_HMC5883L_gain_t;

typedef enum {
	sapi_HMC5883L__continuous_measurement = 0,
	sapi_HMC5883L__single_measurement = 1,
	sapi_HMC5883L__idle = 2,
	sapi_HMC5883L__DEFAULT_mode = sapi_HMC5883L__single_measurement
} sAPI_HMC5883L_mode_t;

typedef struct{
	sAPI_HMC5883L_samples_t samples; /*number of samples averaged (1 to 8) per measurement output.*/
	sAPI_HMC5883L_rate_t rate; /* Data Output Rate Bits. These bits set the rate at which data
   	   	   	    				* is written to all three data output registers.*/
	sAPI_HMC5883L_messurement_t meassurement; /*Measurement Configuration Bits. These bits define the
   	   	   	   	   	    					   *measurement flow of the device, specifically whether or not
   	   	   	   	   	    					   *to incorporate an applied bias into the measurement.*/
	sAPI_HMC5883L_gain_t gain; /* Gain Configuration Bits. These bits configure the gain for
   	   	   	   	 	 	 	 	* the device. The gain configuration is common for all
   	   	   	   	 	 	 	 	* channels.*/
	sAPI_HMC5883L_mode_t mode; /* */
} sAPI_HMC5883L_config_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
bool_t HMC5883L_init();
bool_t HMC5883L_isAlive();
bool_t HMC5883L_prepareDefaultConfig(sAPI_HMC5883L_config_t *config);
bool_t HMC5883L_config(sAPI_HMC5883L_config_t config);
bool_t HMC5883L_getXYZ_raw(uint16_t *x, uint16_t *y, uint16_t *z);


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_HMC5883L_H_ */
