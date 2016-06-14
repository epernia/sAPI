/*
 * HMC5883.c
 *
 *  Created on: 4 de jun. de 2016
 *      Author: alejandro
 */

#include "HMC5883.h"
#include "sAPI_I2c.h"


bool_t HMC5883_init()
{
	if (FALSE == i2cConfig(sapi_I2C0,100000))
	{
		return FALSE;
		/** A problem is detected during I2C initialization*/
	}

	/** If the initialization of I2C is OK,
	 *  an alive check to the chip is performed*/
	return(HMC5883_isAlive());
}

bool_t HMC5883_isAlive()
{
	uint8_t buf[3];
	uint8_t result = FALSE;

	/** In order to check if the chip is alive, the Identification number of the
	 *  chip is read. According to datasheet:
	 *  Record A: address: 0x0A must has the value 0x48
	 *  Record B: address: 0x0B must has the value 0x34
	 *  Record C: address: 0x0C must has the value 0x33
	 *   */


	if (i2cRead(HMC5388_ADDRESS, HMC5388_RECORD_A, buf, 3) == TRUE)
	{
		if ((buf[0] == HMC5388_RECORD_ID_1) &&
			(buf[1] == HMC5388_RECORD_ID_2) &&
			(buf[2] == HMC5388_RECORD_ID_3))
		{
			result= TRUE;
		}
	}

	return(result);
}

