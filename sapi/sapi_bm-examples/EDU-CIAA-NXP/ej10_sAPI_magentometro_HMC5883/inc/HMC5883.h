/*
 * HMC5883.h
 *
 *  Created on: 6 de jun. de 2016
 *      Author: alejandro
 */

#ifndef HMC5883_H_
#define HMC5883_H_

#include "sAPI_I2c.h"

#define HMC5388_ADDRESS 0x1E
#define HMC5388_RECORD_A 0x0A
#define HMC5388_RECORD_B 0x0B
#define HMC5388_RECORD_C 0x0C

#define HMC5388_RECORD_ID_1 0x48
#define HMC5388_RECORD_ID_2 0x34
#define HMC5388_RECORD_ID_3 0x33

bool_t HMC5883_init();
bool_t HMC5883_isAlive();


#endif /* HMC5883_H_ */
