/*
 * sol_common.h
 *
 *  Created on: 14 mars 2019
 *      Author: Daniel
 */

#ifndef SOL_COMMON_H_
#define SOL_COMMON_H_

#include "stm32l4xx_hal.h"

#define SOL_ERROR 	-1
#define SOL_OK 		0

#define HI2C_TEMP hi2c1
#define HI2C_ACC hi2c1

extern I2C_HandleTypeDef hi2c1;

#endif /* SOL_COMMON_H_ */
