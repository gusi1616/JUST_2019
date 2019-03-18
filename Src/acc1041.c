/*
 * acc1041.c
 *
 *  Created on: 18 mars 2019
 *      Author: Daniel
 */

#include "acc1041.h"

int8_t accelerometer_Init(void)
{
	uint8_t config[2] = {0x21, 0x02};
	int8_t ret = SOL_OK;

	if (accelerometer_GetDevID() != ACC1041_DEV_ID)
	{
		return SOL_ERROR;
	}

	return ret;
}

int8_t accelerometer_Write(uint8_t reg, uint8_t *pData, uint16_t size)
{
	HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, &reg, 1, 100);
	HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, pData, size, 100);

	return SOL_OK;
}

int8_t accelerometer_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
	HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, &reg, 1, 100);
	HAL_I2C_Master_Receive(&HI2C_ACC, ACC1041_DEV_READ_ADDR, pData, size, 100);

	return SOL_OK;
}

uint16_t accelerometer_GetDevID(void)
{
	uint8_t buffer[2] = {0};
	uint16_t devID = 0;
	accelerometer_Read(ACC1041_DEV_ID_REG, buffer, 2);
	return devID = (buffer[1] << 8u) | buffer[0];
}
