/*
 * tmp117.c
 *
 *  Created on: 11 mars 2019
 *      Author: Daniel
 */

#include "tmp117.h"


//extern I2C_HandleTypeDef hi2c1;

int8_t tempSensor_Init(void)
{
	uint8_t config[2] = {0x21, 0x02};
	int8_t ret = SOL_OK;

	if (tempSensor_GetDevID() != TMP117_DEV_ID)
	{
		return SOL_ERROR;
	}

	tempSensor_Write(TMP117_CONFIG_REG, config, 2);

	return ret;
}

void tempSensor_Write(uint8_t reg, uint8_t *pData, uint16_t size)
{
	HAL_I2C_Master_Transmit(&HI2C_TEMP, TMP117_DEV_WRITE_ADDR, &reg, 1, 100);
	HAL_I2C_Master_Transmit(&HI2C_TEMP, TMP117_DEV_WRITE_ADDR, pData, size, 100);
}

void tempSensor_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, TMP117_DEV_WRITE_ADDR, &reg, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, TMP117_DEV_READ_ADDR, pData, size, 100);
}

uint16_t tempSensor_GetDevID(void)
{
	uint8_t buffer[2] = {0};
	uint16_t devID = 0;
	tempSensor_Read(TMP117_DEV_ID_REG, buffer, 2);
	return devID = (buffer[1] << 8u) | buffer[0];
}

float tempSensor_GetTemp(void) // TODO: Set interrupt on ALERT Pin and read on interrupt
{
	uint8_t buffer[2] = {0};
	uint16_t rawTemp = 0;
	float celsiusTemp = 0;
	tempSensor_Read(TMP117_DEV_ID_REG, buffer, 2);
	rawTemp = (buffer[1] << 8u) | buffer[0];
	return celsiusTemp = rawTemp * 0.0078125;
}

void tempSensor_SoftReset(void)
{
	uint8_t data[2] = {0x21, 0x02};
	tempSensor_Write(TMP117_CONFIG_REG, data, 2);
}
