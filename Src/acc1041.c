/*
 * acc1041.c
 *
 *  Created on: 18 mars 2019
 *      Author: Daniel
 */

#include "acc1041.h"

int8_t accelerometer_Init(void)
{
	uint8_t ctrl1_data = 0b01110000;		// Res 12bit, interrupt on, +-8g, wakeup disable
	uint8_t int_ctrl1_data = 0b00110000;	// Int pin enabled, active high, clear on read INT_REL
	uint8_t int_ctrl2_data = 0b00111111;	// All axis enabled
	uint8_t data_ctrl_data = 0b00000010; 	// 50 hz output rate
	int8_t ret = SOL_OK;

	if (accelerometer_GetDevID() != ACC1041_DEV_ID)
	{
		return SOL_ERROR;
	}

	// Set config mode and set bits
	accelerometer_Write(ACC1041_CTRL1_REG, &ctrl1_data, 1);

	// Set interrupt
	accelerometer_Write(ACC1041_INT_CTRL1_REG, &int_ctrl1_data, 1);

	// Set interrupt axis
	accelerometer_Write(ACC1041_INT_CTRL2_REG, &int_ctrl2_data, 1);

	// Set output rate
	accelerometer_Write(ACC1041_DATA_CTRL_REG, &data_ctrl_data, 1);

	// Set to operating mode
	accelerometer_Write(ACC1041_CTRL1_REG, &(ctrl1_data & 0x80u), 1);

	return ret;
}

int8_t accelerometer_Write(uint8_t reg, uint8_t *pData, uint16_t size)
{
	if (HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, &reg, 1, 100) != HAL_OK)
		return SOL_ERROR;
	if (HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, pData, size, 100) != HAL_OK)
		return SOL_ERROR;

	return SOL_OK;
}

int8_t accelerometer_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
	if (HAL_I2C_Master_Transmit(&HI2C_ACC, ACC1041_DEV_WRITE_ADDR, &reg, 1, 100) != HAL_OK)
		return SOL_ERROR;
	if (HAL_I2C_Master_Receive(&HI2C_ACC, ACC1041_DEV_READ_ADDR, pData, size, 100) != HAL_OK)
		return SOL_ERROR;

	return SOL_OK;
}

uint16_t accelerometer_GetDevID(void)
{
	uint8_t buffer[2] = {0};
	uint16_t devID = 0;
	accelerometer_Read(ACC1041_DEV_ID_REG, buffer, 2);
	return devID = (buffer[1] << 8u) | buffer[0];
}

int16_t accelerometer_ReadX(void)
{
	uint8_t msb, lsb;
	int16_t data;
	accelerometer_Read(ACC1041_XOUT_H_REG, &msb, 1);
	accelerometer_Read(ACC1041_XOUT_L_REG, &lsb, 1);

	return data = (msb << 8u) | lsb;
}

int16_t accelerometer_ReadY(void)
{
	uint8_t msb, lsb;
	int16_t data;
	accelerometer_Read(ACC1041_YOUT_H_REG, &msb, 1);
	accelerometer_Read(ACC1041_YOUT_L_REG, &lsb, 1);

	return data = (msb << 8u) | lsb;
}

int16_t accelerometer_ReadZ(void)
{
	uint8_t msb, lsb;
	int16_t data;
	accelerometer_Read(ACC1041_ZOUT_H_REG, &msb, 1);
	accelerometer_Read(ACC1041_ZOUT_L_REG, &lsb, 1);

	return data = (msb << 8u) | lsb;
}

uint64_t accelerometer_ProcessData(void)
{
	int16_t x_raw_value, y_raw_value, z_raw_value;
	float x_g_value, y_g_value, z_g_value;
	uint64_t data;

	x_raw_value = accelerometer_ReadX();
	y_raw_value = accelerometer_ReadY();
	z_raw_value = accelerometer_ReadZ();

	x_g_value = x_raw_value * ACC1041_CONVERSION_12BIT_8G;
	y_g_value = y_raw_value * ACC1041_CONVERSION_12BIT_8G;
	z_g_value = z_raw_value * ACC1041_CONVERSION_12BIT_8G;

	return data; // TODO: Return data in one big 64 or array
}
