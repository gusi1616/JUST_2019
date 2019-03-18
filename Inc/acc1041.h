/*
 * acc1041.h
 *
 *  Created on: 18 mars 2019
 *      Author: Daniel
 */

#ifndef ACC1041_H_
#define ACC1041_H_

#include "sol_common.h"

/** Device ID **/
#define ACC1041_DEV_ID 0b00001010u

/** Device address **/
#define ACC1041_DEV_READ_ADDR 			0b00011101u
#define ACC1041_DEV_WRITE_ADDR 			0b00011100u

/** Register address **/
#define ACC1041_XOUT_L_REG 				0x06
#define ACC1041_XOUT_H_REG 				0x07
#define ACC1041_YOUT_L_REG 				0x08
#define ACC1041_YOUT_H_REG 				0x09
#define ACC1041_ZOUT_L_REG 				0x0A
#define ACC1041_ZOUT_H_REG 				0x0B
#define ACC1041_DCST_RESP_REG 			0x0C
#define ACC1041_DEV_ID_REG 				0x0F
#define ACC1041_INT_SOURCE_1_REG 		0x16
#define ACC1041_INT_SOURCE_2_REG 		0x17
#define ACC1041_STATUS_REG 				0x18
#define ACC1041_INT_REL_REG 			0x1A
#define ACC1041_CTRL1_REG 				0x1B
#define ACC1041_CTRL2_REG 				0x1D
#define ACC1041_INT_CTRL1_REG 			0x1E
#define ACC1041_INT_CTRL2_REG 			0x1F
#define ACC1041_DATA_CTRL_REG 			0x21
#define ACC1041_WAKEUP_TIMER_REG 		0x29
#define ACC1041_SELF_TEST_REG 			0x3A
#define ACC1041_WAKEUP_THRESHOLD_REG 	0x6A

/** Function prototypes **/
int8_t accelerometer_Init(void);
int8_t accelerometer_Write(uint8_t reg, uint8_t *pData, uint16_t size);
int8_t accelerometer_Read(uint8_t reg, uint8_t *pData, uint16_t size);
uint16_t accelerometer_GetDevID(void);

#endif /* ACC1041_H_ */
