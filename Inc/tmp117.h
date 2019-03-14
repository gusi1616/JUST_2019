/*
 * tmp117.h
 *
 *  Created on: 11 mars 2019
 *      Author: Daniel
 */

#ifndef TMP117_H_
#define TMP117_H_

#include "sol_common.h"

/** Device ID **/
#define TMP117_DEV_ID 0x0117

/** Device address **/
#define TMP117_DEV_READ_ADDR 		0b10010001u
#define TMP117_DEV_WRITE_ADDR 		0b10010000u

/** Register address **/
#define TMP117_TEMP_REG 			0x00
#define TMP117_CONFIG_REG 			0x01
#define TMP117_HIGH_LIMIT_REG 		0x02
#define TMP117_LOW_LIMIT_REG 		0x03
#define TMP117_EEPROM_UNLOCK_REG 	0x04
#define TMP117_EEPROM1_REG 			0x05
#define TMP117_EEPROM2_REG 			0x06
#define TMP117_TEMP_OFFSET_REG 		0x07
#define TMP117_EEPROM3_REG 			0x08
#define TMP117_DEV_ID_REG 			0x0F

/** Function prototypes **/
int8_t tempSensor_Init(void);
void tempSensor_Write(uint8_t reg, uint8_t *pData, uint16_t size);
void tempSensor_Read(uint8_t reg, uint8_t *pData, uint16_t size);
uint16_t tempSensor_GetDevID(void);
float tempSensor_GetTemp(void);
void tempSensor_SoftReset(void);

#endif /* TMP117_H_ */
