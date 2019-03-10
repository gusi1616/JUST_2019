/*
 * can.h
 *
 *  Created on: 10 mars 2019
 *      Author: Daniel
 */

#ifndef CAN_H_
#define CAN_H_

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

void CAN_Init(void);

#endif /* CAN_H_ */
