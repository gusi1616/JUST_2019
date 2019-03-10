/*
 * motor_old.h
 *
 *  Created on: 10 mars 2019
 *      Author: Daniel
 */

#ifndef MOTOR_OLD_H_
#define MOTOR_OLD_H_

/** CAN ID */
#define MOTOR_CAN_REQ_EXT_ID 0x08f91540
#define MOTOR_CAN_RESP_FRAME_0_ID 0x08850245
#define MOTOR_CAN_RESP_FRAME_1_ID 0x08950245
#define MOTOR_CAN_RESP_FRAME_2_ID 0x08A50245

#define MOTOR_CAN_REQ_FRAME_0 1
#define MOTOR_CAN_REQ_FRAME_1 2
#define MOTOR_CAN_REQ_FRAME_2 4

void Can_To_UART(void);
void Can_Extract_Data(uint32_t frameID);
void motorPrintDebug(void);

#endif /* MOTOR_OLD_H_ */
