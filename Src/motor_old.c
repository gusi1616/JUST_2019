/*
 * motor_old.c
 *
 *  Created on: 10 mars 2019
 *      Author: Daniel
 */

uint16_t BatteryVoltage, BatteryCurrent, BatCurDir, MotorCurPeakAvg, FetTemp, MotorRotSpeed, PWM;
uint16_t PowerMode, MotorControlMode, AcceleratorPosition, RegenerationVrPosition, DigitSwPosition, OutputTargetValue, DriveActionStatus, RegenerationStatus;

void motorRequestFrames(uint8_t frame_request)
{
	// Fill CAN TxHeader and TxData
	TxHeader.StdId = 0x321;
	TxHeader.ExtId = MOTOR_CAN_REQ_EXT_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 1;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxData[0] = frame_request;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void Can_Extract_Data(uint32_t frameID)
{
	UART_Send_Cmd(&huart2, ASCII_HOME);
	if (frameID == MOTOR_CAN_RESP_FRAME_0_ID)
	{
		// Extract the bits
		BatteryVoltage = ((RxData[1U] & 0x03U) << 8U) | RxData[0U];
		BatteryCurrent = ((RxData[2U] & 0x07U) << 6U) | ((RxData[1U] & 0xFCU) >> 2U);
		BatCurDir = (RxData[2U] & 0x08U) >> 3U;
		MotorCurPeakAvg = ((RxData[3U] & 0x3FU) << 4U) | ((RxData[2U] & 0xF0) >> 4U);
		FetTemp = ((RxData[4U] & 0x07U) << 2U) | ((RxData[3U] & 0xC0) >> 6U);
		MotorRotSpeed = ((RxData[5U] & 0x7FU) << 5U) | ((RxData[4U] & 0xF8) >> 3U);
		PWM = ((RxData[7U] & 0x01U) << 9U) | RxData[6U] << 1U | ((RxData[5U] & 0x80U) >> 7U);

		// Convert to unit if necessary
		BatteryVoltage /= 2;
		FetTemp *= 5;
		PWM /= 2;
	}
	else if (frameID == MOTOR_CAN_RESP_FRAME_1_ID)
	{
		// Extract the bits
		PowerMode = RxData[0] & 0x01U;
		MotorControlMode = (RxData[0] & 0x02U) >> 1U;
		AcceleratorPosition = ((RxData[1] & 0x0FU) << 6U) | ((RxData[0] & 0xFCU) >> 2U);
		RegenerationVrPosition = ((RxData[2] & 0x3FU) << 4U) | ((RxData[1] & 0xF0U) >> 4U);
		DigitSwPosition = ((RxData[3] & 0x03U) << 2U) | ((RxData[2] & 0xC0U) >> 6U);
		OutputTargetValue = ((RxData[4] & 0x0FU) << 6U) | ((RxData[3] & 0xFCU) >> 2U);
		DriveActionStatus = (RxData[4] & 0x30U) >> 4U;
		RegenerationStatus = (RxData[4] & 0x40U) >> 5U;

		// Convert to unit if necessary
		AcceleratorPosition /= 2;
		RegenerationVrPosition /= 2;
		OutputTargetValue /= 2;
	}
	else if (frameID == MOTOR_CAN_RESP_FRAME_2_ID)
	{
		uint8_t OverHeatLevel = RxData[4] & 0x03U;
		uint32_t ErrorBits = RxData[3] | RxData[2] | RxData[1] | RxData[0];

		UART_Send_Cmd(&huart2, "\033[30B");
		debug("OverHeat Level: %d\r\n", OverHeatLevel);

		if (ErrorBits > 0)
		{
			debug("ERROR DETECTED\r\n");

			if (ErrorBits & (2^0))
			{
				debug("Analog Sensor Error\r\n");
			}
			if (ErrorBits & (2^1))
			{
				debug("Motor Current Sensor U Error\r\n");
			}
			if (ErrorBits & (2^2))
			{
				debug("Motor Current Sensor W Error\r\n");
			}
			if (ErrorBits & (2^3))
			{
				debug("FET Thermistor Error\r\n");
			}
			if (ErrorBits & (2^5))
			{
				debug("Battery Voltage Sensor Error\r\n");
			}
			if (ErrorBits & (2^6))
			{
				debug("Battery Current Sensor Error\r\n");
			}
			if (ErrorBits & (2^7))
			{
				debug("Battery Current Sensor Adjust Error\r\n");
			}
			if (ErrorBits & (2^8))
			{
				debug("Motor Current Sensor Adjust Error\r\n");
			}
			if (ErrorBits & (2^9))
			{
				debug("Accelerator Position Error\r\n");
			}
			if (ErrorBits & (2^11))
			{
				debug("Controller Voltage Sensor Error\r\n");
			}
			if (ErrorBits & (2^16))
			{
				debug("Power System Error\r\n");
			}
			if (ErrorBits & (2^19))
			{
				debug("Over Current Error\r\n");
			}
			if (ErrorBits & (2^21))
			{
				debug("Over Current Limit\r\n");
			}
			if (ErrorBits & (2^24))
			{
				debug("Motor System Error\r\n");
			}
			if (ErrorBits & (2^25))
			{
				debug("Motor Lock\r\n");
			}
			if (ErrorBits & (2^26))
			{
				debug("Hall Sensor Short\r\n");
			}
			if (ErrorBits & (2^27))
			{
				debug("Hall Sensor Open\r\n");
			}
		}
		else
		{
			debug("NO ERRORS\r\n");
		}
	}

#if (defined SOLAR_CAR)

	// Clear txBuffer
	memset(txBuffer,0,sizeof(txBuffer));

	/*txBuffer[16] = 0x00;
	txBuffer[15] = 0x12;
	txBuffer[14] = 0x30;
	txBuffer[13] = 0x11;
	txBuffer[12] = RxHeader.ExtId & 0xFF000000 >> 24;
	txBuffer[11] = RxHeader.ExtId & 0x00FF0000 >> 16;
	txBuffer[10] = RxHeader.ExtId & 0x0000FF00 >> 8;
	txBuffer[9] = RxHeader.ExtId & 0x000000FF;
	txBuffer[8] = RxData[7];
	txBuffer[7] = RxData[6];
	txBuffer[6] = RxData[5];
	txBuffer[5] = RxData[4];
	txBuffer[4] = RxData[3];
	txBuffer[3] = RxData[2];
	txBuffer[2] = RxData[1];
	txBuffer[1] = RxData[0];
	txBuffer[0] = 0x100 - ((txBuffer[16] + txBuffer[1] + txBuffer[2] + txBuffer[3] +
			txBuffer[4] + txBuffer[5] + txBuffer[6] + txBuffer[7] + txBuffer[8] +
			txBuffer[9] + txBuffer[10] + txBuffer[11] + txBuffer[12] + txBuffer[13] +
			txBuffer[14] + txBuffer[15]) % 0x100);*/

	txBuffer[11] = RxHeader.ExtId & 0xFF000000 >> 24;
	txBuffer[10] = RxHeader.ExtId & 0x00FF0000 >> 16;
	txBuffer[9] = RxHeader.ExtId & 0x0000FF00 >> 8;
	txBuffer[8] = RxHeader.ExtId & 0x000000FF;
	txBuffer[7] = RxData[7];
	txBuffer[6] = RxData[6];
	txBuffer[5] = RxData[5];
	txBuffer[4] = RxData[4];
	txBuffer[3] = RxData[3];
	txBuffer[2] = RxData[2];
	txBuffer[1] = RxData[1];
	txBuffer[0] = RxData[0];

	// Fill txBuffer
	//snprintf(txBuffer, 34, "0x%08x%08x%02x%02x%02x%02x%02x%02x%02x%02x", time_stamp, RxHeader.ExtId, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);

	debug("CAN Message Received: %s\r\n", txBuffer);

	UART_Send_Cmd(&huart2, ASCII_HOME);

	//int len = strlen(txBuffer);

	// Send to wifi
	sendto(udp_socket, (void *)txBuffer, 12, 0, (struct sockaddr*)&strAddr,
			sizeof(strAddr));
#endif
}

void motorPrintDebug(void)
{
	// Set cursor
	UART_Send_Cmd(&huart2, ASCII_HOME);
	UART_Send_Cmd(&huart2, "\033[10B");

	// Print values
	debug("Battery Voltage\t\t\t: %04d\r\n", BatteryVoltage);
	debug("Battery Current\t\t\t: %04d\r\n", BatteryCurrent);
	debug("Current Direction\t\t: %04d\r\n", BatCurDir);
	debug("Motor Current Peak Avg\t\t: %04d\r\n", MotorCurPeakAvg);
	debug("FET Temp\t\t\t: %04d\r\n", FetTemp);
	debug("Motor Rotation Speed\t\t: %04d\r\n", MotorRotSpeed);
	debug("PWM\t\t\t\t: %04d\r\n", PWM);

	// Set cursor
	UART_Send_Cmd(&huart2, ASCII_HOME);
	UART_Send_Cmd(&huart2, "\033[20B");

	// Print values
	debug("Power Mode\t\t\t\t: %04d\r\n", PowerMode);
	debug("Motor Control Mode\t\t\t: %04d\r\n", MotorControlMode);
	debug("Accelerator Position\t\t\t: %04d\r\n", AcceleratorPosition);
	debug("Regeneration VR Position\t\t: %04d\r\n", RegenerationVrPosition);
	debug("Digit SW Position\t\t\t: %04d\r\n", DigitSwPosition);
	debug("Output Target Value\t\t\t: %04d\r\n", OutputTargetValue);
	debug("Drive Action Status\t\t\t: %04d\r\n", DriveActionStatus);
	debug("Regeneration Status\t\t\t: %04d\r\n", RegenerationStatus);

	// Return cursor
	UART_Send_Cmd(&huart2, ASCII_HOME);
}

void Can_To_UART(void)
{
	uint32_t frameID = (rxBuffer[11] << 24) | (rxBuffer[10] << 16) | (rxBuffer[9] << 8) | rxBuffer[8];

	RxData[7] = rxBuffer[7];
	RxData[6] = rxBuffer[6];
	RxData[5] = rxBuffer[5];
	RxData[4] = rxBuffer[4];
	RxData[3] = rxBuffer[3];
	RxData[2] = rxBuffer[2];
	RxData[1] = rxBuffer[1];
	RxData[0] = rxBuffer[0];

	Can_Extract_data(frameID);
}
