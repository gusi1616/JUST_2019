/*
 * uart.c
 *
 *  Created on: 19 nov. 2018
 *      Author: Daniel
 */

#include "uart.h"


void UART_Terminal_Init(UART_HandleTypeDef *huart)
{
	UART_Clear_Terminal(huart);
	UART_Send_Str(huart, "Hello and welcome to this lab4!\n\r");
}
void UART_Send_Int(UART_HandleTypeDef *huart, int16_t intIn)
{
	unsigned char intOut[sizeof(intIn)];
	sprintf(intOut, "%d", intIn);
	HAL_UART_Transmit(huart, intOut, strlen(intOut), 100);
}
void UART_Send_Hex(UART_HandleTypeDef *huart, int16_t intIn)
{
	unsigned char intOut[sizeof(intIn)];
	itoa(intIn, intOut, 16);
	HAL_UART_Transmit(huart, intOut, strlen(intOut), 100);
}
void UART_Send_Str(UART_HandleTypeDef *huart, char *str)
{
	HAL_UART_Transmit(huart, str, strlen(str), 100);
}
void UART_Send_Cmd(UART_HandleTypeDef *huart, char *cmd)
{
	HAL_UART_Transmit(huart, cmd, strlen(cmd), 100);
}
void UART_Send_IntAndStr(UART_HandleTypeDef *huart, int16_t intIn, char *str)
{
	UART_Send_Int(huart, intIn);
	UART_Send_Str(huart, str);
}
void UART_Clear_Terminal(UART_HandleTypeDef *huart)
{
	UART_Send_Cmd(huart, ASCII_CLEAR);
	UART_Send_Cmd(huart, ASCII_HOME);
}
