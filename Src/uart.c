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
void UART_Send_Str(UART_HandleTypeDef *huart, const char *fmt, ...)
{
	/*char buffer[500];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	HAL_UART_Transmit(huart, buffer, strlen(buffer), 100);*/
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
