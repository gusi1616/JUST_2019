/*
 * uart.h
 *
 *  Created on: 19 nov. 2018
 *      Author: Daniel
 */

#ifndef UART_H_
#define UART_H_

#include "stm32l4xx_hal.h"
//#include <stdio.h>


#define ASCII_CLEAR "\033[2J"
#define ASCII_CLEARLINE "\033[2K"
#define ASCII_HOME "\033[H"
#define ASCII_UP "\033[1A"
#define ASCII_DOWN "\033[1B"
#define ASCII_FIRSTCOLUMN "\033[G"

void UART_Terminal_Init(UART_HandleTypeDef *huart);
void UART_Send_Int(UART_HandleTypeDef *huart, int16_t intIn);
void UART_Send_Hex(UART_HandleTypeDef *huart, int16_t intIn);
void UART_Send_Str(UART_HandleTypeDef *huart, const char *fmt, ...);
void UART_Send_Cmd(UART_HandleTypeDef *huart, char *cmd);
void UART_Send_IntAndStr(UART_HandleTypeDef *huart, int16_t intIn, char *str);
void UART_Clear_Terminal(UART_HandleTypeDef *huart);

#endif /* UART_H_ */
