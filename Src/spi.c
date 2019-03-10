/*
 * spi.c
 *
 *  Created on: 10 mars 2019
 *      Author: Daniel
 */


void spi_dummy_send(void)
{
	HAL_SPI_Init(&hspi_winc);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi_winc, 0xFF, 1, 100);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_SET);
}
