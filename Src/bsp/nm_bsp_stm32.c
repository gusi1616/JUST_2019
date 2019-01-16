/**
 *
 * \file
 *
 * \brief This module contains STM32F429 BSP APIs implementation.
 *
 * Copyright (c) 2018 JU Solar Team. All rights reserved.
*/

#include <bsp/nm_bsp_stm32.h>
#include "bsp/nm_bsp.h"
#include "common/nm_common.h"
#include "stm32l4xx_hal.h"

static tpfNmBspIsr gpfIsr;

static void chip_isr(void)
{
	if (gpfIsr) {
		gpfIsr();
	}
}

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(WINC_GPIO_Port, WINC_CFG_Pin|WINC_EN_Pin|WINC_WAKE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(WINC_GPIO_Port, WINC_CS_Pin|WINC_RST_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : WINC_IRQ_Pin */
	GPIO_InitStruct.Pin = WINC_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WINC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : WINC_CFG_Pin WINC_EN_Pin WINC_RST_Pin WINC_CS_Pin
	                           WINC_WAKE_Pin */
	GPIO_InitStruct.Pin = WINC_CFG_Pin|WINC_EN_Pin|WINC_RST_Pin|WINC_CS_Pin
			|WINC_WAKE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void deinit_chip_pins(void)
{
	HAL_GPIO_DeInit(GPIOA, WINC_CFG_Pin|WINC_EN_Pin|WINC_RST_Pin|WINC_WAKE_Pin|WINC_CS_Pin);
}

/*
 *	@fn		nm_bsp_init
 *	@brief	Initialize BSP
 *	@return	0 in case of success and -1 in case of failure
 */
sint8 nm_bsp_init(void)
{
	gpfIsr = NULL;

	init_chip_pins();

	nm_bsp_reset();

	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_deinit
 *	@brief	De-iInitialize BSP
 *	@return	0 in case of success and -1 in case of failure
 */
sint8 nm_bsp_deinit(void)
{
	deinit_chip_pins();

	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
	HAL_GPIO_WritePin(WINC_GPIO_Port, WINC_RST_Pin, GPIO_PIN_RESET);
	nm_bsp_sleep(100);
	HAL_GPIO_WritePin(WINC_GPIO_Port, WINC_RST_Pin, GPIO_PIN_SET);
	nm_bsp_sleep(100);
}

/*
 *	@fn		nm_bsp_sleep
 *	@brief	Sleep in units of mSec
 *	@param[IN]	u32TimeMsec
 *				Time in milliseconds
 */
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	HAL_Delay(u32TimeMsec);
}

/*
 *	@fn		nm_bsp_register_isr
 *	@brief	Register interrupt service routine
 *	@param[IN]	pfIsr
 *				Pointer to ISR handler
 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;
	//attachInterruptMultiArch(WINC_IRQ_Pin, chip_isr, FALLING);
}

/*
 *	@fn		nm_bsp_interrupt_ctrl
 *	@brief	Enable/Disable interrupts
 *	@param[IN]	u8Enable
 *				'0' disable interrupts. '1' enable interrupts
 */
void nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	/*if (u8Enable) {
		attachInterruptMultiArch(WINC_IRQ_Pin, chip_isr, FALLING);
	} else {
		detachInterruptMultiArch(WINC_IRQ_Pin);
	}*/
}
