/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */


/*
 * Variants may define an alternative SPI instace to use for WiFi101.
 * If not defined the following defaults are used:
 *   WINC1501_SPI    - SPI
 */



#include "bsp/nm_bsp.h"
#include "bsp/nm_bsp_stm32.h"
#include "common/nm_common.h"
#include "bus_wrapper/nm_bus_wrapper.h"
#include "stm32f4xx_hal.h"


#define NM_BUS_MAX_TRX_SZ	256

extern SPI_HandleTypeDef hspi_winc;

tstrNmBusCapabilities egstrNmBusCapabilities =
{
		NM_BUS_MAX_TRX_SZ
};

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint8 txd_data = 0;
	uint8 rxd_data = 0;

	if (!pu8Mosi) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	else if(!pu8Miso) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}
	else {
		return M2M_ERR_BUS_FAIL;
	}

	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_RESET);
	while (u16Sz) {
		txd_data = *pu8Mosi;
		HAL_SPI_TransmitReceive(&hspi_winc,&txd_data,&rxd_data,1,1000);
		*pu8Miso = rxd_data;
		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_SET);

	return M2M_SUCCESS;
}


/*
 *	@fn		nm_bus_init
 *	@brief	Initialize the bus wrapper
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 *	@author	M.S.M
 *	@date	28 oct 2013
 *	@version	1.0
 */
sint8 nm_bus_init(void)
{
	sint8 result = M2M_SUCCESS;

	/* SPI parameter configuration*/
//	hspi_winc.Instance = SPI_WINC;
//	hspi_winc.Init.Mode = SPI_MODE_MASTER;
//	hspi_winc.Init.Direction = SPI_DIRECTION_2LINES;
//	hspi_winc.Init.DataSize = SPI_DATASIZE_8BIT;
//	hspi_winc.Init.CLKPolarity = SPI_POLARITY_LOW;
//	hspi_winc.Init.CLKPhase = SPI_PHASE_1EDGE;
//	hspi_winc.Init.NSS = SPI_NSS_SOFT;
//	hspi_winc.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
//	hspi_winc.Init.FirstBit = SPI_FIRSTBIT_MSB;
//	hspi_winc.Init.TIMode = SPI_TIMODE_DISABLE;
//	hspi_winc.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//	hspi_winc.Init.CRCPolynomial = 10;
//	if (HAL_SPI_Init(&hspi_winc) != HAL_OK)
//	{
//		_Error_Handler(__FILE__, __LINE__);
//	}
//
//	/* Configure CS PIN. */
//	HAL_GPIO_WritePin(WINC_GPIO_Port, WINC_CS_Pin, GPIO_PIN_SET);

	/* Reset WINC1500. */
	nm_bsp_reset();
	nm_bsp_sleep(1);

	return result;
}

/*
 *	@fn		nm_bus_ioctl
 *	@brief	send/receive from the bus
 *	@param[IN]	u8Cmd
 *					IOCTL command for the operation
 *	@param[IN]	pvParameter
 *					Arbitrary parameter depenging on IOCTL
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 *	@author	M.S.M
 *	@date	28 oct 2013
 *	@note	For SPI only, it's important to be able to send/receive at the same time
 *	@version	1.0
 */
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
	case NM_BUS_IOCTL_RW: {
		tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
		s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
	}
	break;
	default:
		s8Ret = -1;
		M2M_ERR("invalide ioclt cmd\n");
		break;
	}

	return s8Ret;
}

/*
 *	@fn		nm_bus_deinit
 *	@brief	De-initialize the bus wrapper
 *	@author	M.S.M
 *	@date	28 oct 2013
 *	@version	1.0
 */
sint8 nm_bus_deinit(void)
{
	if (HAL_SPI_DeInit(&hspi_winc) != HAL_OK)
		return M2M_ERR_BUS_FAIL;
	return M2M_SUCCESS;
}

/*
 *	@fn			nm_bus_reinit
 *	@brief		re-initialize the bus wrapper
 *	@param [in]	void *config
 *					re-init configuration data
 *	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 *	@author		Dina El Sissy
 *	@date		19 Sept 2012
 *	@version	1.0
 */
sint8 nm_bus_reinit(void)
{
	nm_bus_deinit();
	nm_bus_init();
	return M2M_SUCCESS;
}

