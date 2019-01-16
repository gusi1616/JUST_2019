/*
  WiFi.cpp - Library for Arduino Wifi shield.
  Copyright (c) 2011-2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include "winc1500.h"
#include "bsp/nm_bsp.h"
#include "bsp/nm_bsp_stm32.h"
#include "driver/m2m_periph.h"
#include "driver/m2m_ssl.h"
#include "driver/m2m_wifi.h"

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
//	switch (u8MsgType) {
//	case M2M_WIFI_RESP_SCAN_DONE:
//	{
//		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
//		scan_request_index = 0;
//		if (pstrInfo->u8NumofCh >= 1) {
//			m2m_wifi_req_scan_result(scan_request_index);
//			scan_request_index++;
//		} else {
//			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
//		}
//
//		break;
//	}
//
//	case M2M_WIFI_RESP_SCAN_RESULT:
//	{
//		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
//		uint16_t demo_ssid_len;
//		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);
//
//		/* display founded AP. */
//		printf("[%d] SSID:%s\r\n", scan_request_index, pstrScanResult->au8SSID);
//
//		num_founded_ap = m2m_wifi_get_num_ap_found();
//		if (scan_ssid_len) {
//			/* check same SSID. */
//			demo_ssid_len = strlen((const char *)MAIN_WLAN_SSID);
//			if
//			(
//					(demo_ssid_len == scan_ssid_len) &&
//					(!memcmp(pstrScanResult->au8SSID, (uint8_t *)MAIN_WLAN_SSID, demo_ssid_len))
//			) {
//				/* A scan result matches an entry in the preferred AP List.
//				 * Initiate a connection request.
//				 */
//				printf("Found %s \r\n", MAIN_WLAN_SSID);
//				m2m_wifi_connect((char *)MAIN_WLAN_SSID,
//						sizeof(MAIN_WLAN_SSID),
//						MAIN_WLAN_AUTH,
//						(void *)MAIN_WLAN_PSK,
//						M2M_WIFI_CH_ALL);
//				break;
//			}
//		}
//
//		if (scan_request_index < num_founded_ap) {
//			m2m_wifi_req_scan_result(scan_request_index);
//			scan_request_index++;
//		} else {
//			printf("can not find AP %s\r\n", MAIN_WLAN_SSID);
//			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
//		}
//
//		break;
//	}
//
//	case M2M_WIFI_RESP_CON_STATE_CHANGED:
//	{
//		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
//		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
//			m2m_wifi_request_dhcp_client();
//		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
//			printf("Wi-Fi disconnected\r\n");
//
//			/* Request scan. */
//			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
//		}
//
//		break;
//	}
//
//	case M2M_WIFI_REQ_DHCP_CONF:
//	{
//		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
//		printf("Wi-Fi connected\r\n");
//		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
//				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
//		break;
//	}
//
//	default:
//	{
//		break;
//	}
//	}
}

int8_t wincInit(void)
{
	tstrWifiInitParam param;
	int8_t ret;

	// Initialize the WiFi BSP:
	nm_bsp_init();

	// Initialize WiFi module and register status callback:
	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;


	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret && M2M_ERR_FW_VER_MISMATCH != ret) {
#ifdef CONF_PERIPH
		if (ret != M2M_ERR_INVALID) {
			// Error led ON (rev A then rev B).
			m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18, 0);
			m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO6, 1);
		}
#endif
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		return ret;
	}

	return ret;
}

char* getFirmwareVersion()
{
	tstrM2mRev rev;
	char version[9];

	nm_get_firmware_info(&rev);
	if (rev.u8FirmwareMajor != rev.u8DriverMajor && rev.u8FirmwareMinor != rev.u8DriverMinor) {
		sprintf(version, "-Err-");
	}
	else {
		sprintf(version, "%d.%d.%d", rev.u8FirmwareMajor, rev.u8FirmwareMinor, rev.u8FirmwarePatch);
	}
	return version;
}

int8_t wincStartConnect(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo, uint8_t channel)
{
	// Initialize socket API and register socket callback:
	socketInit();

	int8_t status = m2m_wifi_connect((char*)ssid, sizeof(ssid), u8SecType,
			(void*)pvAuthInfo, channel);

	return status;
}

uint8_t wincStartAP(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo, uint8_t channel)
{
	tstrM2MAPConfig strM2MAPConfig;

	if (channel < 1) {
		channel = 1; // channel 1 is the minimum channel
	}

	// Enter Access Point mode:
	memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
	strcpy((char *)&strM2MAPConfig.au8SSID, ssid);
	strM2MAPConfig.u8ListenChannel = channel;
	strM2MAPConfig.u8SecType = u8SecType;
	strM2MAPConfig.au8DHCPServerIP[0] = 192;
	strM2MAPConfig.au8DHCPServerIP[1] = 168;
	strM2MAPConfig.au8DHCPServerIP[2] = 1;
	strM2MAPConfig.au8DHCPServerIP[3] = 1;

	if (strM2MAPConfig.au8DHCPServerIP[3] == 100) {
		// limitation of WINC1500 firmware, IP address of client is always x.x.x.100
		return WL_AP_FAILED;
	}

	if (u8SecType == M2M_WIFI_SEC_WEP) {
		tstrM2mWifiWepParams* wep_params = (tstrM2mWifiWepParams*)pvAuthInfo;

		strM2MAPConfig.u8KeyIndx = wep_params->u8KeyIndx;
		strM2MAPConfig.u8KeySz = wep_params->u8KeySz;
		strcpy((char*)strM2MAPConfig.au8WepKey, (char *)wep_params->au8WepKey);
	}

	if (u8SecType == M2M_WIFI_SEC_WPA_PSK) {
		strM2MAPConfig.u8KeySz = strlen((char*)pvAuthInfo);
		strcpy((char*)strM2MAPConfig.au8Key, (char *)pvAuthInfo);
	}

	if (m2m_wifi_enable_ap(&strM2MAPConfig) < 0) {
		return WL_AP_FAILED;
	}

#ifdef CONF_PERIPH
	// WiFi led ON (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0);
#endif

	return WL_AP_CONNECTED;
}

uint8_t wincStartProvision(const char *ssid, const char *url, uint8_t channel)
{
	tstrM2MAPConfig strM2MAPConfig;

	// Enter Provision mode:
	memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
	strcpy((char *)&strM2MAPConfig.au8SSID, ssid);
	strM2MAPConfig.u8ListenChannel = channel;
	strM2MAPConfig.u8SecType = M2M_WIFI_SEC_OPEN;
	strM2MAPConfig.u8SsidHide = SSID_MODE_VISIBLE;
	strM2MAPConfig.au8DHCPServerIP[0] = 192;
	strM2MAPConfig.au8DHCPServerIP[1] = 168;
	strM2MAPConfig.au8DHCPServerIP[2] = 1;
	strM2MAPConfig.au8DHCPServerIP[3] = 1;

	if (m2m_wifi_start_provision_mode((tstrM2MAPConfig *)&strM2MAPConfig, (char*)url, 1) < 0) {
		return WL_PROVISIONING_FAILED;
	}

#ifdef CONF_PERIPH
	// WiFi led ON (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0);
#endif

	return WL_PROVISIONING;
}

void config_ip(uint32_t local_ip, uint32_t dns_server, uint32_t gateway, uint32_t subnet)
{
	tstrM2MIPConfig conf;

	conf.u32DNS = dns_server;
	conf.u32Gateway = gateway;
	conf.u32StaticIP = local_ip;
	conf.u32SubnetMask = subnet;
	m2m_wifi_enable_dhcp(0); // disable DHCP
	m2m_wifi_set_static_ip(&conf);
}

void set_device_name(const char* name)
{
	m2m_wifi_set_device_name((uint8 *)name, strlen(name));
}

int8_t scanNetworks()
{
	// Start scan:
	if (m2m_wifi_request_scan(M2M_WIFI_CH_ALL) < 0) {
		return 0;
	}


	return m2m_wifi_get_num_ap_found();
}

void refresh(void)
{
	// Update state machine:
	m2m_wifi_handle_events(NULL);
}

void lowPowerMode(void)
{
	m2m_wifi_set_sleep_mode(M2M_PS_H_AUTOMATIC, 1);
}

void maxLowPowerMode(void)
{
	m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
}

void noLowPowerMode(void)
{
	m2m_wifi_set_sleep_mode(M2M_NO_PS, 0);
}

void spi_dummy_send(void)
{
	HAL_SPI_Init(&hspi_winc);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi_winc, 0xFF, 1, 100);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_SET);
}
