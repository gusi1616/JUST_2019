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

static SOCKET udp_socket = -1;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		//debug("State: M2M_WIFI_RESP_CON_STATE_CHANGED\r\n");
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			debug("\033[H");
			debug("\033[5B");
			debug("Wi-Fi Status: Connected\r\n");
			m2m_wifi_enable_dhcp(0);
#if (defined USE_STATIONMODE)
			tstrM2MIPConfig stnConfig;
			stnConfig.u32StaticIP = _htonl(MAIN_WIFI_M2M_CLIENT_IP_ALT);
			stnConfig.u32SubnetMask = _htonl(0xFFFFFF00); //corresponds to 255.255.255.0
			m2m_wifi_set_static_ip(&stnConfig);
#endif
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			debug("\033[H");
			debug("\033[5B");
			debug("Wi-Fi Status: Disconnected\r\n");
#if (defined USE_STATIONMODE)
			/* Reconnect. */
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH,
					(char *)MAIN_WLAN_PSK, 1);
#endif
		}
		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		//debug("State: M2M_WIFI_REQ_DHCP_CONF\r\n");
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		debug("\033[H");
		debug("\033[4B");
		debug("Wi-Fi configured\r\n");
#if (defined USE_APMODE)
		debug("Client IP is %d.%d.%d.%d\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
#elif (defined USE_STATIONMODE)
		// Get the current AP info
		//m2m_wifi_get_connection_info();
		m2m_wifi_req_curr_rssi();
#endif
		// Initialize socket
#if (defined USE_CLIENT)
		udpClientStart((char*)MAIN_WIFI_M2M_SERVER_IP, MAIN_WIFI_M2M_SERVER_PORT);
#elif (defined USE_SERVER)
		udpStartServer(MAIN_WIFI_M2M_SERVER_PORT);
#endif
		break;
	}

	/*case M2M_WIFI_RESP_IP_CONFIGURED:
	{
		debug("State: M2M_WIFI_RESP_IP_CONFIGURED\r\n");
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		debug("Wi-Fi RESP IP configured\r\n");
		debug("Wi-Fi Static IP is %d.%d.%d.%d\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		break;
	}*/

	case M2M_WIFI_RESP_CURRENT_RSSI:
	{
		sint8 *rssi = (sint8*)pvMsg;
		global_rssi = *rssi;
		debug("\033[H");
		debug("\033[6B");
		debug("Signal Strength : %d\n", global_rssi);
		break;
	}

	case M2M_WIFI_RESP_CONN_INFO:
	{
		tstrM2MConnInfo *pstrConnInfo = (tstrM2MConnInfo*)pvMsg;

		debug("CONNECTED AP INFO\n");
		debug("SSID : %s\n",pstrConnInfo->acSSID);
		debug("SEC TYPE : %d\n",pstrConnInfo->u8SecType);
		debug("Signal Strength : %d\n", pstrConnInfo->s8RSSI);
		debug("Local IP Address : %d.%d.%d.%d\n", pstrConnInfo->au8IPAddr[0] , pstrConnInfo->au8IPAddr[1], pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]);

		UART_Send_Cmd(&huart2, ASCII_HOME);
		break;
	}

	default:
	{
		break;
	}
	}
}

int8_t wincInit(void)
{
	tstrWifiInitParam param;
	int8_t ret;

	// Init SPI with dummy byte
	spi_dummy_send();

	// Initialize the WiFi BSP:
	nm_bsp_init();

	// Initialize WiFi parameters structure
	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;

	// Initialize WiFi module and register status callback:
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		debug("WINC1500 Wifi Initialization failed\r\n");
	}
	else
	{
		debug("WINC1500 Wifi Initialized\r\n");
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

	return M2M_SUCCESS;
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



void udpClientSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	if((u8Msg == SOCKET_MSG_RECV) || (u8Msg == SOCKET_MSG_RECVFROM))
	{
		debug("State: SOCKET_MSG_RECV\r\n");

		tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
		{
			uint16 len;

			// Clear txBuffer
			memset(txBuffer,0,sizeof(txBuffer));

			// Format a message in the txBuffer and put its length in len
			txBuffer[0] = 0;
			txBuffer[1] = 1;
			txBuffer[2] = 2;
			len = 3;

			// Handle received data.
			debug("Message received: %s\r\n", rxBuffer);

			//sendto(udp_socket, txBuffer, len, 0,
			//	(struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));

			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
		}
	}
	else if (u8Msg == SOCKET_MSG_SENDTO)
	{
		//debug("CAN Message sent via WiFi\r\n");
	}
}

void udpClientStart(char *pcServerIP, uint16_t port)
{
	// Register socket application callbacks.
	registerSocketCallback(udpClientSocketEventHandler, NULL);

	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if(udp_socket >= 0)
	{
		uint16 len;
		strAddr.sin_family = AF_INET;
		strAddr.sin_port = _htons(port);
		strAddr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP); //nmi_inet_addr(pcServerIP);

		// Format some message in the txBuffer and put its length in len
		strcpy(txBuffer, "Solar Car Connected");
		len = strlen(txBuffer);

		sendto(udp_socket, (void *)txBuffer, len, 0, (struct sockaddr*)&strAddr,
				sizeof(strAddr));

		recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
	}
	else
	{
		debug("Failed to create client socket\r\n");
	}
}

static const char *inet_ntop4(const unsigned char *src, char *dst, size_t size)
{
	static const char fmt[] = "%u.%u.%u.%u";
	char tmp[sizeof "255.255.255.255"];

	sprintf(tmp, fmt, src[0], src[1], src[2], src[3]);
	if (strlen(tmp) > size) {
		return (NULL);
	}
	strcpy(dst, tmp);
	return (dst);
}

void udpServerSocketEventHandler(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(u8Msg == SOCKET_MSG_BIND)
	{
		//debug("State: SOCKET_MSG_BIND\r\n");

		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg*)pvMsg;
		if(pstrBind->status == 0)
		{
			//debug("Bind Success\r\n");

			// call Recv
			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
		}
		else
		{
			//debug("Bind Failed\r\n");
		}
	}
	else if(u8Msg == SOCKET_MSG_RECVFROM)
	{
		//debug("State: SOCKET_MSG_RECVFROM\r\n");

		tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
		{
			char remote_addr[16];
			inet_ntop4((unsigned char*)&pstrRecvMsg->strRemoteAddr.sin_addr, (char*)&remote_addr, 16);

			// Handle received data.
			//debug("Message received from %s: %s\r\n", remote_addr, rxBuffer);

			Can_To_UART();

			//HAL_UART_Transmit(&huart2, rxBuffer, strlen(rxBuffer), 100);
			//UART_Send_Cmd(&huart2, "\r\n");

			// Clear txBuffer
			memset(txBuffer,0,sizeof(txBuffer));

			// Fill in the txBuffer with some data
			strcpy((char *)txBuffer, "Ack");

			// Send some data to the same address.
			sendto(udp_socket, txBuffer, strlen(txBuffer), 0,
					&pstrRecvMsg->strRemoteAddr, sizeof(pstrRecvMsg->strRemoteAddr));

			// call Recv
			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
		}
	}
}

void udpStartServer(uint16 u16ServerPort)
{
	// Register socket application callbacks.
	registerSocketCallback(udpServerSocketEventHandler, NULL);

	// Create the server listen socket.
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if(udp_socket >= 0)
	{
		strAddr.sin_family = AF_INET;
		strAddr.sin_port = _htons(u16ServerPort);
		strAddr.sin_addr.s_addr = 0; //INADDR_ANY

		int ret = bind(udp_socket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));

		if (ret == M2M_SUCCESS)
		{
			//debug("WINC1500 Wifi Bind Success\r\n");
		}
		else
		{
			//debug("WINC1500 Wifi Bind Failed\r\n");
		}
	}
}
