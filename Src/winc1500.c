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
	switch (u8MsgType) {
	case M2M_WIFI_RESP_DEFAULT_CONNECT:
	{
		tstrM2MDefaultConnResp *pstrDefaultConnResp = (tstrM2MDefaultConnResp *)pvMsg;
		if (pstrDefaultConnResp->s8ErrorCode) {
			_status = WL_DISCONNECTED;
		}
	}
	break;

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			//SERIAL_PORT_MONITOR.println("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED");
			if (_mode == WL_STA_MODE && !_dhcp) {
				_status = WL_CONNECTED;

#ifdef CONF_PERIPH
				// WiFi led ON.
				m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
#endif
			} else if (_mode == WL_AP_MODE) {
				_status = WL_AP_CONNECTED;
			}
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			//SERIAL_PORT_MONITOR.println("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED");
			if (_mode == WL_STA_MODE) {
				_status = WL_DISCONNECTED;
				if (_dhcp) {
					_localip = 0;
					_submask = 0;
					_gateway = 0;
				}
				// Close sockets to clean state
				// Clients will need to reconnect once the physical link will be re-established
				for (int i = 0; i < MAX_SOCKET; i++) {
					WiFiSocket.close(i);
				}
			} else if (_mode == WL_AP_MODE) {
				_status = WL_AP_LISTENING;
			}
#ifdef CONF_PERIPH
			// WiFi led OFF (rev A then rev B).
			m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 1);
			m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1);
#endif
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		if (_mode == WL_STA_MODE) {
			tstrM2MIPConfig *pstrIPCfg = (tstrM2MIPConfig *)pvMsg;
			_localip = pstrIPCfg->u32StaticIP;
			_submask = pstrIPCfg->u32SubnetMask;
			_gateway = pstrIPCfg->u32Gateway;

			_status = WL_CONNECTED;

#ifdef CONF_PERIPH
			// WiFi led ON (rev A then rev B).
			m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
			m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0);
#endif
		}
		/*uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
				SERIAL_PORT_MONITOR.print("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is ");
				SERIAL_PORT_MONITOR.print(pu8IPAddress[0], 10);
				SERIAL_PORT_MONITOR.print(".");
				SERIAL_PORT_MONITOR.print(pu8IPAddress[1], 10);
				SERIAL_PORT_MONITOR.print(".");
				SERIAL_PORT_MONITOR.print(pu8IPAddress[2], 10);
				SERIAL_PORT_MONITOR.print(".");
				SERIAL_PORT_MONITOR.print(pu8IPAddress[3], 10);
				SERIAL_PORT_MONITOR.println("");*/
	}
	break;

	case M2M_WIFI_RESP_CURRENT_RSSI:
	{
		_resolve = *((int8_t *)pvMsg);
	}
	break;

	case M2M_WIFI_RESP_PROVISION_INFO:
	{
		tstrM2MProvisionInfo *pstrProvInfo = (tstrM2MProvisionInfo *)pvMsg;

		if (pstrProvInfo->u8Status == M2M_SUCCESS) {
			memset(_ssid, 0, M2M_MAX_SSID_LEN);
			memcpy(_ssid, (char *)pstrProvInfo->au8SSID, strlen((char *)pstrProvInfo->au8SSID));
			_mode = WL_STA_MODE;
			_localip = 0;
			_submask = 0;
			_gateway = 0;
			m2m_wifi_connect((char *)pstrProvInfo->au8SSID, strlen((char *)pstrProvInfo->au8SSID),
					pstrProvInfo->u8SecType, pstrProvInfo->au8Password, M2M_WIFI_CH_ALL);
		} else {
			_status = WL_PROVISIONING_FAILED;
			beginProvision();
		}
	}
	break;

	case M2M_WIFI_RESP_SCAN_DONE:
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
		if (pstrInfo->u8NumofCh >= 1) {
			_status = WL_SCAN_COMPLETED;
		}
	}
	break;

	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);
		memset(_scan_ssid, 0, M2M_MAX_SSID_LEN);
		if (scan_ssid_len) {
			memcpy(_scan_ssid, (const char *)pstrScanResult->au8SSID, scan_ssid_len);
		}
		if (_remoteMacAddress) {
			// reverse copy the remote MAC
			for(int i = 0; i < 6; i++) {
				_remoteMacAddress[i] = pstrScanResult->au8BSSID[5-i];
			}
		}
		_resolve = pstrScanResult->s8rssi;
		_scan_auth = pstrScanResult->u8AuthType;
		_scan_channel = pstrScanResult->u8ch;
		_status = WL_SCAN_COMPLETED;
	}
	break;

	case M2M_WIFI_RESP_CONN_INFO:
	{
		tstrM2MConnInfo	*pstrConnInfo = (tstrM2MConnInfo*)pvMsg;

		if (_remoteMacAddress) {
			// reverse copy the remote MAC
			for(int i = 0; i < 6; i++) {
				_remoteMacAddress[i] = pstrConnInfo->au8MACAddress[5-i];
			}
			_remoteMacAddress = 0;
		}

		strcpy((char *)_ssid, pstrConnInfo->acSSID);
	}
	break;

	case M2M_WIFI_RESP_GET_SYS_TIME:
	{
		if (_resolve != 0) {
			memcpy((tstrSystemTime *)_resolve, pvMsg, sizeof(tstrSystemTime));

			_resolve = 0;
		}
	}
	break;

	default:
		break;
	}
}

static void socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	eventCallback(sock, u8Msg, pvMsg);
}

void ping_cb(uint32 u32IPAddr, uint32 u32RTT, uint8 u8ErrorCode)
{
	handlePingResponse(u32IPAddr, u32RTT, u8ErrorCode);
}

void handlePingResponse(uint32 u32IPAddr, uint32 u32RTT, uint8 u8ErrorCode)
{
	if (PING_ERR_SUCCESS == u8ErrorCode) {
		// Ensure this ICMP reply comes from requested IP address
		if (_resolve == u32IPAddr) {
			_resolve = (uint32_t)u32RTT;
		} else {
			// Another network device replied to the our ICMP request
			_resolve = (uint32_t)WL_PING_DEST_UNREACHABLE;
		}
	} else if (PING_ERR_DEST_UNREACH == u8ErrorCode) {
		_resolve = (uint32_t)WL_PING_DEST_UNREACHABLE;
	} else if (PING_ERR_TIMEOUT == u8ErrorCode) {
		_resolve = (uint32_t)WL_PING_TIMEOUT;
	} else {
		_resolve = (uint32_t)WL_PING_ERROR;
	}
}

void setPins(int8_t cs, int8_t irq, int8_t rst, int8_t en)
{
	gi8Winc1501CsPin = cs;
	gi8Winc1501IntnPin = irq;
	gi8Winc1501ResetPin = rst;
	gi8Winc1501ChipEnPin = en;
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
	registerSocketCallback(socket_cb, resolve_cb);

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

void disconnect()
{
	// Close sockets to clean state
	for (int i = 0; i < MAX_SOCKET; i++) {
		WiFiSocket.close(i);
	}

	m2m_wifi_disconnect();

#ifdef CONF_PERIPH
	// WiFi led OFF (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 1);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1);
#endif
}

void end()
{
	// Close sockets to clean state
	for (int i = 0; i < MAX_SOCKET; i++) {
		WiFiSocket.close(i);
	}

	if (_mode == WL_AP_MODE) {
		m2m_wifi_disable_ap();
	} else {
		if (_mode == WL_PROV_MODE) {
			m2m_wifi_stop_provision_mode();
		}

		m2m_wifi_disconnect();
	}

#ifdef CONF_PERIPH
	// WiFi led OFF (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 1);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1);
#endif

	socketDeinit();

	m2m_wifi_deinit(NULL);

	nm_bsp_deinit();

	_mode = WL_RESET_MODE;
	_status = WL_NO_SHIELD;
	_init = 0;
}

uint8_t* macAddress(uint8_t *mac)
{
	m2m_wifi_get_mac_address(mac);
	byte tmpMac[6], i;

	m2m_wifi_get_mac_address(tmpMac);

	for(i = 0; i < 6; i++)
		mac[i] = tmpMac[5-i];

	return mac;
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
	m2m_wifi_set_sleep_mode(M2M_PS_H_AUTOMATIC, true);
}

void maxLowPowerMode(void)
{
	m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, true);
}

void noLowPowerMode(void)
{
	m2m_wifi_set_sleep_mode(M2M_NO_PS, false);
}

int ping(IPAddress host, uint8_t ttl)
{
#ifdef CONF_PERIPH
	// Network led ON (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16, 0);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 0);
#endif

	uint32_t dstHost = (uint32_t)host;
	_resolve = dstHost;

	if (m2m_ping_req((uint32_t)host, ttl, &ping_cb) < 0) {
#ifdef CONF_PERIPH
		// Network led OFF (rev A then rev B).
		m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16, 1);
		m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 1);
#endif
		//  Error sending ping request
		return WL_PING_ERROR;
	}

	// Wait for success or timeout:
	unsigned long start = millis();
	while (_resolve == dstHost && millis() - start < 5000) {
		m2m_wifi_handle_events(NULL);
	}

#ifdef CONF_PERIPH
	// Network led OFF (rev A then rev B).
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16, 1);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 1);
#endif

	if (_resolve == dstHost) {
		_resolve = 0;
		return WL_PING_TIMEOUT;
	} else {
		int rtt = (int)_resolve;
		_resolve = 0;
		return rtt;
	}
}

uint32_t getTime()
{
#ifdef WIFI_101_NO_TIME_H
#warning "No system <time.h> header included, WiFi.getTime() will always return 0"
	return 0;
#else
	tstrSystemTime systemTime;

	_resolve = (uint32_t)&systemTime;

	m2m_wifi_get_sytem_time();

	unsigned long start = millis();
	while (_resolve != 0 && millis() - start < 5000) {
		m2m_wifi_handle_events(NULL);
	}

	time_t t = 0;

	if (_resolve == 0 && systemTime.u16Year > 0) {
		struct tm tm;

		tm.tm_year = systemTime.u16Year - 1900;
		tm.tm_mon = systemTime.u8Month - 1;
		tm.tm_mday = systemTime.u8Day;
		tm.tm_hour = systemTime.u8Hour;
		tm.tm_min = systemTime.u8Minute;
		tm.tm_sec = systemTime.u8Second;
		tm.tm_isdst = -1;

		t = mktime(&tm);
	}

	_resolve = 0;

	return t;
#endif
}

SOCKET clientSocketHdl;
uint8 rxBuffer[256];
/* Socket event handler.
 */
void tcpClientSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	if(sock == clientSocketHdl)
	{
		if(u8Msg == SOCKET_MSG_CONNECT)
		{
			// Connect Event Handler.
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg*)pvMsg;
			if(pstrConnect->s8Error == 0)
			{
				// Perform data exchange.
				uint8acSendBuffer[256];
				uint16 u16MsgSize;
				// Fill in the acSendBuffer with some data here
				// send data
				send(clientSocketHdl, acSendBuffer, u16MsgSize, 0);
				// Recv response from server.
				recv(clientSocketHdl, rxBuffer, sizeof(rxBuffer), 0);
			}
			else
			{
				printf("TCP Connection Failed\n");
			}
		}
		else if(u8Msg == SOCKET_MSG_RECV)
		{
			tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
			if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
			{
				// Process the received message.
				// Close the socket.
				close(clientSocketHdl);
			}
		}
	}
}
// This is the DNS callback. The response of gethostbyname is here.
void dnsResolveCallback(uint8* pu8HostName, uint32 u32ServerIP)
{
	struct sockaddr_in strAddr;
	if(u32ServerIP != 0)
	{
		clientSocketHdl = socket(AF_INET,SOCK_STREAM,u8Flags);
		if(clientSocketHdl >= 0)
		{
			strAddr.sin_family = AF_INET;
			strAddr.sin_port = _htons(443);
			strAddr.sin_addr.s_addr = u32ServerIP;
			connect(clientSocketHdl, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
		}
	}
	else
	{
		printf("DNS Resolution Failed\n");
	}
}

