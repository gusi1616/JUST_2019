/*
 * winc1500.h
 *
 *  Created on: 18 dec. 2018
 *      Author: Daniel
 */

#ifndef WINC1500_H_
#define WINC1500_H_

#define WIFI_FIRMWARE_LATEST_MODEL_A "19.4.4"
#define WIFI_FIRMWARE_LATEST_MODEL_B "19.5.4"

// for backwards compatibility
#define WIFI_FIRMWARE_REQUIRED WIFI_FIRMWARE_LATEST_MODEL_B

#include "driver/m2m_wifi.h"
#include "stm32l4xx_hal.h"

/*
#include "WiFiClient.h"
#include "WiFiSSLClient.h"
#include "WiFiServer.h"
*/

typedef enum {
	WL_NO_SHIELD = 255,
	WL_IDLE_STATUS = 0,
	WL_NO_SSID_AVAIL,
	WL_SCAN_COMPLETED,
	WL_CONNECTED,
	WL_CONNECT_FAILED,
	WL_CONNECTION_LOST,
	WL_DISCONNECTED,
	WL_AP_LISTENING,
	WL_AP_CONNECTED,
	WL_AP_FAILED,
	WL_PROVISIONING,
	WL_PROVISIONING_FAILED
} wl_status_t;

/* Encryption modes */
enum wl_enc_type {  /* Values map to 802.11 encryption suites... */
	ENC_TYPE_WEP  = M2M_WIFI_SEC_WEP,
	ENC_TYPE_TKIP = M2M_WIFI_SEC_WPA_PSK,
	ENC_TYPE_CCMP = M2M_WIFI_SEC_802_1X,
	/* ... except these two, 7 and 8 are reserved in 802.11-2007 */
	ENC_TYPE_NONE = M2M_WIFI_SEC_OPEN,
	ENC_TYPE_AUTO = M2M_WIFI_SEC_INVALID
};

typedef enum {
	WL_RESET_MODE = 0,
	WL_STA_MODE,
	WL_PROV_MODE,
	WL_AP_MODE
} wl_mode_t;

typedef enum {
	WL_PING_DEST_UNREACHABLE = -1,
	WL_PING_TIMEOUT = -2,
	WL_PING_UNKNOWN_HOST = -3,
	WL_PING_ERROR = -4
} wl_ping_result_t;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
void setPins(int8_t cs, int8_t irq, int8_t rst, int8_t en);

int8_t wincInit(void);
int8_t wincStartConnect(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo, uint8_t channel);
uint8_t wincStartAP(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo, uint8_t channel);
uint8_t wincStartProvision(const char *ssid, const char *url, uint8_t channel);

char* getFirmwareVersion(void);
void config_ip(uint32_t local_ip, uint32_t dns_server, uint32_t gateway, uint32_t subnet);
void set_device_name(const char* name);

void disconnect();
void end();

uint8_t *macAddress(uint8_t *mac);

int8_t scanNetworks();

int hostByName(const char* hostname, uint32_t result);

int ping(const char* hostname, uint8_t ttl);

unsigned long getTime();

void refresh(void);

void lowPowerMode(void);
void maxLowPowerMode(void);
void noLowPowerMode(void);

void handlePingResponse(uint32 u32IPAddr, uint32 u32RTT, uint8 u8ErrorCode);

#endif /* WINC1500_H_ */
