/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "winc1500.h"
#include <string.h>
#include "driver/m2m_wifi.h"
#include "driver/nmasic.h"
#include "driver/m2m_hif.h"
#include "socket/socket.h"
#include "driver/m2m_periph.h"
#include "bsp/nm_bsp_stm32.h"
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** Security mode Supported */
//#define USE_WPAPSK 1 /* WPA/WPA2 PSK Security Mode*/
// #define USE_WEP 2 /* WEP Security Mode*/
#define USE_OPEN 3 /* No Security or OPEN Authentication Mode*/

/** AP mode Settings */
#define MAIN_WLAN_SSID "WINC1500_AP" /* < SSID */
#if (defined USE_WPAPSK)
#define MAIN_WLAN_AUTH M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_WPA_PSK "1234567890" /* < Security Key in WPA PSK Mode */
#elif (defined USE_WEP)
#define MAIN_WLAN_AUTH M2M_WIFI_SEC_WEP /* < Security manner */
#define MAIN_WLAN_WEP_KEY "1234567890" /* < Security Key in WEP Mode */
#define MAIN_WLAN_WEP_KEY_INDEX (0)
#elif (defined USE_OPEN)
#define MAIN_WLAN_AUTH M2M_WIFI_SEC_OPEN /* < Security manner */
#endif
#define MAIN_WLAN_CHANNEL (M2M_WIFI_CH_6) /* < Channel number */

#define MAIN_WLAN_PSK "WIFI_PASSWD" /* < Password for Destination SSID */

/** Set CAR or SUPPORT **/
//#define SOLAR_CAR
#define SUPPORT_CAR
//#define GPS_TEST

#if (defined SOLAR_CAR)
#define WIFI_ON 1
#define CAN_ON 0
#define USE_STATIONMODE /** Set WIFI Mode **/
#define USE_CLIENT /** Set UDP Mode **/
#elif (defined SUPPORT_CAR)
#define WIFI_ON 1
#define CAN_ON 0
#define USE_APMODE /** Set WIFI Mode **/
#define USE_SERVER /** Set UDP Mode **/
#elif (defined GPS_TEST)
#define WIFI_ON 0
#define CAN_ON 0
#define GPS_ON 1
#endif


/** Set Debug Mode **/
#define DEBUG 0

#define false 0
#define true 1

/** Port to use for UDP socket */
#define UDP_PORT                       (6666)

/** Max size of incoming packets */
#define MAIN_WIFI_M2M_BUFFER_SIZE		20

/** CAN ID */
#define MOTOR_CAN_REQ_EXT_ID 0x08f91540
#define MOTOR_CAN_RESP_FRAME_0_ID 0x08850245
#define MOTOR_CAN_RESP_FRAME_1_ID 0x08950245
#define MOTOR_CAN_RESP_FRAME_2_ID 0x08A50245

#define MOTOR_CAN_REQ_FRAME_0 1
#define MOTOR_CAN_REQ_FRAME_1 2
#define MOTOR_CAN_REQ_FRAME_2 4
#define MAIN_WIFI_M2M_BROADCAST_IP 0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_CLIENT_IP  0xc0a80164  // 192.168.1.100
#define MAIN_WIFI_M2M_CLIENT_IP_ALT 0xc0a80184 // 192.168.1.132
#define MAIN_WIFI_M2M_SERVER_IP  0xC0A80101  // 192.168.1.1
#define MAIN_WIFI_M2M_SERVER_PORT (6666)

#define UART_BUFFER_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/** Sockets for UDP communication */
static SOCKET udp_socket = -1;

struct sockaddr_in strAddr;
struct sockaddr_in broadcastAddr;

/** Wi-Fi connection state */
static volatile uint8_t wifi_connected;
int8_t global_rssi;

/** Receive buffer definition. */
uint8_t rxBuffer[256], txBuffer[256];
uint8_t uart3RxBuffer[UART_BUFFER_SIZE], uart3TxBuffer[UART_BUFFER_SIZE];
uint8_t uart3RxITFlag = 0;

int8_t result = 0;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint32_t time_stamp;

uint16_t BatteryVoltage, BatteryCurrent, BatCurDir, MotorCurPeakAvg, FetTemp, MotorRotSpeed, PWM;
uint16_t PowerMode, MotorControlMode, AcceleratorPosition, RegenerationVrPosition, DigitSwPosition, OutputTargetValue, DriveActionStatus, RegenerationStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		debug("State: M2M_WIFI_RESP_CON_STATE_CHANGED\r\n");
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			debug("Wi-Fi Status: Connected\r\n");
			m2m_wifi_enable_dhcp(0);
#if (defined USE_STATIONMODE)
			tstrM2MIPConfig stnConfig;
			stnConfig.u32StaticIP = _htonl(MAIN_WIFI_M2M_CLIENT_IP_ALT);
			stnConfig.u32SubnetMask = _htonl(0xFFFFFF00); //corresponds to 255.255.255.0
			m2m_wifi_set_static_ip(&stnConfig);
#endif
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
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
		debug("State: M2M_WIFI_REQ_DHCP_CONF\r\n");
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
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
		//debug("\033[H");
		//debug("\033[6B");
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
		debug("State: SOCKET_MSG_BIND\r\n");

		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg*)pvMsg;
		if(pstrBind->status == 0)
		{
			debug("Bind Success\r\n");

			// call Recv
			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
		}
		else
		{
			debug("Bind Failed\r\n");
		}
	}
	else if(u8Msg == SOCKET_MSG_RECVFROM)
	{
		debug("State: SOCKET_MSG_RECVFROM\r\n");

		tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
		{
			char remote_addr[16];
			inet_ntop4((unsigned char*)&pstrRecvMsg->strRemoteAddr.sin_addr, (char*)&remote_addr, 16);

			// Handle received data.
			debug("Message received from %s: %s\r\n", remote_addr, rxBuffer);

			HAL_UART_Transmit(&huart2, rxBuffer, 17, 100);
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
			debug("WINC1500 Wifi Bind Success\r\n");
		}
		else
		{
			debug("WINC1500 Wifi Bind Failed\r\n");
		}
	}
}

void spi_dummy_send(void)
{
	HAL_SPI_Init(&hspi_winc);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi_winc, 0xFF, 1, 100);
	HAL_GPIO_WritePin(WINC_CS_GPIO_Port, WINC_CS_Pin, GPIO_PIN_SET);
}

void debug(char *fmt, ...)
{
	if (DEBUG)
	{
		char buffer[500];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, args);
		va_end(args);
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_9)
	{
		isr();
	}
}

void Can_Extract_Data(uint32_t frameID)
{
	UART_Send_Cmd(&huart2, ASCII_HOME);
	if (frameID == MOTOR_CAN_RESP_FRAME_0_ID)
	{
		// Extract the bits
		BatteryVoltage = ((RxData[1U] & 0x03U) << 8U) | RxData[0U];
		BatteryCurrent = ((RxData[2U] & 0x07U) << 6U) | ((RxData[1U] & 0xFCU) >> 2U);
		BatCurDir = (RxData[2U] & 0x08U) >> 3U;
		MotorCurPeakAvg = ((RxData[3U] & 0x3FU) << 4U) | ((RxData[2U] & 0xF0) >> 4U);
		FetTemp = ((RxData[4U] & 0x07U) << 2U) | ((RxData[3U] & 0xC0) >> 6U);
		MotorRotSpeed = ((RxData[5U] & 0x7FU) << 5U) | ((RxData[4U] & 0xF8) >> 3U);
		PWM = ((RxData[7U] & 0x01U) << 9U) | RxData[6U] << 1U | ((RxData[5U] & 0x80U) >> 7U);

		// Convert to unit if necessary
		BatteryVoltage /= 2;
		FetTemp *= 5;
		PWM /= 2;

		// Print values
		UART_Send_Cmd(&huart2, "\033[10B");

		debug("Battery Voltage\t\t\t: %04d\r\n", BatteryVoltage);
		debug("Battery Current\t\t\t: %04d\r\n", BatteryCurrent);
		debug("Current Direction\t\t: %04d\r\n", BatCurDir);
		debug("Motor Current Peak Avg\t\t: %04d\r\n", MotorCurPeakAvg);
		debug("FET Temp\t\t\t: %04d\r\n", FetTemp);
		debug("Motor Rotation Speed\t\t: %04d\r\n", MotorRotSpeed);
		debug("PWM\t\t\t\t: %04d\r\n", PWM);
	}
	else if (frameID == MOTOR_CAN_RESP_FRAME_1_ID)
	{
		// Extract the bits
		PowerMode = RxData[0] & 0x01U;
		MotorControlMode = (RxData[0] & 0x02U) >> 1U;
		AcceleratorPosition = ((RxData[1] & 0x0FU) << 6U) | ((RxData[0] & 0xFCU) >> 2U);
		RegenerationVrPosition = ((RxData[2] & 0x3FU) << 4U) | ((RxData[1] & 0xF0U) >> 4U);
		DigitSwPosition = ((RxData[3] & 0x03U) << 2U) | ((RxData[2] & 0xC0U) >> 6U);
		OutputTargetValue = ((RxData[4] & 0x0FU) << 6U) | ((RxData[3] & 0xFCU) >> 2U);
		DriveActionStatus = (RxData[4] & 0x30U) >> 4U;
		RegenerationStatus = (RxData[4] & 0x40U) >> 5U;

		// Convert to unit if necessary
		AcceleratorPosition /= 2;
		RegenerationVrPosition /= 2;
		OutputTargetValue /= 2;

		// Print values
		UART_Send_Cmd(&huart2, "\033[20B");
		debug("Power Mode\t\t\t\t: %04d\r\n", PowerMode);
		debug("Motor Control Mode\t\t\t: %04d\r\n", MotorControlMode);
		debug("Accelerator Position\t\t\t: %04d\r\n", AcceleratorPosition);
		debug("Regeneration VR Position\t\t: %04d\r\n", RegenerationVrPosition);
		debug("Digit SW Position\t\t\t: %04d\r\n", DigitSwPosition);
		debug("Output Target Value\t\t\t: %04d\r\n", OutputTargetValue);
		debug("Drive Action Status\t\t\t: %04d\r\n", DriveActionStatus);
		debug("Regeneration Status\t\t\t: %04d\r\n", RegenerationStatus);
	}
	else if (frameID == MOTOR_CAN_RESP_FRAME_2_ID)
	{
		uint8_t OverHeatLevel = RxData[4] & 0x03U;
		uint32_t ErrorBits = RxData[3] | RxData[2] | RxData[1] | RxData[0];

		UART_Send_Cmd(&huart2, "\033[30B");
		debug("OverHeat Level: %d\r\n", OverHeatLevel);

		if (ErrorBits > 0)
		{
			debug("ERROR DETECTED\r\n");

			if (ErrorBits & (2^0))
			{
				debug("Analog Sensor Error\r\n");
			}
			if (ErrorBits & (2^1))
			{
				debug("Motor Current Sensor U Error\r\n");
			}
			if (ErrorBits & (2^2))
			{
				debug("Motor Current Sensor W Error\r\n");
			}
			if (ErrorBits & (2^3))
			{
				debug("FET Thermistor Error\r\n");
			}
			if (ErrorBits & (2^5))
			{
				debug("Battery Voltage Sensor Error\r\n");
			}
			if (ErrorBits & (2^6))
			{
				debug("Battery Current Sensor Error\r\n");
			}
			if (ErrorBits & (2^7))
			{
				debug("Battery Current Sensor Adjust Error\r\n");
			}
			if (ErrorBits & (2^8))
			{
				debug("Motor Current Sensor Adjust Error\r\n");
			}
			if (ErrorBits & (2^9))
			{
				debug("Accelerator Position Error\r\n");
			}
			if (ErrorBits & (2^11))
			{
				debug("Controller Voltage Sensor Error\r\n");
			}
			if (ErrorBits & (2^16))
			{
				debug("Power System Error\r\n");
			}
			if (ErrorBits & (2^19))
			{
				debug("Over Current Error\r\n");
			}
			if (ErrorBits & (2^21))
			{
				debug("Over Current Limit\r\n");
			}
			if (ErrorBits & (2^24))
			{
				debug("Motor System Error\r\n");
			}
			if (ErrorBits & (2^25))
			{
				debug("Motor Lock\r\n");
			}
			if (ErrorBits & (2^26))
			{
				debug("Hall Sensor Short\r\n");
			}
			if (ErrorBits & (2^27))
			{
				debug("Hall Sensor Open\r\n");
			}
		}
		else
		{
			debug("NO ERRORS\r\n");
		}
	}

	// Clear txBuffer
	memset(txBuffer,0,sizeof(txBuffer));

	txBuffer[16] = 0x00;
	txBuffer[15] = 0x12;
	txBuffer[14] = 0x30;
	txBuffer[13] = 0x11;
	txBuffer[12] = RxHeader.ExtId & 0xFF000000 >> 24;
	txBuffer[11] = RxHeader.ExtId & 0x00FF0000 >> 16;
	txBuffer[10] = RxHeader.ExtId & 0x0000FF00 >> 8;
	txBuffer[9] = RxHeader.ExtId & 0x000000FF;
	txBuffer[8] = RxData[7];
	txBuffer[7] = RxData[6];
	txBuffer[6] = RxData[5];
	txBuffer[5] = RxData[4];
	txBuffer[4] = RxData[3];
	txBuffer[3] = RxData[2];
	txBuffer[2] = RxData[1];
	txBuffer[1] = RxData[0];
	txBuffer[0] = 0x100 - ((txBuffer[16] + txBuffer[1] + txBuffer[2] + txBuffer[3] +
			txBuffer[4] + txBuffer[5] + txBuffer[6] + txBuffer[7] + txBuffer[8] +
			txBuffer[9] + txBuffer[10] + txBuffer[11] + txBuffer[12] + txBuffer[13] +
			txBuffer[14] + txBuffer[15]) % 0x100);

	// Fill txBuffer
	snprintf(txBuffer, 34, "0x%08x%08x%02x%02x%02x%02x%02x%02x%02x%02x", time_stamp, RxHeader.ExtId, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);

	debug("CAN Message Received: %s\r\n", txBuffer);

	UART_Send_Cmd(&huart2, ASCII_HOME);

	int len = strlen(txBuffer);

	// Send to wifi
	sendto(udp_socket, (void *)txBuffer, len, 0, (struct sockaddr*)&strAddr,
			sizeof(strAddr));
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	MX_CAN1_Init();
	MX_USART3_UART_Init();
	MX_TIM15_Init();
	/* USER CODE BEGIN 2 */

#if CAN_ON
	// CAN filter configuration
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	// Fill CAN TxHeader and TxData
	TxHeader.StdId = 0x321;
	TxHeader.ExtId = MOTOR_CAN_REQ_EXT_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 1;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxData[0] = MOTOR_CAN_REQ_FRAME_0 | MOTOR_CAN_REQ_FRAME_1 | MOTOR_CAN_REQ_FRAME_2;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;
#endif

	debug("%s", ASCII_CLEAR);
	debug("%s", ASCII_HOME);
	debug("Device start\r");

#if WIFI_ON
	// Init SPI with dummy byte
	spi_dummy_send();

	// Init WINC1500
	/*if (wincInit() != M2M_SUCCESS)
	{
		result = M2M_ERR_INIT;
	}*/

	tstrWifiInitParam param;
	int8_t ret;

	// Initialize the WiFi BSP:
	nm_bsp_init();

	// Initialize WiFi parameters structure
	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;

	// Initialize WiFi module and register status callback:
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret && M2M_ERR_FW_VER_MISMATCH != ret) {
		debug("WINC1500 Wifi Initialization failed\r\n");
	}
	else
	{
		debug("WINC1500 Wifi Initialized\r\n");
	}

	// Initialize the socket layer.
	socketInit();

	// Configure broadcast struct.
	broadcastAddr.sin_family = AF_INET;
	broadcastAddr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	broadcastAddr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_BROADCAST_IP);


#if (defined USE_STATIONMODE)
	// Start Connect
	ret = m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH,
			(char *)MAIN_WLAN_PSK, 1);
	//ret = m2m_wifi_default_connect();
#elif (defined USE_APMODE)
	tstrM2MAPConfig apConfig;
	strcpy(apConfig.au8SSID, "WINC1500_AP"); // Set SSID
	apConfig.u8SsidHide = SSID_MODE_VISIBLE; // Set SSID to be broadcasted
	apConfig.u8ListenChannel = 1; // Set Channel
	apConfig.u8SecType = MAIN_WLAN_AUTH; // Set Security to OPEN

	// IP Address
	apConfig.au8DHCPServerIP[0] = 192;
	apConfig.au8DHCPServerIP[1] = 168;
	apConfig.au8DHCPServerIP[2] = 1;
	apConfig.au8DHCPServerIP[3] = 1;

	tstrM2MIPConfig ipConfig;
	ipConfig.u32StaticIP = MAIN_WIFI_M2M_SERVER_IP;

	m2m_wifi_set_static_ip(&ipConfig);

	// Start AP mode
	ret = m2m_wifi_enable_ap(&apConfig);
#endif

	if (ret != M2M_SUCCESS)
	{
		debug("WINC1500 Wifi failed to connect\r\n");
		result = M2M_ERR_JOIN_FAIL;
	}
	else
	{
		debug("WINC1500 Wifi Connected\r\n");
	}

	// Wait for connection
	nm_bsp_sleep(10000);
#endif

	//HAL_UART_Receive_IT(&huart3, uart3RxBuffer, UART_BUFFER_SIZE);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
#if WIFI_ON
		m2m_wifi_handle_events(NULL);
#endif
#ifdef USE_CLIENT

		// Clear txBuffer
		/*memset(txBuffer,0,sizeof(txBuffer));

		// Fill in the txBuffer with some data
		strcpy((char *)txBuffer, "Alive");

		sendto(udp_socket, (void *)txBuffer, strlen(txBuffer), 0, (struct sockaddr*)&strAddr,
				sizeof(strAddr));*/

		//m2m_wifi_req_curr_rssi();

		HAL_Delay(50);

		// Clear txBuffer
		memset(txBuffer,0,sizeof(txBuffer));

		// Fill in the txBuffer with some data
		//sprintf(txBuffer, "Signal: %d", global_rssi);
		//sprintf(txBuffer, "0x00123011088502457e82040f9120003e");
		txBuffer[0] = 0x00;
		txBuffer[1] = 0x12;
		txBuffer[2] = 0x30;
		txBuffer[3] = 0x11;
		txBuffer[4] = 0x08;
		txBuffer[5] = 0x85;
		txBuffer[6] = 0x02;
		txBuffer[7] = 0x45;
		txBuffer[8] = 0x7e;
		txBuffer[9] = 0x82;
		txBuffer[10] = 0x04;
		txBuffer[11] = 0x0f;
		txBuffer[12] = 0x91;
		txBuffer[13] = 0x20;
		txBuffer[14] = 0x00;
		txBuffer[15] = 0x3e;
		txBuffer[16] = 0x100 - ((txBuffer[0] + txBuffer[1] + txBuffer[2] + txBuffer[3] +
				txBuffer[4] + txBuffer[5] + txBuffer[6] + txBuffer[7] + txBuffer[8] +
				txBuffer[9] + txBuffer[10] + txBuffer[11] + txBuffer[12] + txBuffer[13] +
				txBuffer[14] + txBuffer[15]) % 0x100);


		sendto(udp_socket, (void *)txBuffer, 17, 0, (struct sockaddr*)&strAddr,
				sizeof(strAddr));

		debug("UDP Sent\r\n");

		//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		//nm_bsp_sleep(1000);
#endif
		if (uart3RxITFlag == 1)
		{
			uart3RxITFlag = 0;
			debug("%s", uart3RxBuffer);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}

	return 0;
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 40;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 0;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 19999;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPS_DIR_GPIO_Port, GPS_DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPS_RSTN_GPIO_Port, GPS_RSTN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, WINC_CS_Pin|WINC_EN_Pin|WINC_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPS_DIR_Pin */
	GPIO_InitStruct.Pin = GPS_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_DIR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPS_RSTN_Pin */
	GPIO_InitStruct.Pin = GPS_RSTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_RSTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : WINC_CS_Pin WINC_EN_Pin WINC_RST_Pin */
	GPIO_InitStruct.Pin = WINC_CS_Pin|WINC_EN_Pin|WINC_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : WINC_IRQ_Pin */
	GPIO_InitStruct.Pin = WINC_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(WINC_IRQ_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	//debug("CAN Request Sent\r\n");
	time_stamp = HAL_CAN_GetTxTimestamp(hcan, TxMailbox);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	Can_Extract_Data(RxHeader.ExtId);
	//debug("CAN Message Received: 0x%x%x%x%x%x%x%x%x \r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		uart3RxITFlag = 1;
		HAL_UART_Receive_IT(&huart3, uart3RxBuffer, UART_BUFFER_SIZE);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
