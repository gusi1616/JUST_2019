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

/** Set WIFI Mode **/
//#define USE_APMODE
#define USE_STATIONMODE

/** Set UDP Mode **/
#define USE_CLIENT
//#define USE_SERVER

/** Set Debug Mode **/
#define DEBUG 1

#define false 0
#define true 1

/** Max length of SSID and password */
#define MAX_LEN            0x11

/** Port to use for TCP socket */
#define TCP_PORT                       (1234)
/** Port to use for UDP socket */
#define UDP_PORT                       (6666)

/** Max size of incoming packets */
#define MAIN_WIFI_M2M_BUFFER_SIZE		20

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_WIFI_M2M_BROADCAST_IP 0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_CLIENT_IP  0xc0a80164  // 192.168.1.100
#define MAIN_WIFI_M2M_CLIENT_IP_ALT 0xc0a80184 // 192.168.1.132
#define MAIN_WIFI_M2M_SERVER_IP  0xC0A80101  // 192.168.1.1
#define MAIN_WIFI_M2M_SERVER_PORT (6666)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/** Sockets for TCP and UDP communication */
static SOCKET tcp_server_socket = -1;
static SOCKET tcp_client_socket = -1;
static SOCKET udp_socket = -1;

struct sockaddr_in strAddr;
struct sockaddr_in broadcastAddr;

/** Wi-Fi connection state */
static volatile uint8_t wifi_connected;

/** Receive buffer definition. */
uint8_t rxBuffer[256], txBuffer[256];

int result = 0;

int arefCount = 0;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
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
			debug("Wi-Fi connected\r\n");
			m2m_wifi_enable_dhcp(0);
#if (defined USE_STATIONMODE)
			tstrM2MIPConfig stnConfig;
			stnConfig.u32StaticIP = _htonl(MAIN_WIFI_M2M_CLIENT_IP_ALT);
			stnConfig.u32SubnetMask = _htonl(0xFFFFFF00); //corresponds to 255.255.255.0
			m2m_wifi_set_static_ip(&stnConfig);
#endif
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			debug("Wi-Fi disconnected\r\n");
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
		debug("Client IP is %d.%d.%d.%d\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		// Initialize socket
#if (defined USE_CLIENT)
		udpClientStart((char*)MAIN_WIFI_M2M_SERVER_IP, MAIN_WIFI_M2M_SERVER_PORT);
#elif (defined USE_SERVER)
		udpStartServer(MAIN_WIFI_M2M_SERVER_PORT);
#endif
		break;
	}

	case M2M_WIFI_RESP_IP_CONFIGURED:
	{
		debug("State: M2M_WIFI_RESP_IP_CONFIGURED\r\n");
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		debug("Wi-Fi RESP IP configured\r\n");
		debug("Wi-Fi Static IP is %d.%d.%d.%d\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		break;
	}

	default:
	{
		break;
	}
	}
}

void tcpClientSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	if(sock == tcp_client_socket)
	{
		if(u8Msg == SOCKET_MSG_CONNECT)
		{
			debug("State: SOCKET_MSG_CONNECT\r\n");

			// Connect Event Handler
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg*)pvMsg;

			if(pstrConnect->s8Error == 0)
			{
				// Perform data exchange
				uint16_t u16MsgSize;

				// Fill in the txBuffer with some data here
				txBuffer[0] = 8;
				txBuffer[1] = 0;
				txBuffer[2] = 0;
				txBuffer[3] = 8;
				txBuffer[4] = 5;
				// Send data
				send(tcp_client_socket, txBuffer, u16MsgSize, 0);
				// Recv response from server
				recv(tcp_client_socket, rxBuffer, sizeof(rxBuffer) , 0);
			}
			else
			{
				debug("TCP Connection Failed\n");
			}
		}
		else if(u8Msg == SOCKET_MSG_RECV)
		{
			debug("State: SOCKET_MSG_RECV\r\n");

			tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;

			// If not empty
			if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
			{
				// Process the received message
				for(int i=0; i < pstrRecvMsg->s16BufferSize; i++)
				{
					UART_Send_Int(&huart2, pstrRecvMsg->pu8Buffer[i]);
				}

				debug("\nEnd of message\n");

				// Close the socket
				close(tcp_client_socket);
			}
		}
	}
}

void dnsResolveCallback(uint8_t* pu8HostName, uint32_t u32ServerIP)
{
	struct sockaddr_in strAddr;

	if(u32ServerIP != 0)
	{
		tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0);
		if(tcp_client_socket >= 0)
		{
			strAddr.sin_family		= AF_INET;
			strAddr.sin_port		= _htons(MAIN_WIFI_M2M_SERVER_PORT);
			strAddr.sin_addr.s_addr	= _htonl(MAIN_WIFI_M2M_SERVER_IP);

			int ret = connect(tcp_client_socket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
			if (ret != M2M_SUCCESS)
			{
				debug("Connected to TCP server\r\n");
			}
			else
			{
				debug("Connection failed to TCP server\r\n");
			}
		}
	}
	else
	{
		debug("DNs Resolution Failed\n");
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

			sendto(udp_socket, txBuffer, len, 0,
					(struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));

			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);


			// Close the socket after finished
			//close(udp_socket);
		}
	}
	else if (u8Msg == SOCKET_MSG_SENDTO)
	{
		debug("State: SOCKET_MSG_SENDTO\r\n");

		debug("Sent HejAref nr: %d\r\n", arefCount);
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
		strcpy(txBuffer, "Hello Aref");
		len = strlen(txBuffer);

		HAL_Delay(500);

		sendto(udp_socket, (void *)txBuffer, len, 0, (struct sockaddr*)&strAddr,
				sizeof(strAddr));

		/*sendto(udp_socket, txBuffer, strlen(txBuffer), 0, (struct sockaddr*)&broadcastAddr,
									sizeof(struct sockaddr_in));*/

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

void udpServerSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
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

			uint16 len;

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
  /* USER CODE BEGIN 2 */

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
	TxHeader.ExtId = 0x08f91540;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 1;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxData[0] = 1;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

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
	//ret = m2m_wifi_init(&param);
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
	//ret = m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH,
	//		(char *)MAIN_WLAN_PSK, 1);
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
	//nm_bsp_sleep(10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//m2m_wifi_handle_events(NULL);
#ifdef USE_CLIENT
		if (arefCount == 10)
		{
			close(udp_socket);
			udp_socket = -1;
			debug("UDP Test Complete\r\n");
			break;
		}

		HAL_Delay(1000);

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		// Clear txBuffer
		memset(txBuffer,0,sizeof(txBuffer));

		// Fill txBuffer
		strcpy((char *)txBuffer, "HejAref");

		// Send txBuffer
		//ret = sendto(udp_socket, (void *)txBuffer, strlen((char*)txBuffer), 0, (struct sockaddr*)&strAddr,
		//		sizeof(struct sockaddr_in));
		/*if (ret == M2M_SUCCESS)
		{
			arefCount++;
			debug("Message sent\r\n");
		}
		else
		{
			debug("Failed to send message\r\n");
		}*/
#endif

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
	debug("CAN Request Sent\r\n");
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	debug("CAN Message Received: 0x%x%x%x%x%x%x%x%x \r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
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
