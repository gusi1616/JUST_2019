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
#include "winc1500.h"
#include <string.h>
#include "driver/m2m_wifi.h"
#include "driver/nmasic.h"
#include "driver/m2m_hif.h"
#include "socket/socket.h"
#include "driver/m2m_periph.h"
#include "bsp/nm_bsp_stm32.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 chip information example --"STRING_EOL \
		"-- "BOARD_NAME " --"STRING_EOL	\
		"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

/** Security mode Supported */
//#define USE_WPAPSK 1 /* WPA/WPA2 PSK Security Mode*/
// #define USE_WEP 2 /* WEP Security Mode*/
#define USE_OPEN 3 /* No Security or OPEN Authentication Mode*/
/** AP mode Settings */
#define MAIN_WLAN_SSID "SimonsWIFI" /* < SSID */
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


#define false 0
#define true 1

/** Max length of SSID and password */
#define MAX_LEN            0x11

/** Port to use for TCP socket */
#define TCP_PORT                       (1234)
/** Port to use for UDP socket */
#define UDP_PORT                       (6789)

/** Max size of incoming packets */
#define MAIN_WIFI_M2M_BUFFER_SIZE		20

/** Buffer to send to identify Winc + Mega to Android app */
#define KEY_BUFFER                     {0x6d, 0x61, 0x72, 0x74, 0x65}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_WIFI_M2M_SERVER_IP 0xc0a80164 //0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_SERVER_PORT (6666)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t scan_request_index = 0;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;

/** Sockets for TCP and UDP communication */
static SOCKET tcp_server_socket = -1;
static SOCKET tcp_client_socket = -1;
static SOCKET udp_socket = -1;

struct sockaddr_in strAddr;

/** Wi-Fi connection state */
static volatile uint8_t wifi_connected;

/** Variable for main loop to see if a toggle request is received */
volatile uint8_t toggle = false;
/** Variable for main loop to see if a temperature read request is received */
volatile uint8_t temperature = false;

/** Receive buffer definition. */
static char gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];
uint8_t rxBuffer[256], txBuffer[256];

/** Buffer for wifi password */
uint8_t password[MAX_LEN];
/** Buffer for wifi SSID*/
uint8_t ssid[MAX_LEN];
/** Buffer for wifi security */
uint8_t security[2];

/** Variable to store status of provisioning mode  */
uint8_t provision = 1;

extern time_flags_typedef timeFlags;

int result = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
	{
		UART_Send_Str(&huart2, "State: M2M_WIFI_RESP_SCAN_DONE\n\r");

		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
		scan_request_index = 0;
		if (pstrInfo->u8NumofCh >= 1) {
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} else {
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		UART_Send_Str(&huart2, "State: M2M_WIFI_RESP_SCAN_RESULT\n\r");

		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
		uint16_t demo_ssid_len;
		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);

		/* display founded AP. */
		printf("[%d] SSID:%s\r\n", scan_request_index, pstrScanResult->au8SSID);

		num_founded_ap = m2m_wifi_get_num_ap_found();
		if (scan_ssid_len) {
			/* check same SSID. */
			demo_ssid_len = strlen((const char *)MAIN_WLAN_SSID);
			if
			(
					(demo_ssid_len == scan_ssid_len) &&
					(!memcmp(pstrScanResult->au8SSID, (uint8_t *)MAIN_WLAN_SSID, demo_ssid_len))
			) {
				/* A scan result matches an entry in the preferred AP List.
				 * Initiate a connection request.
				 */
				printf("Found %s \r\n", MAIN_WLAN_SSID);
				m2m_wifi_connect((char *)MAIN_WLAN_SSID,
						sizeof(MAIN_WLAN_SSID),
						MAIN_WLAN_AUTH,
						(void *)MAIN_WLAN_PSK,
						M2M_WIFI_CH_ALL);
				break;
			}
		}

		if (scan_request_index < num_founded_ap) {
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} else {
			printf("can not find AP %s\r\n", MAIN_WLAN_SSID);
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		UART_Send_Str(&huart2, "State: M2M_WIFI_RESP_CON_STATE_CHANGED\n\r");

		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			UART_Send_Str(&huart2, "Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			UART_Send_Str(&huart2, "Wi-Fi disconnected\r\n");

			/* Request scan. */
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		UART_Send_Str(&huart2, "State: M2M_WIFI_REQ_DHCP_CONF\n\r");
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		UART_Send_Str(&huart2, "Wi-Fi connected\r\n");
		UART_Send_Str(&huart2, "Wi-Fi IP is ");
		UART_Send_IntAndStr(&huart2, pu8IPAddress[0], ".");
		UART_Send_IntAndStr(&huart2, pu8IPAddress[1], ".");
		UART_Send_IntAndStr(&huart2, pu8IPAddress[2], ".");
		UART_Send_IntAndStr(&huart2, pu8IPAddress[3], "\r\n");
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
			UART_Send_Str(&huart2, "State: SOCKET_MSG_CONNECT\n\r");

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
				UART_Send_Str(&huart2, "TCP Connection Failed\n");
			}
		}
		else if(u8Msg == SOCKET_MSG_RECV)
		{
			UART_Send_Str(&huart2, "State: SOCKET_MSG_RECV\n\r");

			tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;

			// If not empty
			if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
			{
				// Process the received message
				for(int i=0; i < pstrRecvMsg->s16BufferSize; i++)
				{
					UART_Send_Int(&huart2, pstrRecvMsg->pu8Buffer[i]);
				}

				UART_Send_Str(&huart2, "\nEnd of message\n");

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
				UART_Send_Str(&huart2, "Connected to TCP server\n\r");
			}
			else
			{
				UART_Send_Str(&huart2, "Connection failed to TCP server\n\r");
			}
		}
	}
	else
	{
		UART_Send_Str(&huart2, "DNs Resolution Failed\n");
	}
}

void udpClientSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	if((u8Msg == SOCKET_MSG_RECV) || (u8Msg == SOCKET_MSG_RECVFROM))
	{
		tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
		{
			uint16 len;
			// Format a message in the txBuffer and put its length in len
			txBuffer[0] = 0;
			txBuffer[1] = 1;
			txBuffer[2] = 2;
			len = 3;

			sendto(udp_socket, txBuffer, len, 0,
					(struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));

			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);

			// Close the socket after finished
			close(udp_socket);
		}
	}
}

void udpClientStart(char *pcServerIP, uint16_t port)
{
	// Initialize the socket layer.
	socketInit();

	// Register socket application callbacks.
	registerSocketCallback(udpClientSocketEventHandler, NULL);

	udp_socket = socket(AF_INET, SOCK_STREAM, 0);
	if(udp_socket >= 0)
	{
		uint16 len;
		strAddr.sin_family = AF_INET;
		strAddr.sin_port = _htons(port);
		strAddr.sin_addr.s_addr = nmi_inet_addr(pcServerIP);

		// Format some message in the txBuffer and put its length in len
		txBuffer[0] = 0;
		txBuffer[1] = 1;
		txBuffer[2] = 2;
		len = 3;

		sendto(udp_socket, txBuffer, len, 0, (struct sockaddr*)&strAddr,
				sizeof(struct sockaddr_in));

		recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);

		UART_Send_Str(&huart2, "Message received: ");
		UART_Send_Str(&huart2, rxBuffer);
		UART_Send_Str(&huart2, "\n\r");
	}
}

void udpServerSocketEventHandler(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	if(u8Msg == SOCKET_MSG_BIND)
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg*)pvMsg;
		if(pstrBind->status == 0)
		{
			UART_Send_Str(&huart2, "Bind Success\n\r");
			// call Recv
			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);
			UART_Send_Str(&huart2, "Message received: ");
			UART_Send_Str(&huart2, rxBuffer);
			UART_Send_Str(&huart2, "\n\r");
		}
		else
		{
			UART_Send_Str(&huart2, "Bind Failed\n\r");
		}
	}
	else if(u8Msg == SOCKET_MSG_RECV)
	{
		tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
		{
			// Perform data exchange.
			uint16 len;

			// Fill in the acSendBuffer with some data
			txBuffer[0] = 0;
			txBuffer[1] = 1;
			txBuffer[2] = 2;
			len = 3;

			// Send some data to the same address.
			/*sendto(udp_socket, txBuffer, len, 0,
					&pstrRecvMsg->strRemoteAddr, sizeof(pstrRecvMsg->strRemoteAddr));*/


			// call Recv
			recvfrom(udp_socket, rxBuffer, sizeof(rxBuffer), 0);

			UART_Send_Str(&huart2, "Message received: ");
			UART_Send_Str(&huart2, rxBuffer);
			UART_Send_Str(&huart2, "\n\r");

			// Close the socket when finished.
			close(udp_socket);
		}
	}
}

void udpStartServer(uint16 u16ServerPort)
{
	// Initialize the socket layer.
	socketInit();

	// Register socket application callbacks.
	registerSocketCallback(udpServerSocketEventHandler, NULL);

	// Create the server listen socket.
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if(udp_socket >= 0)
	{
		strAddr.sin_family = AF_INET;
		strAddr.sin_port = _htons(u16ServerPort);
		strAddr.sin_addr.s_addr = MAIN_WIFI_M2M_SERVER_IP; //INADDR_ANY

		int ret = bind(udp_socket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));

		if (ret == M2M_SUCCESS)
		{
			UART_Send_Str(&huart2, "WINC1500 Wifi Bind Success\n\r");
		}
		else
		{
			UART_Send_Str(&huart2, "WINC1500 Wifi Bind Failed\n\r");
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
  /* USER CODE BEGIN 2 */

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
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		UART_Send_Str(&huart2, "WINC1500 Wifi Initialization failed\n\r");
	}
	else
	{
		UART_Send_Str(&huart2, "WINC1500 Wifi Initialized\n\r");
	}

	// Initialize socket
	udpStartServer(MAIN_WIFI_M2M_SERVER_PORT);

	// Start Connect
	/*ret = m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH,
			(char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);*/
	//ret = wincStartConnect("SimonsWifix", M2M_WIFI_SEC_OPEN, NULL, 6);

	tstrM2MAPConfig apConfig;
	strcpy(apConfig.au8SSID, "WINC1500_AP"); // Set SSID
	apConfig.u8SsidHide = SSID_MODE_VISIBLE; // Set SSID to be broadcasted
	apConfig.u8ListenChannel = 1; // Set Channel
	apConfig.u8SecType = MAIN_WLAN_AUTH; // Set Security to OPEN

	// IP Address
	apConfig.au8DHCPServerIP[0] = 192;
	apConfig.au8DHCPServerIP[1] = 168;
	apConfig.au8DHCPServerIP[2] = 1;
	apConfig.au8DHCPServerIP[3] = 100;

	// Start AP mode
	ret = m2m_wifi_enable_ap(&apConfig);
	if (ret != M2M_SUCCESS)
	{
		UART_Send_Str(&huart2, "WINC1500 Wifi failed to connect\n\r");
		result = M2M_ERR_JOIN_FAIL;
	}
	else
	{
		UART_Send_Str(&huart2, "WINC1500 Wifi Connected\n\r");
	}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		m2m_wifi_handle_events(NULL);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
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
