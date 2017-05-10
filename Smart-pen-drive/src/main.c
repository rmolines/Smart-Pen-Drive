#include "asf.h"
#include <string.h>
#include "main.h"
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

/************************************************************************/
/*  Defines                                                             */
/************************************************************************/

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP server example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    8
#define LED_PIN_MASK  (1<<LED_PIN)

/************************************************************************/
/*  Global vars                                                         */
/************************************************************************/

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for TCP communication */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
	
uint32 g_rxCnt = 0 ;

/************************************************************************/
/*  SOCKET MSGs                                                         */
/************************************************************************/

/** Comandos recebidos via socket */
const uint8_t MSG_SOCKET_LED_ON[]       = "Turn Led ON ! \n";
const uint8_t MSG_SOCKET_LED_ON_ACK[]   = "Led Powered ON ! \n";
const uint8_t MSG_SOCKET_LED_OFF[]      = "Turn Led OFF ! \n";
const uint8_t MSG_SOCKET_LED_OFF_ACK[]  = "Led Powered OFF \n";
const uint8_t MSG_SOCKET_LED_STATUS[]   = "Led Status ? \n";
const uint8_t MSG_SOCKET_ERRO[]         = "Command not defined \n";
enum MSG_SOCKET_COMMANDS {COMMAND_LED_ON, COMMAND_LED_OFF, COMMAND_LED_STATUS, COMMAND_ERRO};

/************************************************************************/
/*  Funcoes                                                             */
/************************************************************************/

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * Faz a interpretação da mensagem 
 * enviada via socket txp/ip
 * retorna um numero equivalente ao
 * comando a ser executado
 */
uint8_t message_parsing(uint8_t *message){ 

  if(!strcmp(message, MSG_SOCKET_LED_ON)){
    printf(MSG_SOCKET_LED_ON_ACK);
    return(COMMAND_LED_ON);
  }
  else if(!strcmp(message, MSG_SOCKET_LED_OFF)){
    printf(MSG_SOCKET_LED_OFF);
    return(COMMAND_LED_OFF);
  }
  else if(!strcmp(message, MSG_SOCKET_LED_STATUS)){
     printf(MSG_SOCKET_LED_STATUS);
     return(COMMAND_LED_STATUS);
  }
  else{
    printf(MSG_SOCKET_ERRO);
    return(COMMAND_ERRO);
  }      
}

/************************************************************************/
/*  CallBacks                                                           */
/************************************************************************/

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
   
  /* Socket connected */
  case SOCKET_MSG_CONNECT:
  {
    uint16_t rtn;
    memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
    sprintf((char *)gau8ReceivedBuffer, "%s%s", HOST_MSG, HOST_MSG_SUFFIX);
    
    tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
    if (pstrConnect && pstrConnect->s8Error >= 0) {
      printf("socket_cb: connect success!\r\n");
      rtn = send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
      memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
      recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);  
      } else {
      printf("socket_cb: connect error!\r\n");
      close(tcp_client_socket);
      tcp_client_socket = -1;
    }
  }
  break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{
		printf("socket_cb: send success!\r\n");
	  //printf("TCP Server Test Complete!\r\n");
		//printf("close socket\n");
		//close(tcp_client_socket);
		//close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
    
    uint8_t  messageAck[64];
    uint16_t messageAckSize;
    uint8_t  command;
        
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {     
			printf(" ---------- \n Cnt : %d \n -----------\n", ++g_rxCnt);
			   
      // Para debug das mensagens do socket
			printf("%s \r\n", pstrRecv->pu8Buffer);   
       
      // limpa o buffer de recepcao e tx
      memset(pstrRecv->pu8Buffer, 0, pstrRecv->s16BufferSize); 
      //memset(pstrRecv->pu8Buffer, 0, pstrRecv->s16BufferSize); 
      
      // envia a resposta
      //int8_t  messageAck[]="GET /led=status";
      //send(tcp_client_socket, messageAck, sizeof(messageAck), 0);
      
      // Requista novos dados
      //recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
      
 		} else {
			printf("socket_cb: recv error!\r\n");
			close(tcp_client_socket);
			tcp_client_socket = -1;
		}    
	}

	break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = 1;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
	}
}

/************************************************************************/
/*  Main                                                               */
/************************************************************************/

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then test function of TCP server.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;
	
	

	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_SERVER_PORT);
	addr.sin_addr.s_addr = MAIN_SERVER_IP;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
  
	
	/** SDCARD */
	irq_initialize_vectors();
	cpu_irq_enable();
	
	char test_file_name[] = "0:sd_mmc_test.txt";
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	
	/* Initialize SD MMC stack */
	sd_mmc_init();
	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
	
	printf("Please plug an SD, MMC or SDIO card in slot.\n\r");
	/* Wait card present and ready */
	do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("Card install FAIL\n\r");
				printf("Please unplug and re-plug the card.\n\r");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
	} while (CTRL_GOOD != status);	
	
	printf("Mount disk (f_mount)...\r\n");

	
	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);

		if (wifi_connected == M2M_WIFI_CONNECTED) {
				if (tcp_client_socket < 0) {
  				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    				printf("main: failed to create TCP client socket error!\r\n");
    				continue;
  				}
          
          /* Connect TCP client socket. */
          if (connect(tcp_client_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR ) {
            printf("main: failed to connect socket error!\r\n");
            close(tcp_client_socket);
            continue;
          }else{
            printf("Conectado ! \n");
          }             
        }				
		}
	}

	return 0;
}

