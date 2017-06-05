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


#define WIFI_EN

#define INFO 0
#define HEADER 1
#define BODY 2

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

uint8_t reception_flag = 0;

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];
static uint8_t gau8SocketTestBuffer2[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for TCP communication */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
static uint8_t gau8ReceivedBuffer2[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};


static uint8_t gau8SentBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

volatile uint8_t card_info[MAIN_WIFI_M2M_BUFFER_SIZE];  
volatile uint8_t server_info[MAIN_WIFI_M2M_BUFFER_SIZE];
volatile uint8_t file_content[MAIN_WIFI_M2M_BUFFER_SIZE];
volatile uint8_t header[MAIN_WIFI_M2M_BUFFER_SIZE];
volatile uint8_t **file_names;

	
volatile uint32 g_rxCnt = 0 ;
volatile uint8_t recv_flag = 0;
volatile uint8_t send_flag = 0;
volatile uint8_t terminate = 0;
volatile uint8_t first_try = 0;
volatile uint8_t http_flag = INFO;
volatile uint8_t number_of_files = 0;

char card_info_name[] = "0:info.txt";
Ctrl_status status;
FRESULT res;
FATFS fs;
FIL card_file;

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
uint8_t host_msg[]												= "GET /";
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

uint8_t **info_parser (uint8_t *info, int size, uint8_t **file_names) {
	int start = 0;
	int temp_c = 0;
	uint8_t file_c = 0;
	int i;
	char t;
	uint8_t *temp = malloc(sizeof(char)*100);

	
	while (i < strlen(info)) {
		t = (char) info[i];
		if (start == 1 && t == ':') {
			start = 0;
			temp_c = 0;
			file_names[file_c] = malloc(sizeof (char) * 100);
			memset(file_names[file_c], NULL, sizeof file_names[file_c]);
			strcpy(file_names[file_c], temp);
			//printf("%d, %s", file_c, file_names[file_c]);
			memset(temp, NULL, strlen(temp));
			file_c++;
		}
		
		if (start) {
			temp[temp_c] = info[i];		
			temp_c++;
		}
		
		if (t == '{' || t == ','){
			memset(temp, NULL, sizeof temp);
			start = 1;
		}
		i++;
	}
	
	number_of_files = file_c-1;
	return file_names;
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
		printf("\r\n");
	  //printf("TCP Server Test Complete!\r\n");
	  //printf("close socket\n");
	  //close(tcp_client_socket);
	  //close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *buffer = (tstrSocketRecvMsg *)pvMsg;
		
		sint16 nbytes = buffer->s16BufferSize;

	
    uint8_t  messageAck[64];
    uint16_t messageAckSize;
    uint8_t  command;
    uint8_t *temp;
		
		if (buffer && buffer->s16BufferSize > 0) {
			switch (http_flag)	{
				case INFO:							
					
					printf("INFO\r\n");
					memcpy(server_info, buffer->pu8Buffer, MAIN_WIFI_M2M_BUFFER_SIZE);
					
					// limpa o buffer de recepcao e tx
					memset(buffer->pu8Buffer, 0, buffer->s16BufferSize);
					
					printf("Parsing... \r\n") ;
					info_parser(server_info, sizeof(server_info), file_names);
					
					//printf(file_names[2]);
					printf("\r\n");
					printf("OK!");
					printf("\r\n");
					/*Numero de arquivos*/
					printf("Numero de arquivos: %d\r\n", number_of_files);
					
					
					uint16_t rtn;					
					uint8_t *substring = malloc(sizeof (char) * strlen(file_names[number_of_files])-1);

					memcpy( substring, &file_names[number_of_files][1], strlen(file_names[number_of_files])-2);
					substring[strlen(file_names[number_of_files])-2] = '\0';
					
					printf("Nome de arquivos: %s\r\n",  substring);

					memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
					sprintf((char *)gau8ReceivedBuffer, "%s%s%s%s", HOST_MSG, "file/", substring, HOST_MSG_SUFFIX);
					
					
					rtn = send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
					memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
					
					http_flag = HEADER;
					recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
					
					//memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
					//recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);				
					break;
					
				case HEADER:
				
					printf("HEADER\r\n");
					memcpy(header, buffer->pu8Buffer, MAIN_WIFI_M2M_BUFFER_SIZE);
				
					//printf(header);
					printf("\r\n");
					
					recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
					
					
					// limpa o buffer de recepcao e tx
					memset(buffer->pu8Buffer, 0, buffer->s16BufferSize);
					
					http_flag = BODY;
					break;
					
				case BODY:
				
				  memset(file_content, 0, MAIN_WIFI_M2M_BUFFER_SIZE);

				
					printf("BODY\r\n");
					
					memcpy(file_content, buffer->pu8Buffer, nbytes);
					
					printf("Conteudo no arquivo:\r\n");
					printf(file_content);
					printf("\r\nFim\r\n");
					
					/*SD Card Create File*/
					char file_name[] = "0:";
					
					uint8_t *subname = malloc(sizeof (char) * strlen(file_names[number_of_files])-1);

					memcpy( subname, &file_names[number_of_files][1], strlen(file_names[number_of_files])-2);
					subname[strlen(file_names[number_of_files])-2] = '\0';
	
					strcat(file_name, subname);
					printf("FILE NAME: %s", file_name);
					
					printf("Create file (f_open)...\r\n");
					file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
					res = f_open(&card_file,
					(char const *)file_name,
					FA_CREATE_ALWAYS | FA_WRITE);
					if (res != FR_OK) {
						printf("[FAIL] res %d\r\n", res);
					}
					printf("[OK]\r\n");
					
					printf("Write to info file (f_puts)...\r\n");
						
					//char server_info_2[] = "{\"teste.txt\":\"2017-05-24T19:45:57.911Z\",\"teste1.txt\":\"2017-05-30T16:06:12.858Z\",\"testecopy.txt\":\"2017-05-30T16:06:12.858Z\"}";
					
					//f_puts(file_content, &card_file);
					uint8_t temp = 0;
					
					for(int i=0; i<sizeof(file_content);i++) {
						
						f_putc(file_content[i], &card_file);
						//f_write(&card_file, file_content, MAIN_WIFI_M2M_BUFFER_SIZE, temp);
						
					}
					printf("\r\n\r\n");
					printf("OK \r\n");
					
					printf("Fechando arquivo \n");

					/* Close the file */
					f_close(&card_file);
					
					printf("[OK]\r\n");
					
					number_of_files--;
					printf("NUMBER OF FILES: %d\r\n", number_of_files);
										
					// limpa o buffer de recepcao e tx
					memset(buffer->pu8Buffer, NULL, sizeof (buffer->pu8Buffer));
					
						
					if(number_of_files == 0) {
						reception_flag = 1;
						close(tcp_client_socket);
						break;
					}
					if (number_of_files > 0) {
						uint16_t rtn;
						uint8_t *substring = malloc(sizeof (char) * strlen(file_names[number_of_files])-1);
						
						memcpy( substring, &file_names[number_of_files][1], strlen(file_names[number_of_files])-2);
						substring[strlen(file_names[number_of_files])-2] = '\0';
						
						memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
						sprintf((char *)gau8ReceivedBuffer, "%s%s%s%s", HOST_MSG, "file/", substring, HOST_MSG_SUFFIX);
						
						
						rtn = send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
						memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
						
						http_flag = HEADER;
						recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
					}
				
					break;
				
			}
			
		
		} else {
		printf("socket_cb: recv error!\r\n");
		close(tcp_client_socket);
		tcp_client_socket = -1;
	}       
	
	break; 
	}
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
	
	file_names = malloc(sizeof(uint8_t *)*20);

	
	

	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);
	
	
	/** SDCARD */
	irq_initialize_vectors();
	cpu_irq_enable();
	
	/* Initialize SD MMC stack */
	sd_mmc_init();
	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
	

	char temp_buffer[MAIN_WIFI_M2M_BUFFER_SIZE];
	
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
	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	if (FR_INVALID_DRIVE == res) {
		printf("[FAIL] res %d\r\n", res);
		goto main_end_of_test;
	}
	printf("[OK]\r\n");
	
#ifdef WIFI_EN_N

		printf("Create a file (f_open)...\r\n");
		card_info_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&card_file,
		(char const *)card_info_name,
		FA_CREATE_ALWAYS | FA_WRITE);
		if (res != FR_OK) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");

		printf("Write to test file (f_puts)...\r\n");
		
		int i;
		//pega cada elemento do buffer e os grava no cartão sd
		for(i=0; i<sizeof(imagem);i++)
			f_putc(imagem[i], &card_file);
	
			printf("[OK]\r\n");
			f_close(&card_file);
			printf("Test is successful.\n\r");
#endif

#ifdef WIFI_EN

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
	
	while (!reception_flag) {
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
#endif
	
	
	//for (int k = 0; k<MAIN_WIFI_M2M_BUFFER_SIZE; k++) {
	//	printf("%c", server_info[k]);
	//	if (k > 200) {
	//		delay_ms(20);
	//	}
	//}
		
	main_end_of_test:
	printf("Please unplug the card.\n\r");
	while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
	}

	return 0;
}

