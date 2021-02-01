/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

// default step
//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-size "${BuildArtifactFileName}"
//
// my step
//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <malloc.h>

#include "hdr.h"
#include "stm32f4xx_hal_adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_TMP_SIZE  256
#define MAX_FIFO_SIZE  64//32
#define MAX_UART_BUF 1024//512
#define MAX_VCC_BUF     8
#define MAX_COMP_BUF    8


#define _1ms 1
#define _2ms 2
#define _3ms 3
#define _4ms 4
#define _5ms 5
#define _10ms (_1ms * 10)
#define _15ms (_1ms * 15)
#define _20ms (_1ms * 20)
#define _30ms (_1ms * 30)
#define _40ms (_1ms * 40)
#define _50ms (_1ms * 50)
#define _100ms (_1ms * 100)
#define _110ms (_1ms * 110)
#define _120ms (_1ms * 120)
#define _125ms (_1ms * 125)
#define _130ms (_1ms * 130)
#define _140ms (_1ms * 140)
#define _150ms (_1ms * 150)
#define _200ms (_1ms * 200)
#define _210ms (_1ms * 210)
#define _220ms (_1ms * 220)
#define _230ms (_1ms * 230)
#define _240ms (_1ms * 240)
#define _250ms (_1ms * 250)
#define _300ms (_1ms * 300)
#define _350ms (_1ms * 350)
#define _400ms (_1ms * 400)
#define _500ms (_1ms * 500)
#define _600ms (_1ms * 600)
#define _700ms (_1ms * 700)
#define _800ms (_1ms * 800)
#define _900ms (_1ms * 900)
#define _1s (_1ms * 1000)
#define _1s5 (_1ms * 1500)
#define _2s (_1s * 2)//2000
#define _3s (_1s * 3)//3000
#define _4s (_1s * 4)//4000
#define _5s (_1s * 5)//5000
#define _10s (_1s * 10)//10000
#define _15s (_1s * 15)
#define _20s (_1s * 20)
#define _25s (_1s * 25)
#define _30s (_1s * 30)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

#ifdef SET_OLED_SPI
	const char gradus = 0x1f;
#else
	const char gradus = '^';
#endif
const char *cr_lf = "\r\n";

static evt_t evt_fifo[MAX_FIFO_SIZE] = {msg_empty};
uint8_t rd_evt_adr = 0;
uint8_t wr_evt_adr = 0;
uint8_t wr_evt_err = 0;
uint8_t cnt_evt = 0;
uint8_t max_evt = 0;

volatile uint32_t tik = 0;
volatile time_t epoch = 1610786184;
uint8_t tZone = 2;
volatile uint8_t setDate = 0;
char rxData[MAX_TMP_SIZE] = {0};
char rxByte = '\0';
uint8_t rx_uk = 0;
uint16_t rxBytes = 0;
char stx[MAX_TMP_SIZE] = {0};
char tmp[MAX_UART_BUF] = {0};
char txData[MAX_UART_BUF] = {0};
volatile uint8_t txDoneFlag = 1;
volatile uint8_t ledValue = 0;
uint32_t next = 0;
uint32_t msBegin = 0, msCnt = 0;
const uint32_t msPeriod = _100ms;
//
int siCmd = 0;
uint32_t tx_icnt = 0, rx_icnt = 0;
//

int jtune = 0;//set output in single string
uint8_t outMode = cjsonMode;
uint8_t outDebug = 0;

//
#if defined(SET_OLED_SPI) || defined(SET_IPS)
	SPI_HandleTypeDef *portOLED = NULL;
	char line[MAX_TMP_SIZE] = {0};
	uint32_t spi_cnt;
	char devName[32] = {0};//" - STM32F411 -  ";
#endif

#ifdef SET_BMx280
	I2C_HandleTypeDef *portBMP = NULL;
	uint8_t reg_id = 0;
	uint8_t data_rdx[DATA_LENGTH] = {0};
	size_t d_size = 6;
	bool bStat = false;
	bool bmxCalibr = false;
	char bmxName[16] = {0};
#endif

compas_data_t compData = {0.0, 0.0};
I2C_HandleTypeDef *portHMC = NULL;

ADC_HandleTypeDef *portADC = NULL;
volatile uint16_t adcValue = 0;
volatile float VccF = 0.0;
uint32_t iadc_cnt = 0;
uint8_t startADC = 1;
uint16_t VccBuf[MAX_VCC_BUF] = {0};
uint8_t VccValCounter = 0;

float CompBuf[MAX_COMP_BUF] = {0};
uint8_t CompValCounter = 0;

uint32_t one_sec = 0;
uint32_t seconds = 0;
volatile uint8_t devError = 0;
uint32_t tmr_out = 0;
uint32_t errStatCnt = 0;
uint32_t cikl_out = _10s;


#ifdef SET_MPU
	I2C_HandleTypeDef *portMPU = NULL;
	uint32_t cntMPU = 0;
	bool mpuPresent = false;
	uint8_t mpuStatus;
#endif


struct mallinfo mem_info;// = mallinfo();

#ifdef SET_IRED

	const one_key_t keyAll[MAX_IRED_KEY] = {
			{"irCH-",   0xe318261b},
			{"irCH",    0x00511dbb},
			{"irCH+",   0xee886d7f},
			{"irLEFT",  0x52a3d41f},
			{"irRIGHT", 0xd7e84b1b},
			{"irSP",    0x20fe4dbb},
			{"ir-",     0xf076c13b},
			{"ir+",     0xa3c8eddb},
			{"irEQ",    0xe5cfbd7f},
			{"ir100+",  0x97483bfb},
			{"ir200+",  0xf0c41643},
			{"ir0",     0xc101e57b},
			{"ir1",     0x9716be3f},
			{"ir2",     0x3d9ae3f7},
			{"ir3",     0x6182021b},
			{"ir4",     0x8c22657b},
			{"ir5",     0x488f3cbb},
			{"ir6",     0x0449e79f},
			{"ir7",     0x32c6fdf7},
			{"ir8",     0x1bc0157b},
			{"ir9",     0x3ec3fc1b}
	};

#endif


#ifdef SET_NET
	//#define HTTP_SOCKET     0
	//#define PORT_TCPS		9000
	//#define DATA_BUF_SIZE   2048

	//uint8_t gDATABUF[DATA_BUF_SIZE];

	char localIP[32] = {0};
	char netChipID[16] = {0};
	int dhcpSOC = 0;
	int udpSOC = 1;
	int tcpSOC = 2;
	uint8_t stat_dhcp = DHCP_STOPPED;
	volatile uint8_t ip_assigned = 0;
	volatile uint32_t net_cnt = 0;
	uint8_t udpAddr[4] = {101};
	uint16_t udpPort = 8004;
	uint16_t tcpPort = 8008;
	char udpText[128] = {0};
	int mlen = 0;//sprintf(udpText, "Message from device=%s ip=%s\r\n", netChipID, localIP);
	char tcpBuf[256] = {0};
	char tcpTmp[128] = {0};
	uint8_t con_tcp = 0;

	int32_t cnt_udp = 0;
	int32_t cnt_tcp = 0;
	uint32_t udp_pack_num = 0;
	uint8_t en_udp = 0;
	uint8_t en_tcp = 0;
	int8_t usoc = -1;
	int8_t tsoc = -1;


	wiz_NetInfo gWIZNETINFO = {
				.mac = {0x00, 0x08, 0xdc, 0x47, 0x47, 0x54},
				.ip = {192, 168, 0, 150},
				.sn = {255, 255, 255, 0},
				.gw = {192, 168, 0, 1},
				.dns = {0, 0, 0, 0},
				.dhcp = NETINFO_STATIC};

	//--------------------------------------------------------------------------
	void W5500_Reset()
	{
		HAL_GPIO_WritePin(NET_RST_GPIO_Port, NET_RST_Pin, GPIO_PIN_RESET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(NET_RST_GPIO_Port, NET_RST_Pin, GPIO_PIN_SET);
		HAL_Delay(20);
	}
	//--------------------------------------------------------------------------
	void W5500_Select() { CS_NET_SELECT(); }
	void W5500_Unselect() { CS_NET_DESELECT(); }
	//--------------------------------------------------------------------------
	void W5500_ReadBuff(uint8_t* buff, uint16_t len)
	{
	    if (HAL_SPI_Receive(&hspi4, buff, len, HAL_MAX_DELAY) != HAL_OK) devError |= devNet;
	}
	//--------------------------------------------------------------------------
	void W5500_WriteBuff(uint8_t* buff, uint16_t len)
	{
	    if (HAL_SPI_Transmit(&hspi4, buff, len, HAL_MAX_DELAY) != HAL_OK) devError |= devNet;
	}
	//--------------------------------------------------------------------------
	uint8_t W5500_ReadByte()
	{
	    uint8_t byte;
	    if (HAL_SPI_Receive(&hspi4, &byte, sizeof(uint8_t), HAL_MAX_DELAY) != HAL_OK) devError |= devNet;
	    return byte;
	}
	//--------------------------------------------------------------------------
	void W5500_WriteByte(uint8_t byte)
	{
		if (HAL_SPI_Transmit(&hspi4, &byte, sizeof(uint8_t), HAL_MAX_DELAY) != HAL_OK) devError |= devNet;
	}
	//--------------------------------------------------------------------------
	void Callback_IPAssigned() {
	    //UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
		getIPfromDHCP(gWIZNETINFO.ip);
		getGWfromDHCP(gWIZNETINFO.gw);
		getSNfromDHCP(gWIZNETINFO.sn);
		sprintf(localIP, "%d.%d.%d.%d", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
		memcpy(udpAddr, gWIZNETINFO.ip, 3);
	    ip_assigned = 1;
	}
	//--------------------------------------------------------------------------
	void Callback_IPConflict() {
	    //UART_Printf("Callback: IP conflict!\r\n");
		devError |= devNet;
	}
	//--------------------------------------------------------------------------
	static char *socStatus(int code)
	{
		switch (code) {
			case SOCK_CLOSED://            0x00
				return "Closed";
			case SOCKERR_SOCKNUM://       (SOCK_ERROR - 1)     ///< Invalid socket number
				return "Invalid socket number";
			case SOCKERR_SOCKOPT://       (SOCK_ERROR - 2)     ///< Invalid socket option
				return "Invalid socket option";
			case SOCKERR_SOCKINIT://      (SOCK_ERROR - 3)     ///< Socket is not initialized or SIPR is Zero IP address when Sn_MR_TCP
				return "Socket is not init or SIPR is Zero IP";
			case SOCKERR_SOCKCLOSED://    (SOCK_ERROR - 4)     ///< Socket unexpectedly closed.
				return "Socket unexpectedly closed";
			case SOCKERR_SOCKMODE://      (SOCK_ERROR - 5)     ///< Invalid socket mode for socket operation.
				return "Invalid socket mode";
			case SOCKERR_SOCKFLAG://      (SOCK_ERROR - 6)     ///< Invalid socket flag
				return "Invalid socket flag";
			case SOCKERR_SOCKSTATUS://    (SOCK_ERROR - 7)     ///< Invalid socket status for socket operation.
				return "Invalid socket status";
			case SOCKERR_ARG://           (SOCK_ERROR - 10)    ///< Invalid argument.
				return "Invalid argument";
			case SOCKERR_PORTZERO://      (SOCK_ERROR - 11)    ///< Port number is zero
				return "Zero Port number";
			case SOCKERR_IPINVALID://     (SOCK_ERROR - 12)    ///< Invalid IP address
				return "Invalid IP";
			case SOCKERR_TIMEOUT://       (SOCK_ERROR - 13)    ///< Timeout occurred
				return "Timeout";
			case SOCKERR_DATALEN://       (SOCK_ERROR - 14)    ///< Data length is zero or greater than buffer max size.
				return "Zero data length";
			case SOCKERR_BUFFER://        (SOCK_ERROR - 15)    ///< Socket buffer is not enough for data communication.
				return "Socket data buffer is not enough";
			case SOCK_LISTEN://                  0x14
				return "Listen";
			case SOCK_ESTABLISHED://             0x17
				return "Connected";
			case SOCK_FIN_WAIT://                0x18
				return "Disconnected";
			case SOCK_CLOSING://                 0x1A
				return "Socket closed";
			case SOCK_CLOSE_WAIT://              0x1C
				return "Client closed connection";
			default : return "Unkown";
		}
	}
#endif
	//--------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------------------------------------------
void led_OnOff()
{
	HAL_GPIO_TogglePin(tLED_GPIO_Port, tLED_Pin);
}
//-------------------------------------------------------------------------------------------
uint32_t getTimer(uint32_t t)
{
	return (tik + t);
}
int chkTimer(uint32_t t)
{
	return (tik >= t ? 1 : 0);
}
//-------------------------------------------------------------------------------------------
void setSec(uint32_t ep)
{
	//HAL_NVIC_DisableIRQ(TIM2_IRQn);
		seconds = ep;
	//HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
//-------------------------------------------------------------------------------------------
uint32_t getSec()
{
	return seconds;
}
//-------------------------------------------------------------------------------------------
void putMsg(evt_t evt)
{
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(ADC_IRQn);
#ifndef SET_COMPAS_BLOCK
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
#endif

	if (cnt_evt >= MAX_FIFO_SIZE) {
		wr_evt_err++;
	} else {
		evt_fifo[wr_evt_adr] = evt;
		cnt_evt++;
		if (wr_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			wr_evt_adr++;
		} else  {
			wr_evt_adr = 0;
		}
		wr_evt_err = 0;
		if (cnt_evt > max_evt) max_evt = cnt_evt;
	}

	if (wr_evt_err) devError |= devFifo;
		       else devError &= ~devFifo;

#ifndef SET_COMPAS_BLOCK
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
#endif
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
//-------------------------------------------------------------------------------------------
evt_t getMsg()
{
evt_t ret = msg_empty;

	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(ADC_IRQn);
#ifndef SET_COMPAS_BLOCK
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
#endif

	if (cnt_evt) {
		ret = evt_fifo[rd_evt_adr];
		if (cnt_evt) cnt_evt--;
		if (rd_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			rd_evt_adr++;
		} else {
			rd_evt_adr = 0;
		}
	}

#ifndef SET_COMPAS_BLOCK
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
#endif
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	return ret;
}
//-------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM2) {//1ms period
		tik++;
		if (!(tik % (_10ms))) putMsg(msg_10ms);
		//
		one_sec++;
		if (one_sec >= _1s) {
			one_sec = 0;
			seconds++;
			putMsg(msg_sec);
		}
		//
	}// else if (htim->Instance == TIM3) {
		//PWM pinA5
	//}
#ifdef SET_IRED
	else if (htim->Instance == TIM4) {
		uint8_t irdata = RECIV_PIN; // пин для приёма
		irparams.timer++;  // One more 50uS tick
		if (irparams.rawlen >= RAWBUF) irparams.rcvstate = STATE_OVERFLOW;  // Buffer overflow

		switch(irparams.rcvstate) {
			case STATE_IDLE: // In the middle of a gap
				if (irdata == MARK) {
					if (irparams.timer < GAP_TICKS) { // Not big enough to be a gap.
						irparams.timer = 0;
					} else {
						// Gap just ended; Record duration; Start recording transmission
						irparams.overflow = 0;
						irparams.rawlen  = 0;
						irparams.rawbuf[irparams.rawlen++] = irparams.timer;
						irparams.timer = 0;
						irparams.rcvstate = STATE_MARK;
					}
				}
			break;
			case STATE_MARK:  // Timing Mark
				if (irdata == SPACE) {// Mark ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_SPACE;
				}
			break;
			case STATE_SPACE:  // Timing Space
				if (irdata == MARK) {// Space just ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_MARK;
				} else if (irparams.timer > GAP_TICKS) {// Space
					irparams.rcvstate = STATE_STOP;
				}
			break;
			case STATE_STOP:  // Waiting; Measuring Gap
			 	if (irdata == MARK) irparams.timer = 0;  // Reset gap timer
			break;
			case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
				irparams.overflow = 1;
				irparams.rcvstate = STATE_STOP;
			break;
		}
	}
#endif
  /* USER CODE END Callback 1 */
}
//----------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1) {
		rxData[rx_uk & 0x7f] = rxByte;
		if (rxByte == '\n') {//end of line
			rxData[(rx_uk + 1) & 0x7f] = '\0';
			strcpy(stx, rxData);//, rx_uk + 1);
			putMsg(msg_rxDone);

			rx_uk = 0;
			memset(rxData, 0, sizeof(rxData));
		} else rx_uk++;

		HAL_UART_Receive_IT(huart, (uint8_t *)&rxByte, 1);

	}
}
//-----------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) txDoneFlag = 1;
}
//-----------------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) devError |= devUART;
}
//-----------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == iKEY_Pin) {
		devError = devOK;
		putMsg(msg_out);
	}
#ifdef SET_NET
	else if (GPIO_Pin == NET_EXTI4_Pin) {
		net_cnt++;
	}
#endif
}
//-----------------------------------------------------------------------------
#if defined(SET_BMx280) || defined(SET_COMPAS)
	void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
	{
		if (hi2c->Instance == I2C1) devError |= devI2C1;
	}
	//-----------------------------------------------------------------------------
	void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
	{
		if (hi2c->Instance == I2C1) {
			tx_icnt++;
			putMsg(msg_i2c);
		}
	}
	void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
	{
		if (hi2c->Instance == I2C1) {
			rx_icnt++;
			putMsg(msg_i2c);
		}
	}
#endif
//-----------------------------------------------------------------------------
#if defined(SET_OLED_SPI) || defined(SET_IPS)
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2) {
#if defined(SET_OLED_SPI)
		if (withDMA) {
			CS_OLED_DESELECT();
			spiRdy = 1;
		}
		spi_cnt++;
#elif defined(SET_IPS)
	#ifdef SET_WITH_CS
		ST7789_SelOFF();
	#endif
		dma_spi2_cnt--;
		if (!dma_spi2_cnt) {
			HAL_SPI_DMAStop(portOLED);
			dma_spi2_flag = dma_spi2_cnt = 1;
		}
		spi_cnt++;
#endif
	}
#ifdef SET_NET
	else if (hspi->Instance == SPI4) {
		//
	}
#endif
}
//-----------------------------------------------------------------------------
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if ((hspi->Instance == SPI2) || (hspi->Instance == SPI4)) devError |= devSPI;
}
#endif
//-----------------------------------------------------------------------------
int sec_to_str_time(char *st)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	return (sprintf(st, "%02u.%02u %02u:%02u:%02u",
						sDate.Date, sDate.Month,
						sTime.Hours, sTime.Minutes, sTime.Seconds));
}
//-------------------------------------------------------------------------------------------
void Report(const char *tag, unsigned char addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;
char *buff = &txData[0];

	if (!txDoneFlag) return;

//	char *buff = (char *)calloc(1, len);//pvPortMalloc(len);//vPortFree(buff);
//	if (buff) {
		txDoneFlag = 0;

		if (addTime) dl = sec_to_str_time(buff);
		if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);

		va_start(args, fmt);
		vsnprintf(buff + dl, len - dl, fmt, args);
		va_end(args);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buff, strlen(buff));

		/*while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
				if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX) break;
				//HAL_Delay(1);
		}*/

//		free(buff);//vPortFree(buff);

#ifdef SET_NET
		if (con_tcp) {
			send(tcpSOC, (uint8_t *)buff, strlen(buff));
		}
#endif
//	}
}
//------------------------------------------------------------------------------------------
void set_Date(time_t ep)
{
struct tm ts;

	//setSec((uint32_t)ep);

	if (!localtime_r(&ep, &ts)) return;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	sTime.Hours   = ts.tm_hour + tZone;
	sTime.Minutes = ts.tm_min;
	sTime.Seconds = ts.tm_sec;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) setDate = 1;
	}

}
//
//------------------------------------------------------------------------------------------
uint32_t get_DateTime()
{
struct tm ts;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return 0;
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return 0;

	ts.tm_hour = sTime.Hours;
	ts.tm_min  = sTime.Minutes;
	ts.tm_sec  = sTime.Seconds;

	ts.tm_wday = sDate.WeekDay;
	ts.tm_mon  = sDate.Month - 1;
	ts.tm_mday = sDate.Date;
	ts.tm_year = sDate.Year;

	return ((uint32_t)mktime(&ts));
}
//-------------------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		iadc_cnt++;
		startADC = 0;
		putMsg(msg_adcReady);
	}
}
//-------------------------------------------------------------------------------------------
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) devError |= devADC;
}
//-------------------------------------------------------------------------------------------
uint8_t VccAddVal(uint16_t value)
{
int8_t i;

	if (!VccValCounter) {
		for (i = 0; i < MAX_VCC_BUF; i++) VccBuf[i] = value;
		VccValCounter = MAX_VCC_BUF;
	} else {
		for (i = MAX_VCC_BUF - 2; i >= 0; i--) VccBuf[i + 1] = VccBuf[i];
		VccBuf[0] = value;
		if (VccValCounter < MAX_VCC_BUF) VccValCounter++;
	}
    return VccValCounter;
}
//-------------------------------------------------------------------------------------------
uint8_t CompAddVal(float value)
{
int8_t i;

	if (!CompValCounter) {
		for (i = 0; i < MAX_COMP_BUF; i++) CompBuf[i] = value;
		CompValCounter = MAX_COMP_BUF;
	} else {
		for (i = MAX_COMP_BUF - 2; i >= 0; i--) CompBuf[i + 1] = CompBuf[i];
		CompBuf[0] = value;
		if (CompValCounter < MAX_COMP_BUF) CompValCounter++;
	}
    return CompValCounter;
}
//-------------------------------------------------------------------------------------------
void procCompas()
{
	cStat = COPMAS_ReadStat() & 1;
	if (cStat) {
		if (COPMAS_GetAngle() != HAL_OK) devError |= devI2C1;
	}
}
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
void ackParse()
{
char *uk, *uke;

	if ((uk = strstr(stx, "epoch=")) != NULL) {
		uk += 6;
		if (strlen(uk) >= 10) {
			uke = strchr(uk, ':');
			if (uke) {
				tZone = atoi(uke + 1);
				*uke = '\0';
			} else tZone = 0;
			uint32_t cep = atol(uk);
			if (cep > 0) set_Date((time_t)cep);
		}
	}
#ifdef SET_NET
	else if (strstr(stx, "udp_on") != NULL) {
		en_udp = 1;
	} else if (strstr(stx, "udp_off") != NULL) {
		en_udp = 0;
		if (usoc == udpSOC) close(udpSOC);
		usoc = -1;
		cnt_udp = 0;
		udp_pack_num = 0;
	}
	else if (strstr(stx, "tcp_on") != NULL) {
		en_tcp = 1;
	} else if (strstr(stx, "tcp_off") != NULL) {
		en_tcp = 0;
		if (tsoc == tcpSOC) close(tcpSOC);
		tsoc = -1;
		cnt_tcp = 0;
		con_tcp = 0;
	}
#endif
	else if (strstr(stx, "jtune_on") != NULL) {//key_100
		jtune = 1;
	} else if (strstr(stx, "jtune_off") != NULL) {//key_200
		jtune = 0;
	}
#ifdef SET_cJSON
	else if (strstr(stx, "cjson") != NULL) {//key_2
		outMode = cjsonMode;
	}
#endif
	else if (strstr(stx, "json") != NULL) {//key_1
		outMode = jsonMode;
	} else if (strstr(stx, "text") != NULL) {//key_0
		outMode = textMode;
	}
	else if (strstr(stx, "dbg_on") != NULL) {//key_3
		outDebug = 1;
	} else if (strstr(stx, "dbg_off") != NULL) {//key_4
		outDebug = 0;
	} else if (strstr(stx, "get") != NULL) {//key_eq
		tmr_out = getTimer(_1ms);
	} else if (strstr(stx, "rst") != NULL) {//key_ch
		putMsg(msg_rst);
	} else if ((uk = strstr(stx, "period=")) != NULL) {//key_minus : -100ms, key_plus : +100ms
		int val = atoi(uk + 7);
		if ((val > 0) && (val <= 30000)) cikl_out = val;
	}

}
//------------------------------------------------------------------------------------------
char *printOut(char *tmp)
{

switch (outMode) {

#ifdef SET_cJSON
	case cjsonMode:
	{
		cJSON *obj = cJSON_CreateObject();
		if (obj) {
			char dt[32] = {0};
			sec_to_str_time(dt);
			cJSON_AddItemToObject(obj, "time", cJSON_CreateString(dt));
			cJSON_AddItemToObject(obj, "ms", cJSON_CreateNumber(tik));
			cJSON *arr1 = cJSON_CreateArray();
			if (arr1) {
				cJSON_AddItemToArray(arr1, cJSON_CreateNumber((int)cnt_evt));
				cJSON_AddItemToArray(arr1, cJSON_CreateNumber((int)max_evt));
				cJSON_AddItemToObject(obj, "fifo", arr1);
			} else devError |= devMem;
			cJSON_AddItemToObject(obj, "devError", cJSON_CreateNumber(devError));
			cJSON_AddItemToObject(obj, "volt", cJSON_CreateNumber(VccF));
			//
#ifdef SET_BMx280
			cJSON *sens1 = cJSON_CreateObject();
			if (sens1) {
				cJSON_AddItemToObject(sens1, "pres", cJSON_CreateNumber(sensors.bmx_pres));
				cJSON_AddItemToObject(sens1, "temp", cJSON_CreateNumber(sensors.bmx_temp));
				if (reg_id == BME280_SENSOR) cJSON_AddItemToObject(sens1, "humi", cJSON_CreateNumber(sensors.bmx_humi));
				cJSON_AddItemToObject(obj, bmxName, sens1);
			} else devError |= devMem;
#endif
			//
			cJSON *sens2 = cJSON_CreateObject();
			if (sens2) {
				cJSON_AddItemToObject(sens2, "azimut", cJSON_CreateNumber(compData.angleHMC));
				cJSON_AddItemToObject(sens2, "temp2", cJSON_CreateNumber(compData.tempHMC));
				cJSON_AddItemToObject(obj, "QMC5883L", sens2);
			} else devError |= devMem;
			//
			cJSON *sens3 = cJSON_CreateObject();
			if (sens3) {
				cJSON_AddItemToObject(sens3, "stat", cJSON_CreateNumber(mpuStatus));
				cJSON_AddItemToObject(sens3, "temp3", cJSON_CreateNumber(mpu_data.TEMP));
				cJSON *arr2 = cJSON_CreateArray();
				if (arr2) {
					cJSON_AddItemToArray(arr2, cJSON_CreateNumber(mpu_data.xACCEL));
					cJSON_AddItemToArray(arr2, cJSON_CreateNumber(mpu_data.yACCEL));
					cJSON_AddItemToArray(arr2, cJSON_CreateNumber(mpu_data.zACCEL));
					cJSON_AddItemToObject(sens3, "accel", arr2);
				} else devError |= devMem;
				cJSON *arr3 = cJSON_CreateArray();
				if (arr3) {
					cJSON_AddItemToArray(arr3, cJSON_CreateNumber(mpu_data.xGYRO));
					cJSON_AddItemToArray(arr3, cJSON_CreateNumber(mpu_data.yGYRO));
					cJSON_AddItemToArray(arr3, cJSON_CreateNumber(mpu_data.zGYRO));
					cJSON_AddItemToObject(sens3, "gyro", arr3);
				} else devError |= devMem;
				cJSON_AddItemToObject(obj, "MPU6050", sens3);
			} else devError |= devMem;
			//
			char *st = cJSON_PrintBuffered(obj, sizeof(tmp) - 1, jtune);
			if (st) {
				strcpy(tmp, st);
				strcat(tmp, ">");
				free(st);
			}

			cJSON_Delete(obj);

		} else devError |= devMem;
	}
	break;
#endif

	case jsonMode:
	{
		char dt[32] = {0};
		sec_to_str_time(dt);
		if (jtune) {
			sprintf(tmp, "{%s  \"time\": \"%s\",%s  \"ms\": %lu,%s  \"fifo\": [%u,%u]",
					cr_lf, dt, cr_lf, tik, cr_lf, cnt_evt, max_evt);
			sprintf(tmp+strlen(tmp), ",%s  \"devError\": %X", cr_lf, devError);
			sprintf(tmp+strlen(tmp), ",%s  \"volt\": %.3f", cr_lf, VccF);
#ifdef SET_BMx280
			sprintf(tmp+strlen(tmp), ",%s  \"%s\": {%s    \"pres\": %.2f,%s    \"temp\": %.2f",
					cr_lf, bmxName, cr_lf, sensors.bmx_pres, cr_lf, sensors.bmx_temp);
			if (reg_id == BME280_SENSOR) sprintf(tmp+strlen(tmp), ",%s    \"humi\": %.2f", cr_lf, sensors.bmx_humi);
			sprintf(tmp+strlen(tmp), "%s  }", cr_lf);
#endif
			sprintf(tmp+strlen(tmp), ",%s  \"QMC5883L\": {%s    \"azimut\": %.2f,%s    \"temp2\": %.2f%s  }" ,
					cr_lf, cr_lf, compData.angleHMC, cr_lf, compData.tempHMC, cr_lf);
#ifdef SET_MPU
			sprintf(tmp+strlen(tmp), ",%s  \"MPU6050\": {%s    \"stat\": %X,%s    \"temp3\": %.2f,%s    \"accel\": [%d,%d,%d],%s    \"gyro\": [%d,%d,%d]%s  }%s",
							cr_lf, cr_lf,
							mpuStatus, cr_lf,
							mpu_data.TEMP, cr_lf,
							mpu_data.xACCEL, mpu_data.yACCEL, mpu_data.zACCEL, cr_lf,
							mpu_data.xGYRO, mpu_data.yGYRO, mpu_data.zGYRO, cr_lf, cr_lf);
#endif
		} else {
			sprintf(tmp, "{\"time\": \"%s\",\"ms\": %lu,\"fifo\": [%u,%u]", dt, tik, cnt_evt, max_evt);
			sprintf(tmp+strlen(tmp), ",\"devError\": %X", devError);
			sprintf(tmp+strlen(tmp), ",\"volt\": %.3f", VccF);
#ifdef SET_BMx280
			sprintf(tmp+strlen(tmp), ",\"%s\": {\"press\": %.2f,\"temp\": %.2f",
					bmxName, sensors.bmx_pres, sensors.bmx_temp);
			if (reg_id == BME280_SENSOR) sprintf(tmp+strlen(tmp), ",\"humi\": %.2f", sensors.bmx_humi);
			strcat(tmp, "}");
#endif
			sprintf(tmp+strlen(tmp), ",\"QMC5883L\": {\"azimut\": %.2f,\"temp2\": %.2f}", compData.angleHMC, compData.tempHMC);
#ifdef SET_MPU
			sprintf(tmp+strlen(tmp), ",\"MPU6050\": {\"stat\": %X,\"temp3\": %.2f,\"accel\": [%d,%d,%d],\"gyro\": [%d,%d,%d]}",
							mpuStatus,
							mpu_data.TEMP,
							mpu_data.xACCEL, mpu_data.yACCEL, mpu_data.zACCEL,
							mpu_data.xGYRO, mpu_data.yGYRO, mpu_data.zGYRO);
#endif

		}
		strcat(tmp, "}>");
	}
	break;

	default : {// textMode

		sprintf(tmp, " | ms=%lu fifo:%u/%u", tik, cnt_evt, max_evt);
		sprintf(tmp+strlen(tmp), " devError=%X", devError);
		sprintf(tmp+strlen(tmp), " | Vcc=%.3f", VccF);
#ifdef SET_BMx280
		sprintf(tmp+strlen(tmp)," | %s: pres=%.2f temp=%.2f",
								bmxName, sensors.bmx_pres, sensors.bmx_temp);
		if (reg_id == BME280_SENSOR) sprintf(tmp+strlen(tmp), " humi=%.2f", sensors.bmx_humi);
#endif
		//compas_stat_t *cStat = (compas_stat_t *)confRegHmc;
		sprintf(tmp+strlen(tmp)," | QMC5883L: azimut=%.2f temp2=%.2f",
			    			compData.angleHMC,
							compData.tempHMC);
#ifdef SET_MPU
		sprintf(tmp+strlen(tmp)," | MPU6050: stat=%X temp3=%.2f accel=%d,%d,%d gyro=%d,%d,%d",
							mpuStatus,
							mpu_data.TEMP,
							mpu_data.xACCEL, mpu_data.yACCEL, mpu_data.zACCEL,
							mpu_data.xGYRO, mpu_data.yGYRO, mpu_data.zGYRO);
#endif

	}
}

	if (outDebug) {
		mem_info = mallinfo();
		sprintf(tmp+strlen(tmp), "\r\n(noused=%d #freeChunk=%d #freeBlk=%d #mapReg=%d busy=%d max_busy=%d #freedBlk=%d used=%d free=%d top=%d)",
			mem_info.arena,// Non-mmapped space allocated (bytes)
			mem_info.ordblks,// Number of free chunks
			mem_info.smblks,// Number of free fastbin blocks
			mem_info.hblks,// Number of mmapped regions
			mem_info.hblkhd,// Space allocated in mmapped regions (bytes)
			mem_info.usmblks,// Maximum total allocated space (bytes)
			mem_info.fsmblks,// Space in freed fastbin blocks (bytes)
			mem_info.uordblks,// Total allocated space (bytes)
			mem_info.fordblks,// Total free space (bytes)
			mem_info.keepcost);// Top-most, releasable space (bytes)
	}

	return &tmp[0];
}
//-------------------------------------------------------------------------------------------
void toDisplay(char *line)
{
	line[0] = '\0';

#if defined(SET_OLED_SPI)
	if (!devError) sprintf(line+strlen(line), "  VCC:%.3fv\n", VccF);
			  else sprintf(line+strlen(line), " devError:0x%02X\n", devError);
	//if (mpuPresent) sprintf(line+strlen(line), " mpuTemp:%.2f%c\n", mpu_data.TEMP, gradus);
	sprintf(line+strlen(line), "  pres:%.2fmm\n  temp:%.2f%c\n", sensors.bmx_pres, sensors.bmx_temp, gradus);
	if (reg_id == BME280_SENSOR) sprintf(line+strlen(line), "  humi:%.2f%%\n", sensors.bmx_humi);
	sprintf(line+strlen(line), " azimut:%.2f%c\n", compData.angleHMC, gradus);
	if (ip_assigned) sprintf(line+strlen(line), " %s", localIP);
	spi_ssd1306_text_xy(line, 2, 2);
#elif defined(SET_IPS)
	if (!devError) sprintf(line+strlen(line), "    Vcc:%.3fv\n", VccF);
			  else sprintf(line+strlen(line), "   devError:0x%02X\n", devError);
	if (mpuPresent) {
		sprintf(line+strlen(line), "   mpuTemp:%.2f^\n", mpu_data.TEMP);
		sprintf(line+strlen(line), "   mpuAccel:%d,%d,%d\n", mpu_data.xACCEL, mpu_data.yACCEL, mpu_data.zACCEL);
		sprintf(line+strlen(line), "   mpuGyro:%d,%d,%d\n", mpu_data.xGYRO, mpu_data.yGYRO, mpu_data.zGYRO);
	}
	sprintf(line+strlen(line), "   pres:%.2fmm\n   temp:%.2f^\n", sensors.bmx_pres, sensors.bmx_temp);
	if (reg_id == BME280_SENSOR) sprintf(line+strlen(line), "   humi:%.2f%%\n", sensors.bmx_humi);
	sprintf(line+strlen(line), "   azimut:%.2f^\n   temp2:%.2f^\n", compData.angleHMC, compData.tempHMC);
	if (ip_assigned) sprintf(line+strlen(line), "%s | %d", localIP, (int)cnt_udp);
	ST7789_WriteString(0, tFont.height + (tFont.height * 0.75), line, tFont, WHITE, BLACK);
#endif
//
}
//-------------------------------------------------------------------------------------------

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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


#if defined(SET_OLED_SPI) || defined(SET_IPS)
    sprintf(devName, "Speed:%lu", huart1.Init.BaudRate);
#endif

  	/* start timer1 + interrupt
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);*/
    //"start" rx_interrupt
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);


	evt_t evt;
	uint32_t schMS = 0;
	uint8_t setRTC = 0;

	ON_ERR_LED();//!!!!!!!!!!!!!!!!!!!!!
	STROB_UP();//!!!!!!!!!!!!!!!!!!!!!

#if defined(SET_OLED_SPI) || defined(SET_IPS)
	portOLED = &hspi2;
#endif
#if defined(SET_OLED_SPI)
    spi_ssd1306_Reset();
    //spi_ssd1306_on(1);//screen ON
    spi_ssd1306_init();//screen INIT
    spi_ssd1306_pattern();//set any params for screen
    //spi_ssd1306_invert();
    spi_ssd1306_clear();//clear screen

    int8_t len8 = 0;

#elif defined(SET_IPS)

    ST7789_Reset();
    ST7789_Init();

    FontDef fntKey = Font_16x26;
    FontDef tFont = Font_11x18;

    //uint8_t on_ips = 0;
    //ipsOn(on_ips);

    ST7789_Fill(0, 0, ST7789_WIDTH - 1, fntKey.height, WHITE);
    ST7789_Fill(0, ST7789_WIDTH - fntKey.height, ST7789_WIDTH - 1, ST7789_HEIGHT - 1, YELLOW);

#endif


#ifdef SET_BMx280
    portBMP = &hi2c1;

    i2c_reset_bmx280(&reg_id);
    if (reg_id == BME280_SENSOR) {
    	d_size = 8;
    	strcpy(bmxName, bme280s);
    } else {
    	d_size = 6;
    	if (reg_id == BMP280_SENSOR) strcpy(bmxName, bmp280s); else strcpy(bmxName, bmx280s);
    }

    //bmxCalibr = bmx280_readCalibrationData(reg_id);

#endif

#ifdef SET_MPU
    portMPU = &hi2c1;
    if (mpuID() == HAL_OK) {
    	if (mpuInit() == HAL_OK) {
    		mpuPresent = true;
    		mpuDisableInterrupts();
    	}
    }
#endif


#ifdef SET_NET

	//reset device
	W5500_Reset();

	reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

	uint8_t tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
	uint8_t rx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};

	if (wizchip_init(tx_buff_sizes, rx_buff_sizes) != 0) {
		devError |= devNet;
	} else {
		memset(netChipID, 0, sizeof(netChipID));
		ctlwizchip(CW_GET_ID, (void *)&netChipID[0]);
		if (gWIZNETINFO.dhcp == NETINFO_STATIC) {//STATIC ADDR
			ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);
			ctlnetwork(CN_GET_NETINFO, (void *)&gWIZNETINFO);
			sprintf(localIP, "%d.%d.%d.%d", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);

			ip_assigned = 1;
			memcpy(&udpAddr[0], (uint8_t *)&gWIZNETINFO.ip[0], 3);
			udpAddr[3] = 255;
		} else {//ADDR from DHCP
			ip_assigned = 0;
			uint8_t dhcpBuf[1024];

			setSHAR(gWIZNETINFO.mac);
			DHCP_init(dhcpSOC, dhcpBuf);
			reg_dhcp_cbfunc(Callback_IPAssigned, Callback_IPAssigned, Callback_IPConflict);

			if (!ip_assigned) {
				if (!((stat_dhcp == DHCP_IP_ASSIGN) ||
						(stat_dhcp == DHCP_IP_CHANGED) ||
							(stat_dhcp == DHCP_FAILED) ||
								(stat_dhcp == DHCP_IP_LEASED))) stat_dhcp = DHCP_run();
			}
		}
	}

	uint8_t netFlag = SF_IO_NONBLOCK | SF_BROAD_BLOCK;//SF_UNI_BLOCK;//SF_MULTI_BLOCK;//SF_BROAD_BLOCK;//SF_IO_NONBLOCK

	int32_t stat_udp;
	const uint8_t udp_sec_period = 10;
	uint8_t udp_sec = 5;

	int8_t stat_tcp = SOCKERR_SOCKCLOSED;
	uint8_t cliIP[4] = {0};
	uint8_t statSR = 0;

#endif


    //---------------------------------------
	//"start" ADC interrupt
	portADC = &hadc1;
	if (startADC) HAL_ADC_Start_IT(portADC);
    //
    // start timer1 + interrupt
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);
    //
    //---------------------------------------


#ifdef SET_IRED
    HAL_GPIO_WritePin(irLED_GPIO_Port, irLED_Pin, GPIO_PIN_SET);
	uint32_t tmr_ired = 0;
	enIntIRED();
#endif


	portHMC = &hi2c1;
	xyz = (xyz_t *)&magBuf[0];
//	COPMAS_Reset();
	COPMAS_Init();
	cStat = COPMAS_ReadStat();
#ifdef SET_COMPAS_BLOCK
    siCmd = msg_empty;
    evt = msg_startCompas;
#else
    siCmd = msg_startCompas;
    evt = msg_i2c;
    next = getTimer(_100ms);
#endif

    tmr_out = getTimer(_1s);
    next = getTimer(0);
    errStatCnt = 0;

    putMsg(evt);


#ifdef SET_NET
    en_udp = 1;
    if (usoc < 0) putMsg(msg_mkUdp);
    HAL_Delay(10);
    en_tcp = 1;
    if (tsoc < 0) putMsg(msg_mkTcp);
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)  {

#ifdef SET_IRED
		if (!tmr_ired) {
			if (decodeIRED(&results)) {
				tmr_ired = getTimer(_300ms);
				IRRED_LED();
				int8_t kid = -1;
				for (int8_t i = 0; i < MAX_IRED_KEY; i++) {
					if (results.value == keyAll[i].code) {
						kid = i;
						break;
					}
				}
#if defined(SET_OLED_SPI)
				int8_t k;
				if (kid == -1) k = sprintf(line, " CODE:%08lX ", results.value);
				          else k = sprintf(line, " irKEY: %s ", keyAll[kid].name);
				if (k < len8) spi_ssd1306_clear_line(8);
				len8 = k;
				spi_ssd1306_text_xy(line, 1, 8);
#elif defined(SET_IPS)
				if (kid == -1) sprintf(line, "CODE:%08lX", results.value);
				          else sprintf(line, "irKEY: %s", keyAll[kid].name);
				//ST7789_Fill(0, ST7789_WIDTH - fntKey.height, ST7789_WIDTH - 1, ST7789_HEIGHT - 1, YELLOW);
				ST7789_WriteString(0, ST7789_WIDTH - fntKey.height, mkLineCenter(line, ST7789_WIDTH / fntKey.width), fntKey, MAGENTA, YELLOW);
#endif
				if (kid != -1) {
					switch (kid) {
						case key_100: jtune = 1; break;
						case key_200: jtune = 0; break;

						case key_minus:
							if (cikl_out >= 200) cikl_out -= 100;
						break;
						case key_plus:
							if (cikl_out < 30000) cikl_out += 100;
						break;

						case key_0: outMode = textMode;  break;
						case key_1: outMode = jsonMode;  break;
						case key_2: outMode = cjsonMode; break;

						case key_3: outDebug = 1; break;//dbg_on
						case key_4: outDebug = 0; break;//dbg_off

						case key_ch: putMsg(msg_rst); break;//rst

						case key_eq: tmr_out = getTimer(_1ms); break;//out

						case key_left://udp_on / udp_off
							en_udp++; en_udp &= 1;
							if (!en_udp) {
								if (usoc == udpSOC) close(udpSOC);
								usoc = -1;
								cnt_udp = 0;
								udp_pack_num = 0;
							}
						break;
						case key_right://tcp_on / tcp_off
						{
							en_tcp++; en_tcp &= 1;
							if (!en_tcp) {
								if (tsoc == tcpSOC) {
									if (con_tcp) disconnect(tcpSOC);
									close(tcpSOC);
								}
								tsoc = -1;
								con_tcp = 0;
	#ifdef SET_NET_DEBUG
								uint8_t sta = getSn_SR(tcpSOC);
								Report(NULL, 0, "\t[%s] (%u) Device closed tcp server\r\n", socStatus(sta), sta);
	#endif
							}
						}
						break;
					}
				}
			}
		}

		if (tmr_ired) {
			if (chkTimer(tmr_ired)) {
				tmr_ired = 0;
				resumeIRED();
				IRRED_LED();
			}
		}
#endif

		switch (getMsg()) {

			case msg_mkUdp:
				if (usoc < 0) {
					usoc = socket(udpSOC, Sn_MR_UDP, udpPort, netFlag);
				}
			break;
			case msg_mkTcp:
				if (tsoc < 0) {
					tsoc = socket(tcpSOC, Sn_MR_TCP, tcpPort, netFlag);
					if (tsoc == tcpSOC) {
						stat_tcp = listen(tcpSOC);
#ifdef SET_NET_DEBUG
						Report(NULL, 0, "\tTcp socket create OK. Listen (%d) tcp client...\r\n", stat_tcp);
#endif
						putMsg(msg_mkListen);
					}
				}
			break;
			case msg_mkListen:
				statSR = getSn_SR(tcpSOC);
				//if (statSR == SOCK_LISTEN) {
				//	putMsg(msg_mkListen);
				//} else
				if (statSR == SOCK_ESTABLISHED) {
					if (!con_tcp) {
						getsockopt(tcpSOC, SO_DESTIP, &cliIP);
						con_tcp = 1;
						memset(tcpBuf, 0, sizeof(tcpBuf));
#ifdef SET_NET_DEBUG
						Report(NULL, 0, "\t[%s] Connect to client:  %d.%d.%d.%d\r\n", socStatus(statSR), cliIP[0], cliIP[1], cliIP[2], cliIP[3]);
#endif
						/*en_udp = 0;
						if (usoc == udpSOC) {
							close(udpSOC);
							usoc = -1;
							cnt_udp = 0;
							udp_pack_num = 0;
						}*/
					}
				} else if (statSR < 0) {
					close(tcpSOC);
					tsoc = -1;
					con_tcp = 0;
#ifdef SET_NET_DEBUG
					Report(NULL, 0, "\t[%s] Socket error %d\r\n", socStatus(statSR), statSR);
#endif
				} else {
					putMsg(msg_mkListen);
				}
			break;
		    case msg_adcReady:
		    {
		    	adcValue = (uint16_t)HAL_ADC_GetValue(portADC) & 0xfff;// * 0.80586;
		    	//
		    	if (VccAddVal(adcValue) == MAX_VCC_BUF) {//в окне накоплено MAX_VCC_BUF выборок -> фильтрация !
		    		uint16_t sum = 0;
		    		for (int8_t j = 0; j < MAX_VCC_BUF; j++) sum += VccBuf[j];
		    		sum /= MAX_VCC_BUF;
		    		VccF = (float)(sum * 0.80586) / 1000;
		    	}
		    	//
		    	startADC = 1;
		    }
		    break;
		    case msg_10ms:
		    	schMS++;
		    	//
#ifdef SET_NET
		    	if (usoc < 0) {
		    		if (en_udp) putMsg(msg_mkUdp);
		    	}
		    	//
		    	if (tsoc < 0) {
		    		if (en_tcp) putMsg(msg_mkTcp);
		    	} else {
		    		statSR = getSn_SR(tcpSOC);
		    		if (statSR == SOCK_CLOSE_WAIT) {
		    			if (tsoc == tcpSOC) close(tcpSOC);
		    			tsoc = -1;
		    			con_tcp = 0;
	#ifdef SET_NET_DEBUG
		    			Report(NULL, 0, "\t[%s] (%d) Disconnect client:  %d.%d.%d.%d\r\n",
		    								socStatus(statSR), statSR,
											cliIP[0], cliIP[1], cliIP[2], cliIP[3]);
	#endif
		    		}
		    		if (con_tcp) {
		    			int32_t rlen = recv(tcpSOC, (uint8_t *)tcpTmp, 64);
		    			if (rlen > 0) {
		    				if (sizeof(tcpBuf) > (rlen + strlen(tcpBuf))) {
		    					strncat(tcpBuf, tcpTmp, rlen);
		    					if (strstr(tcpBuf, "\r\n")) {
		    						int rl = strlen(tcpBuf);
		    						strncpy(stx, tcpBuf, rl);
		    						tcpBuf[0] = '\0';
		    						stx[rl] = '\0';
		    						putMsg(msg_rxDone);
		    					}
		    				} else {
		    					tcpBuf[0] = '\0';
		    				}
		    			}
		    		}
		    	}
#endif
		    	//
		    	if (tmr_out) {
		    		if (chkTimer(tmr_out)) {
		    			tmr_out = 0;
		    			putMsg(msg_out);
		    		}
		    	}
		    	//
		    	if (!setRTC) {
		    		setRTC = 1;
		    		set_Date(epoch);
		    		setSec(get_DateTime());
		    	}
		    	//
		    	if (!(schMS % (_100ms))) {// 100ms
		    		if (startADC) HAL_ADC_Start_IT(portADC);
		    		if (devError) ON_ERR_LED()
		    		         else OFF_ERR_LED()
		    		//
		    	}
		    	//
		    break;
		    case msg_sec:
		    	led_OnOff();
		    	//
#if defined(SET_OLED_SPI) || defined(SET_IPS)
		    	sec_to_str_time(line);
#endif
#if defined(SET_OLED_SPI)
		    	//print time
		    	spi_ssd1306_text_xy(line, 2, 1);
		    	toDisplay(line);
#elif defined(SET_IPS)
		    	//print time
		    	ST7789_WriteString(0, 0, mkLineCenter(line, ST7789_WIDTH / fntKey.width), fntKey, RED, WHITE);
		    	toDisplay(line);
		    	//netChipID
		    	sprintf(line, "chip : %s", netChipID);
		    	ST7789_WriteString(0, ST7789_WIDTH - fntKey.height, mkLineCenter(line, ST7789_WIDTH / fntKey.width), fntKey, MAGENTA, YELLOW);
#endif
		    	//
#ifdef SET_NET
		    	if (ip_assigned) {
		    		if (usoc < 0) putMsg(msg_mkUdp);
		    		else {
		    			udp_sec--;
		    			if (!udp_sec) {
		    				udp_sec = udp_sec_period;
		    				if (en_udp) {
		    					sec_to_str_time(line);
		    					mlen = sprintf(udpText, "[%ld] %s | device=%s ip=%s\r\n", ++udp_pack_num, line, netChipID, localIP);
		    					stat_udp = sendto(udpSOC, (uint8_t *)udpText, mlen, udpAddr, udpPort);
		    					if (netFlag & SF_IO_NONBLOCK) {
		    						if (stat_udp == mlen) cnt_udp++;
		    						else {
		    							if (stat_udp == SOCK_BUSY) cnt_udp++; else cnt_udp = stat_udp;
		    						}
		    					} else {
		    						if (stat_udp > 0) cnt_udp++; else cnt_udp = stat_udp;
		    					}
	#ifdef SET_OLED_SPI
		    					sprintf(line, "   udp: %d", (int)cnt_udp);
		    					spi_ssd1306_clear_line(8);
		    					spi_ssd1306_text_xy(line, 1, 8);
	#endif
		    					if (stat_udp <= 0) {
		    						if (cnt_udp > 0) cnt_udp--;
		    					}
		    				}
		    			}
		    		}
		    	}
#endif
		    break;
			case msg_rxDone:
				Report(NULL, 0, stx);
				ackParse();
			break;
			case msg_rst:
				HAL_Delay(800);
				NVIC_SystemReset();
			break;
			case msg_out:
				tmr_out = getTimer(cikl_out);
				uint8_t with_dt = 0;
				if (outMode == textMode) with_dt = 1;
				Report(NULL, with_dt, "%s\r\n", printOut(tmp));
			break;
#ifndef SET_COMPAS_BLOCK
			case msg_i2c:
				switch (siCmd) {
					case msg_startCompas:
						if (chkTimer(next)) {
							//
							//STROB_DOWN();
							//
							msBegin = HAL_GetTick();
							next = 0;
							procCompas();
							//
						} else {
							putMsg(msg_i2c);
						}
					break;
					case msg_getCompas:
						//
						if (CompAddVal(COPMAS_CalcAngle()) == MAX_COMP_BUF) {//в окне накоплено MAX_VCC_BUF выборок -> фильтрация !
							float sum = 0;
							for (int8_t j = 0; j < MAX_COMP_BUF; j++) sum += CompBuf[j];
							sum /= MAX_COMP_BUF;
							compData.angleHMC = sum;
						}
						//
						siCmd = msg_empty;
						next = 0;
						putMsg(msg_nextSens);
						//
					break;
					case msg_rdyTest:
						//
						i2c_test_ready_bmx280(&info_bmp280);
						//
					break;
					case msg_rdyStat:
						bStat = i2c_getStat_bmx280();
						if (!bStat) {
							errStatCnt++;
							if (errStatCnt < 10) {
								siCmd = msg_rdyTest;
								putMsg(msg_i2c);
							} else {
								devError |= devI2C1;
								errStatCnt = 0;
							}
							break;
						}
						//
						if (i2c_read_data_bmx280(data_rdx, d_size) != HAL_OK) devError |= devI2C1;
						//
					break;
					case msg_readBMX:
						//
						if (!bmxCalibr) bmx280_readCalibr1Data(reg_id);//goto read calibration data part 1
						else {
							bmx280_CalcAll(&sensors, reg_id);
#ifdef SET_MPU
							if (mpuPresent) {
								//
	#ifdef SET_MPU_INTERRUPT
								mpuEnableInterrupts();
	#else
								siCmd = msg_mpuAllRead;
								putMsg(msg_i2c);
	#endif
							} else putMsg(msg_endNext);
#else
							putMsg(msg_endNext);
#endif
						}
						//
					break;
					case msg_calibr1Done:
						//
						bmx280_calcCalibr1Data(reg_id);
						if (reg_id == BME280_SENSOR) {
							bmx280_readCalibr2Data(reg_id);
						} else {
							bmx280_CalcAll(&sensors, reg_id);
#ifdef SET_MPU
							if (mpuPresent) {
								siCmd = msg_mpuAllRead;
								putMsg(msg_i2c);
							} else putMsg(msg_endNext);
#else
							putMsg(msg_endNext);
#endif
						}
						//
					break;
					case msg_calibr2Done:
						//
						if (reg_id == BME280_SENSOR) bmx280_calcCalibr2Data();
						bmx280_CalcAll(&sensors, reg_id);
						bmxCalibr = true;
#ifdef SET_MPU
						if (mpuPresent) {
							//
	#ifdef SET_MPU_INTERRUPT
							mpuEnableInterrupts();
	#else
							siCmd = msg_mpuAllRead;
							putMsg(msg_i2c);
	#endif
						} else putMsg(msg_endNext);
#else
						putMsg(msg_endNext);
#endif
						//
					break;
#ifdef SET_MPU
					case msg_mpuAllRead:
						//
	#ifndef SET_MPU_INTERRUPT
						mpuStatus = mpuReadStatus();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	#endif
						//
						mpuAllRead();//start read all data from adr 0x3b....(14 bytes)
						//
					break;
					case msg_mpuAllReady:
						//
						mpuConvData();
						//STROB_UP();
						putMsg(msg_endNext);
						//
					break;
#endif
				}
			break;
#else
			case msg_startCompas:
			{
				if (chkTimer(next)) {
					//STROB_DOWN();
					//
					msBegin = HAL_GetTick();
					next = 0;
					procCompas();

					//
					if (CompAddVal(COPMAS_CalcAngle()) == MAX_COMP_BUF) {//в окне накоплено MAX_VCC_BUF выборок -> фильтрация !
						float sum = 0;
						for (int8_t j = 0; j < MAX_COMP_BUF; j++) sum += CompBuf[j];
						sum /= MAX_COMP_BUF;
						compData.angleHMC = sum;
					}
					//
					putMsg(msg_nextSens);
					//
					//STROB_UP();
				} else putMsg(msg_startCompas);
			}
			break;
#endif
			case msg_nextSens:
				if (chkTimer(next)) {
					//STROB_DOWN();
					//
#ifdef SET_BMx280
					//
					if (i2c_test_bmx280(&info_bmp280, reg_id) != HAL_OK) devError |= devI2C1;
					//next = getTimer(_1ms);
	#ifdef SET_COMPAS_BLOCK
					bStat = i2c_getStat_bmx280();
					if (!bStat) {
						next = getTimer(_1ms);
						putMsg(msg_nextSens);
						break;
					}
					//
					if (i2c_read_bmx280(BMP280_REG_PRESSURE, data_rdx, d_size) == HAL_OK) {
						//
						if (!bmxCalibr) bmxCalibr = bmx280_readCalibrationData(reg_id);
						//
						bmx280_CalcAll(&sensors, reg_id);
						//
					} else devError |= devI2C1;
					putMsg(msg_endNext);
	#endif
#endif
					//STROB_UP();
				} else putMsg(msg_nextSens);
			break;
			case msg_endNext:
				//
				//
				//
				msCnt = HAL_GetTick() - msBegin;
				if (msCnt < msPeriod) {
					msCnt = msPeriod - msCnt;
				} else {
					msCnt -= msPeriod;
				}
				if (!msCnt) msCnt++;
				next = getTimer(msCnt);
				//
#ifdef SET_COMPAS_BLOCK
				evt = msg_startCompas;
				siCmd = msg_empty;
#else
				evt = msg_i2c;
				siCmd = msg_startCompas;
#endif
				putMsg(evt);
				//
				//STROB_UP();
			break;
		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
	// for 50 MHz clock : for OLED/IPS display
		// _2 - 25MHz
		// _4 - 12.5MHz
		// hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; - 6.25 Mbits/s
		// _16 - 3.125MHz
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */
	// for 100 MHz clock : for W5500 internet port
	// _2 - 50MHz
	// _4 - 25MHz
	// _8 - 12.5MHz
	// hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; - 6.25 Mbits/s
	// _32 - 3.125MHz
  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  	  // 1ms period :
  	  // for 100MHz clock
  	  	  // htim2.Init.Prescaler = 446;
  	  	  // htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  	  	  // htim2.Init.Period = 223;
  	  	  //
  	  //  for 96MHz clock
  	  	  // htim2.Init.Prescaler = 437;//446;
    	  // htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    	  // htim2.Init.Period = 218;//223;
  	  	  //
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(irLED_GPIO_Port, irLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin|OLED_RST_Pin|OLED_CS_Pin|NET_CS_Pin
                          |NET_RST_Pin|STROB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, tLED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : irLED_Pin */
  GPIO_InitStruct.Pin = irLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(irLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : iKEY_Pin NET_EXTI4_Pin */
  GPIO_InitStruct.Pin = iKEY_Pin|NET_EXTI4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRED_Pin */
  GPIO_InitStruct.Pin = IRED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_CS_Pin NET_CS_Pin
                           NET_RST_Pin ERR_LED_Pin STROB_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_CS_Pin|NET_CS_Pin
                          |NET_RST_Pin|ERR_LED_Pin|STROB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : tLED_Pin */
  GPIO_InitStruct.Pin = tLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(tLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
	ON_ERR_LED()
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
