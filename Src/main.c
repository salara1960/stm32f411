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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hdr.h"
#include "stm32f4xx_hal_adc.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_TMP_SIZE  256
#define MAX_FIFO_SIZE  64
#define MAX_UART_BUF  512
#define MAX_VCC_BUF     4
#define MAX_COMP_BUF    4


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
#define _130ms (_1ms * 130)
#define _140ms (_1ms * 140)
#define _150ms (_1ms * 150)
#define _200ms (_1ms * 200)
#define _250ms (_1ms * 250)
#define _300ms (_1ms * 300)
#define _400ms (_1ms * 400)
#define _500ms (_1ms * 500)
#define _1s (_1ms * 1000)
#define _2s (_1s * 2)//2000
#define _3s (_1s * 3)//3000
#define _4s (_1s * 4)//4000
#define _5s (_1s * 5)//5000

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

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

static evt_t evt_fifo[MAX_FIFO_SIZE] = {msg_empty};
uint8_t rd_evt_adr = 0;
uint8_t wr_evt_adr = 0;
uint8_t wr_evt_err = 0;
uint8_t cnt_evt = 0;
uint8_t max_evt = 0;

volatile uint32_t tik = 0;
//volatile uint8_t mktik = 0;
volatile time_t epoch = 1607791830;
uint8_t tZone = 0;//GMT
volatile uint8_t setDate = 0;
char rxData[MAX_TMP_SIZE] = {0};
char rxByte = '\0';
uint8_t rx_uk = 0;
uint16_t rxBytes = 0;
char stx[MAX_TMP_SIZE] = {0};
char tmp[MAX_TMP_SIZE] = {0};
volatile uint32_t txDoneCnt = 0;
volatile uint8_t txDoneFlag = 1;
volatile uint8_t ledValue = 0;
uint32_t next = 0;
uint32_t seconds = 0;
uint32_t msBegin = 0, msCnt = 0;
//
int siCmd = 0;
uint32_t tx_icnt = 0, rx_icnt = 0;

#ifdef SET_OLED_SPI
	SPI_HandleTypeDef *portOLED = NULL;
	char line[MAX_TMP_SIZE] = {0};
	uint32_t spi_cnt;
#endif

#ifdef SET_BMx280
	I2C_HandleTypeDef *portBMP = NULL;
	uint8_t reg_id = 0;
	uint8_t data_rdx[DATA_LENGTH] = {0};
	size_t d_size = 6;
#endif

compas_data_t compData = {0, 0.0};
I2C_HandleTypeDef *portHMC = NULL;


ADC_HandleTypeDef *portADC = NULL;
volatile uint16_t adcValue = 0;
volatile float VccF = 0.0;
uint32_t iadc_cnt = 0;
uint8_t startADC = 1;
uint16_t VccBuf[MAX_VCC_BUF] = {0};//буфер для накопления данных перед усреднением
uint8_t VccValCounter = 0;

uint16_t CompBuf[MAX_COMP_BUF] = {0};//буфер для накопления данных перед усреднением
uint8_t CompValCounter = 0;

volatile uint8_t devError = 0;

#ifdef MIC_PRESENT
	volatile uint8_t enEXT3 = 1;
	uint32_t tmrEXT3 = 0;
#endif

#ifdef SET_CDC_PORT
	char rxCDC[2048] = {0};
	uint32_t lenCDC = 0;
#endif

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
/* USER CODE BEGIN PFP */

//void set_Date(time_t epoch);
//uint32_t get_Date();
//int sec_to_str_time(char *st);
//void Report(const char *tag, unsigned char addTime, const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
uint32_t get5ms(uint32_t t)
{
	return (tik + t);
}
int chk5ms(uint32_t t)
{
	return (tik >= t ? 1 : 0);
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
//		putMsg(msg_empty);
		//mktik = (mktik + 1) & 1;
		//if (!mktik) {
			tik++;
			if (!(tik % (_1ms))) putMsg(msg_1ms);
		//}
	}
  /* USER CODE END Callback 1 */
}
//----------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1) {
		rxData[rx_uk & 0x7f] = rxByte;
		if (rxByte == '\n') {//end of line
			rxData[(rx_uk+1) & 0x7f] = '\0';
			strncpy(stx, rxData, rx_uk + 1);
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
	if (huart->Instance == USART1) {
		txDoneFlag = 1;
		txDoneCnt++;
	}
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
#ifdef MIC_PRESENT
	else if (GPIO_Pin == MIC_DIG_Pin) {
		if (enEXT3) {
			enEXT3 = 0;
			HAL_GPIO_TogglePin(bLED_GPIO_Port, bLED_Pin);
			putMsg(msg_out);
			tmrEXT3 = get5ms(_500ms);
		}
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
#ifdef SET_OLED_SPI
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI4) {
		if (withDMA) {
			CS_OLED_DESELECT();
			spiRdy = 1;
		}
		spi_cnt++;
	}
}
//-----------------------------------------------------------------------------
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI4) devError |= devSPI;
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

	if (!txDoneFlag) return;

#ifdef SET_CDC_PORT
	if (devError & devCDC) devError &= ~devCDC;
#endif

	char *buff = (char *)calloc(1, len);//pvPortMalloc(len);//vPortFree(buff);
	if (buff) {
		txDoneFlag = 0;

		if (addTime) dl = sec_to_str_time(buff);
		if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);

		va_start(args, fmt);
		vsnprintf(buff + dl, len - dl, fmt, args);
		uint16_t slen = strlen(buff);


		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buff, slen);
		/*while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
				if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX) break;
				HAL_Delay(1);
		}*/

		va_end(args);
#ifdef SET_CDC_PORT
		if (CDC_Transmit_FS((uint8_t *)buff, slen) != USBD_OK) devError |= devCDC;
#endif

		free(buff);//vPortFree(buff);
	}
}
//------------------------------------------------------------------------------------------
void set_Date(time_t ep)
{
struct tm ts;

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
uint32_t get_Date()
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
uint8_t CompAddVal(uint16_t value)
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
void ledOnOff()
{
	tLED_ONOFF();
	bLED_ONOFF();
}
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
/*void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (hpcd) {
		uint8_t cha = 0x30;
		if (hpcd->pData) {
			uint8_t *uk = (uint8_t *)hpcd->pData;
			cha = *uk;
		}
		char buf[32];
		uint8_t col = ssd1306_calcx(sprintf(buf, "val=0x%02X", cha));
		ssd1306_text_xy(buf, col, 6);
	}
}*/
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
#ifdef SET_CDC_PORT
  MX_USB_DEVICE_Init();
#endif
  /* USER CODE BEGIN 2 */

  	// start timer1 + interrupt
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
    //"start" rx_interrupt
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);


    ////HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
    //uint8_t ep_addr = 0;
    //HAL_PCD_EP_Receive(&hpcd, ep_addr, (uint8_t *)&v_uRxByte, 1);


	set_Date(epoch);

	char *uk = NULL, *uke = NULL;
	evt_t evt;
	uint32_t schMS = 0;

	ON_ERR_LED();//!!!!!!!!!!!!!!!!!!!!!
	STROB_UP();//!!!!!!!!!!!!!!!!!!!!!


#ifdef SET_OLED_SPI
    portOLED = &hspi4;
    spi_ssd1306_Reset();
    spi_ssd1306_on(1);//screen ON
    spi_ssd1306_init();//screen INIT
    spi_ssd1306_pattern();//set any params for screen
    //spi_ssd1306_invert();
    spi_ssd1306_clear();//clear screen

    sec_to_str_time(line);
    spi_ssd1306_text_xy(line, 2, 1);
#endif


#ifdef SET_BMx280
    portBMP = &hi2c1;
    bool bStat = false;
    bool bmxCalibr = false;
    char bmxName[16] = {0};
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
#endif
    putMsg(evt);
    next = get5ms(_1ms);

    portADC = &hadc1;
    if (startADC) HAL_ADC_Start_IT(portADC);

#ifdef SET_CDC_PORT
    CDC_Recv(rxCDC, &lenCDC);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)  {

		switch (getMsg()) {
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
		    case msg_1ms:
//		    	HAL_GPIO_TogglePin(STROB_GPIO_Port, STROB_Pin);
		    	schMS++;
		    	if (!(schMS % (_100ms))) {// 100ms
		    		if (startADC) HAL_ADC_Start_IT(portADC);
		    		if (devError) ON_ERR_LED()
		    		         else OFF_ERR_LED()
#ifdef MIC_PRESENT
					if (tmrEXT3) {
						if (chk5ms(tmrEXT3)) {
							enEXT3 = 1;
							tmrEXT3 = 0;
						}
					}
#endif
		    	}
#ifdef SET_OLED_SPI
		    	if (!(schMS % (_250ms))) {// 250ms

		    		sprintf(line, "  temp:%.2f\n  Azimut:%u", compData.tempHMC, compData.angleHMC);
		    		spi_ssd1306_clear_line(7);
		    		spi_ssd1306_clear_line(8);
		    		spi_ssd1306_text_xy(line, 1, 7);
		    	}
#endif
		    	if (!(schMS % (_1s))) {// 1000ms
//					HAL_GPIO_TogglePin(tLED_GPIO_Port, tLED_Pin);
		    		//HAL_GPIO_TogglePin(bLED_GPIO_Port, bLED_Pin);
		    		ledOnOff();
#ifdef SET_OLED_SPI
					sec_to_str_time(line);
					sprintf(line+strlen(line), "\n devError:0x%02X\n", devError);
					sprintf(line+strlen(line), "  VCC:%.3fV\n", VccF);
		#ifdef SET_BMx280
					sprintf(line+strlen(line), "  mmHg:%.2f\n  DegC:%.2f", sensors.bmx_pres, sensors.bmx_temp);
					if (reg_id == BME280_SENSOR) sprintf(line+strlen(line), "\n  %%rH:%.2f\n", sensors.bmx_humi);
		#endif
					//spi_ssd1306_clear();
					spi_ssd1306_text_xy(line, 2, 1);
#endif
					if (!(++seconds % (5))) {
						putMsg(msg_out);
#ifdef SET_CDC_PORT
						if (lenCDC) {
							Report(NULL, 1, "%s", (char *)rxCDC);
							lenCDC = 0;
							CDC_Recv(rxCDC, &lenCDC);
						}
#endif
					}
		    	}
		    break;
			case msg_rxDone:
				Report(NULL, 0, "%s", stx);
				if ((uk = strstr(stx, "epoch=")) != NULL) {
					uk += 6;
					if (strlen(uk) >= 10) {
						uke = strchr(uk, ':');
						if (uke) {
							tZone = atoi(uke + 1);
							*uke = '\0';
						} else tZone = 0;
						uint32_t cep = atol(uk);
						if (cep > 0) {
							set_Date((time_t)cep);
						}
					}
				} else if ((uk = strstr(stx, "rst")) != NULL) {
					putMsg(msg_rst);
				}
			break;
			case msg_rst:
				HAL_Delay(1000);
				NVIC_SystemReset();
			break;
			case msg_out:
				sprintf(tmp, " | tik=%lu fifo:%u/%u", tik, cnt_evt, max_evt);
				if (devError) sprintf(tmp+strlen(tmp), " devError:0x%02X", devError);
				sprintf(tmp+strlen(tmp), " | Vcc=%.3fV", VccF);
#ifdef SET_BMx280
				sprintf(tmp+strlen(tmp)," | %s: ready=%d mmHg=%.2f degC=%.2f", bmxName, bStat, sensors.bmx_pres, sensors.bmx_temp);
				if (reg_id == BME280_SENSOR) sprintf(tmp+strlen(tmp), " %%rH=%.2f", sensors.bmx_humi);
#endif
				//compas_stat_t *cStat = (compas_stat_t *)confRegHmc;
				sprintf(tmp+strlen(tmp), " | QMC5883L: ready=%u azimut=%u temp=%.2f",
						cStat,
						compData.angleHMC,
						compData.tempHMC);//, xyz->x, xyz->y, xyz->z);
				Report(NULL, 1, "%s\r\n", tmp);
			break;
#ifndef SET_COMPAS_BLOCK
			case msg_i2c:
				switch (siCmd) {
					case msg_startCompas:
						if (chk5ms(next)) {
							STROB_DOWN();
							//
							msBegin = HAL_GetTick();
							next = 0;
							procCompas();
							//
							//STROB_UP();
						} else {
							putMsg(msg_i2c);
						}
					break;
					case msg_getCompas:
					{
						//STROB_DOWN();
						//
						if (CompAddVal(COPMAS_CalcAngle()) == MAX_COMP_BUF) {//в окне накоплено MAX_VCC_BUF выборок -> фильтрация !
							uint16_t sum = 0;
							for (int8_t j = 0; j < MAX_COMP_BUF; j++) sum += CompBuf[j];
							sum /= MAX_COMP_BUF;
							compData.angleHMC = sum;
						}
						//
						siCmd = msg_empty;
						next = get5ms(0);
						putMsg(msg_nextSens);
						//
						//STROB_UP();
					}
					break;
					case msg_rdyTest:
						//STROB_DOWN();
						//
						i2c_test_ready_bmx280(&info_bmp280);
						//
						//STROB_UP();
					break;
					case msg_rdyStat:
						//STROB_DOWN();
						bStat = i2c_getStat_bmx280();
						if (!bStat) {
							//next = get5ms(_1ms);
							siCmd = msg_rdyTest;
							putMsg(msg_i2c);
							break;
						}
						//
						if (i2c_read_data_bmx280(data_rdx, d_size) != HAL_OK) devError |= devI2C1;
						//
						//STROB_UP();
					break;
					case msg_readBMX:
						//STROB_DOWN();
						//
						if (!bmxCalibr) bmx280_readCalibr1Data(reg_id);//goto read calibration data part 1
						else {
							bmx280_CalcAll(&sensors, reg_id);
							putMsg(msg_endNext);
						}
						//
						//STROB_UP();
					break;
					case msg_calibr1Done:
						//STROB_DOWN();
						//
						bmx280_calcCalibr1Data(reg_id);
						if (reg_id == BME280_SENSOR) {
							bmx280_readCalibr2Data(reg_id);
						} else {
							bmx280_CalcAll(&sensors, reg_id);
							putMsg(msg_endNext);
						}
						//
						//STROB_UP();
					break;
					case msg_calibr2Done:
						//STROB_DOWN();
						//
						if (reg_id == BME280_SENSOR) bmx280_calcCalibr2Data();
						bmx280_CalcAll(&sensors, reg_id);
						bmxCalibr = true;
						putMsg(msg_endNext);
						//
						//STROB_UP();
					break;
				}
			break;
#else
			case msg_startCompas:
			{
				if (chk5ms(next)) {
					//STROB_DOWN();
					//
					msBegin = HAL_GetTick();
					next = 0;
					procCompas();

					//
					if (CompAddVal(COPMAS_CalcAngle()) == MAX_COMP_BUF) {//в окне накоплено MAX_VCC_BUF выборок -> фильтрация !
						uint16_t sum = 0;
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
				if (chk5ms(next)) {
					//STROB_DOWN();
					//
#ifdef SET_BMx280
					//
					if (i2c_test_bmx280(&info_bmp280, reg_id) != HAL_OK) devError |= devI2C1;
					next = get5ms(_1ms);
	#ifdef SET_COMPAS_BLOCK
					bStat = i2c_getStat_bmx280();
					if (!bStat) {
						next = get5ms(_1ms);
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
				msCnt = HAL_GetTick() - msBegin;
				if (msCnt < _250ms) {
					msCnt = _250ms - msCnt;
				} else {
					msCnt -= _250ms;
				}
				if (!msCnt) msCnt++;
				next = get5ms(msCnt);
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
				STROB_UP();
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  	  // 1ms period :
  	  // htim2.Init.Prescaler = 437;//446;
  	  // htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  	  // htim2.Init.Period = 218;//223;
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 437;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 218;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
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
  HAL_GPIO_WritePin(bLED_GPIO_Port, bLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin|OLED_RST_Pin|OLED_CS_Pin|STROB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, tLED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : bLED_Pin */
  GPIO_InitStruct.Pin = bLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(bLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : iKEY_Pin */
  GPIO_InitStruct.Pin = iKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(iKEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_DIG_Pin */
  GPIO_InitStruct.Pin = MIC_DIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MIC_DIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_CS_Pin ERR_LED_Pin 
                           STROB_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_CS_Pin|ERR_LED_Pin 
                          |STROB_Pin;
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
