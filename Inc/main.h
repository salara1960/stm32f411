/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f4xx_hal_adc.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>


#include "hdr.h"
#include "sensors.h"
#include "ssd1306.h"
#include "jfes.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define DATA_LENGTH    8//256        //!<Data buffer length for test buffer


typedef enum {
	msg_empty = 0,
	msg_10ms,//msg_1ms,
	msg_sec,
	msg_rxDone,
	msg_rst,
	msg_out,
	msg_i2c,
	msg_adcReady,
	msg_startCompas,
	msg_getCompas,
	msg_nextSens,
	msg_rdyTest,
	msg_rdyStat,
	msg_endNext,
	msg_readBMX,
	msg_calibr1Done,
	msg_calibr2Done,
	msg_iMPU,
	msg_mpuAllRead,
	msg_mpuAllReady,
	msg_mpuReadInterruptsStatus,
	msg_shiftEvent,
	msg_none
} evt_t;

enum {
	devOK = 0,
	devI2C1 = 1,
	devSPI = 2,
	devUART = 4,
	devADC = 8,
	devTmr2 = 0x10,
	devFifo = 0x20,
	devMem = 0x40,
	devJfes = 0x80
};

enum {
	textMode = 0,
	jfesMode,
	jsonMode
};

#ifndef bool
	#define true 1
	#define false 0
#endif

#pragma pack(push,1)
typedef struct {
	float angleHMC;
	float tempHMC;
} compas_data_t;
#pragma pack(pop)

volatile uint32_t tik;
int siCmd;
uint32_t tx_icnt, rx_icnt;
volatile uint8_t devError;

I2C_HandleTypeDef *portHMC;
compas_data_t compData;

#ifdef SET_OLED_SPI
	SPI_HandleTypeDef *portOLED;
#endif
#ifdef SET_BMx280
	I2C_HandleTypeDef *portBMP;
	uint8_t data_rdx[DATA_LENGTH];
#endif

#ifdef SET_MPU
	I2C_HandleTypeDef *portMPU;
#endif

//extern evt_t evt_fifo[MAX_FIFO_SIZE];
//extern uint8_t rd_evt_adr, wr_evt_adr;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/*
enum {
	msg_empty,
	msg_10ms,
	msg_txDone
} evt_t;
*/
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define iKEY_Pin GPIO_PIN_0
#define iKEY_GPIO_Port GPIOA
#define iKEY_EXTI_IRQn EXTI0_IRQn
#define OLED_MOSI_Pin GPIO_PIN_1
#define OLED_MOSI_GPIO_Port GPIOA
#define iADC_Pin GPIO_PIN_2
#define iADC_GPIO_Port GPIOA
#define MIC_DIG_Pin GPIO_PIN_3
#define MIC_DIG_GPIO_Port GPIOA
#define MIC_DIG_EXTI_IRQn EXTI3_IRQn
#define iEXTI4_Pin GPIO_PIN_4
#define iEXTI4_GPIO_Port GPIOA
#define iEXTI4_EXTI_IRQn EXTI4_IRQn
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_1
#define OLED_RST_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_2
#define OLED_CS_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_13
#define OLED_SCK_GPIO_Port GPIOB
#define tLED_Pin GPIO_PIN_5
#define tLED_GPIO_Port GPIOB
#define ERR_LED_Pin GPIO_PIN_8
#define ERR_LED_GPIO_Port GPIOB
#define STROB_Pin GPIO_PIN_9
#define STROB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



#define STROB_UP()   HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_SET);
#define STROB_DOWN() HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_RESET);

#define ON_ERR_LED() HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_SET);
#define OFF_ERR_LED() HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_RESET);



#ifdef SET_OLED_SPI
	#define CS_OLED_SELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
	#define CS_OLED_DESELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
