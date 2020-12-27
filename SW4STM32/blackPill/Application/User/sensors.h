/*
 * sensors.h
 *
 *  Created on: Dec 2, 2020
 *      Author: alarm
 */

#include "hdr.h"
#include "main.h"

#ifndef APPLICATION_USER_SENSORS_H_
#define APPLICATION_USER_SENSORS_H_

#pragma pack(push,1)
    typedef struct {
	    float bmx_temp;// DegC
	    float bmx_pres;// mmHg
	    float bmx_humi;// RH
	} result_t;
#pragma pack(pop)

#pragma pack(push,1)
	typedef struct {
		uint8_t conf;
		uint8_t mode;
		uint8_t stat;
	} info_bmp280_t;
#pragma pack(pop)

#pragma pack(push,1)
	typedef struct {
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t stat;
		uint16_t temp;
	} xyz_t;
#pragma pack(pop)


	#define MAG_BUF_SIZE 9

	#ifndef M_PI
		#define M_PI 3.14159265
	#endif

	#define COPMAS_ADDRESS  0x0D//0x1E // this device only has one address
	#define COPMAS_STAT_REG 6
	#define COPMAS_CTRL_REG 9 //9<-0x1D, 0x0a<-0, 0x0b<-1

	#define Mode_Standby    0b00000000
	#define Mode_Continuous 0b00000001

	#define ODR_10Hz        0b00000000
	#define ODR_50Hz        0b00000100
	#define ODR_100Hz       0b00001000
	#define ODR_200Hz       0b00001100

	#define RNG_2G          0b00000000
	#define RNG_8G          0b00010000

	#define OSR_512         0b00000000
	#define OSR_256         0b01000000
	#define OSR_128         0b10000000
	#define OSR_64          0b11000000


	result_t sensors;
	uint8_t	magBuf[MAG_BUF_SIZE];
	xyz_t *xyz;
	uint8_t cStat;


	HAL_StatusTypeDef COPMAS_Init();
	HAL_StatusTypeDef COPMAS_Reset();
	uint8_t COPMAS_ReadStat();
	HAL_StatusTypeDef COPMAS_GetAngle();
	uint16_t COPMAS_CalcAngle();



#ifdef SET_BMx280

    #define bmp280s "BMP280"
	#define bme280s "BME280"
	#define bmx280s "???"

	/*  BMP280/BME280 registers  */
	#define BMP280_ADDR            0x76 //!< slave address for BMP280 sensor
	#define BMP280_SENSOR          0x58
	#define BME280_SENSOR          0x60
	#define BMP280_REG_TEMP_XLSB   0xFC // bits: 7-4
	#define BMP280_REG_TEMP_LSB    0xFB
	#define BMP280_REG_TEMP_MSB    0xFA
	#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
	#define BMP280_REG_PRESS_XLSB  0xF9 // bits: 7-4
	#define BMP280_REG_PRESS_LSB   0xF8
	#define BMP280_REG_PRESS_MSB   0xF7
	#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
	#define BMP280_REG_CONFIG      0xF5 // bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en
	#define BMP280_REG_CTRL        0xF4 // bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode
	#define BMP280_REG_STATUS      0xF3 // bits: 3 measuring; 0 im_update
	#define BME280_REG_CTRL_HUM    0xF2 // bits: 2-0 osrs_h;
	#define BMP280_REG_RESET       0xE0
	#define BMP280_REG_ID          0xD0
	#define BMP280_REG_CALIB       0x88
	#define BMP280_REG_CALIB_EXT   0xE1
	#define BMP280_NORMAL_MODE     0x3
	#define BMP280_FORCED1_MODE    0x1
	#define BMP280_FORCED2_MODE    0x2
	#define BMP280_OVERSAMP_1      1
	#define BMP280_OVERSAMP_2      2
	#define BMP280_OVERSAMP_4      3
	#define BMP280_OVERSAMP_8      4
	#define BMP280_OVERSAMP_16     5
	#define BMP280_OSRS_T          (BMP280_OVERSAMP_1 << 5)//0x20
	#define BMP280_OSRS_P          (BMP280_OVERSAMP_1 << 2)//0x04
	#define BMP280_CONF_T_SB       0//0xC0//0x40
	#define BMP280_CONF_FILTER     0x00
	#define BMP280_CONF_SPI3W      0x00
	#define BME280_OSRS_H          BMP280_OVERSAMP_1//0x01
	#define BMP280_RESET_VALUE     0xB6

//	#define DATA_LENGTH            8//256        //!<Data buffer length for test buffer

	typedef struct bmp280_calib_t {
		uint16_t dig_T1;
		int16_t  dig_T2;
		int16_t  dig_T3;
		uint16_t dig_P1;
		int16_t  dig_P2;
		int16_t  dig_P3;
		int16_t  dig_P4;
		int16_t  dig_P5;
		int16_t  dig_P6;
		int16_t  dig_P7;
		int16_t  dig_P8;
		int16_t  dig_P9;
		int8_t   dig_H1;
		int16_t  dig_H2;
		int8_t   dig_H3;
		int16_t  dig_H4;
		int16_t  dig_H5;
		int8_t   dig_H6;
	} bmx280_calib_t;

	info_bmp280_t info_bmp280;
	uint8_t data_bmp280[25];

	HAL_StatusTypeDef i2c_test_bmx280(info_bmp280_t *info_bmp280, uint8_t chip_id);
	HAL_StatusTypeDef i2c_reset_bmx280(uint8_t *chip_id);
	bool i2c_getStat_bmx280();
#ifdef SET_COMPAS_BLOCK
	HAL_StatusTypeDef i2c_read_bmx280(uint8_t reg, uint8_t *data_rd, size_t size);
	bool bmx280_readCalibrationData(uint8_t chip_id);
#else
	HAL_StatusTypeDef i2c_read_data_bmx280(uint8_t *data_rd, size_t size);
	HAL_StatusTypeDef i2c_test_ready_bmx280(info_bmp280_t *info_bmp280);
	HAL_StatusTypeDef bmx280_readCalibr1Data(uint8_t chip_id);
	void bmx280_calcCalibr1Data(uint8_t chip_id);
	HAL_StatusTypeDef bmx280_readCalibr2Data();
	void bmx280_calcCalibr2Data();
#endif
	void bmx280_CalcAll(result_t *ssen, int32_t chip_id);//, int32_t tp, uint32_t pp, uint32_t hh);

#endif


#endif /* APPLICATION_USER_SENSORS_H_ */
