/*
 * sensors.c
 *
 *  Created on: Dec 2, 2020
 *      Author: alarm
 */

#include "hdr.h"
#include "sensors.h"


result_t sensors = {0.0, 0.0, 0.0};
const uint32_t min_wait_ms = 150;
const uint32_t max_wait_ms = 500;
uint8_t hmc_ini[] = {Mode_Continuous | ODR_100Hz | RNG_8G | OSR_512, 0, 1};

uint8_t	magBuf[MAG_BUF_SIZE] = {0};
uint8_t cStat = 0;
xyz_t *xyz = NULL;

//******************************************************************************************


//-----------------------------------------------------------------------------
HAL_StatusTypeDef COPMAS_Init()
{
	return HAL_I2C_Mem_Write(portHMC, COPMAS_ADDRESS << 1, COPMAS_CTRL_REG, 1, hmc_ini, 3, min_wait_ms);
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef COPMAS_Reset()
{
	uint8_t dt[] = {0x0A, 0x80};
	return HAL_I2C_Master_Transmit(portHMC, COPMAS_ADDRESS << 1, dt, 2, min_wait_ms);
}
//-----------------------------------------------------------------------------
uint8_t COPMAS_ReadStat()
{
uint8_t stat = 0;

	if (HAL_I2C_Mem_Read(portHMC, COPMAS_ADDRESS << 1, COPMAS_STAT_REG, 1, &stat, 1, min_wait_ms) != HAL_OK) devError |= devI2C1;
	return stat;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef COPMAS_GetAngle()
{

	//return HAL_I2C_Mem_Read_DMA(portHMC, COPMAS_ADDRESS << 1, 0, 1, magBuf, 6);

	uint8_t adr = 0;
	HAL_StatusTypeDef rt = HAL_I2C_Master_Transmit(portHMC, COPMAS_ADDRESS << 1, &adr, 1, min_wait_ms);
	if (rt == HAL_OK) {
#ifdef SET_COMPAS_BLOCK
		rt |= HAL_I2C_Master_Receive(portHMC, COPMAS_ADDRESS << 1, magBuf, MAG_BUF_SIZE, min_wait_ms);
#else
		siCmd = msg_getCompas;
		rt |= HAL_I2C_Master_Receive_DMA(portHMC, COPMAS_ADDRESS << 1, magBuf, MAG_BUF_SIZE);
#endif
	} else rt |= devI2C1;

	devError = rt;

	return rt;
}
//-----------------------------------------------------------------------------
float COPMAS_CalcAngle()
{
double azimut, x, y, z, con = 2 * M_PI;

	xyz = (xyz_t *)&magBuf[0];

	uint16_t tempHMCraw = xyz->temp & 0x3fff;
	compData.tempHMC = (float)(tempHMCraw) / 520.0;

	x = xyz->x * 0.92;
	y = xyz->y * 0.92;
	z = xyz->z * 0.92;

	//compData.x = (int)x;
	//compData.y = (int)y;
	//compData.z = (int)z;

	azimut = atan2(y, x);
	if (azimut < 0) azimut += con;
	else
	if (azimut > con) azimut -= con;

	float VX = atan2(x, z) / 0.0174532925;
	if (VX < 0) VX += 360;
	VX = 360 - VX - 180;
	if (VX < 0) VX += 360;
	compData.x = (int)VX;
	//
	float VY = atan2(y, z) / 0.0174532925;
	if (VY < 0) VY += 360;
	VY = 360 - VY - 180;
	if (VY < 0) VY += 360;
	compData.y = (int)VY;

	return (float)(azimut * (180 / M_PI));
}
//-----------------------------------------------------------------------------


//******************************************************************************************

#ifdef SET_BMx280

	info_bmp280_t info_bmp280 = {0};
    uint8_t data_bmp280[25] = {0};

	static bmx280_calib_t calib;

	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef i2c_reset_bmx280(uint8_t *chip_id)
	{
	HAL_StatusTypeDef rt = HAL_ERROR;
	uint8_t dat[] = {BMP280_REG_RESET, BMP280_RESET_VALUE};

		if (chip_id) {
			rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, dat, 2, min_wait_ms);
			if (rt == HAL_OK) {
				rt |= HAL_I2C_Mem_Read(portBMP, BMP280_ADDR << 1, BMP280_REG_ID, 1, chip_id, 1, min_wait_ms);
			}
		}

	    return rt;
	}
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef i2c_test_bmx280(info_bmp280_t *info_bmp280, uint8_t chip_id)
	{
	HAL_StatusTypeDef rt = HAL_ERROR;
#ifdef SET_COMPAS_BLOCK
	uint8_t dati[] = {BME280_OSRS_H,//for BME280_SENSOR only
					  //for all bmx sensors
					  BMP280_OSRS_T | BMP280_OSRS_P | BMP280_FORCED1_MODE,
					  BMP280_CONF_T_SB | BMP280_CONF_FILTER | BMP280_CONF_SPI3W};

	if (chip_id == BME280_SENSOR) {
		rt = HAL_I2C_Mem_Write(portBMP, BMP280_ADDR << 1, BME280_REG_CTRL_HUM, 1, &dati[0], 1, min_wait_ms);
	} else rt = HAL_OK;
	if (rt == HAL_OK) {
		rt |= HAL_I2C_Mem_Write(portBMP, BMP280_ADDR << 1, BMP280_REG_CTRL, 1, &dati[1], 2, min_wait_ms);
		if (rt == HAL_OK) {
			if (rt == HAL_OK) rt |= HAL_I2C_Mem_Read(portBMP, BMP280_ADDR << 1, BMP280_REG_STATUS, 1, (uint8_t *)&info_bmp280, 3, min_wait_ms);
		}
	}
#else
	uint8_t dati[] = {BME280_REG_CTRL_HUM, BME280_OSRS_H,//for BME280_SENSOR only
						//for all bmx sensors
						BMP280_REG_CTRL,
						BMP280_OSRS_T | BMP280_OSRS_P | BMP280_FORCED1_MODE,
						BMP280_REG_CTRL + 1,
						BMP280_CONF_T_SB | BMP280_CONF_FILTER | BMP280_CONF_SPI3W};

	siCmd = msg_rdyTest;
	uint8_t *uk = &dati[2];
	uint16_t len = 4;

	if (chip_id == BME280_SENSOR) {
		uk = &dati[0];
		len = 6;
	}
	rt = HAL_I2C_Master_Transmit_DMA(portBMP, BMP280_ADDR << 1, uk, len);
#endif
	    return rt;
	}
	//-----------------------------------------------------------------------------
	bool i2c_getStat_bmx280()
	{
		uint8_t stat = (info_bmp280.stat >> 3) & 1;

		return (stat == 0) ? true : false;
	}
	//-----------------------------------------------------------------------------

#ifdef SET_COMPAS_BLOCK
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef i2c_read_bmx280(uint8_t reg, uint8_t *data_rd, size_t size)
	{
		return HAL_I2C_Mem_Read(portBMP, BMP280_ADDR << 1, reg, 1, data_rd, size, max_wait_ms);
	}
	//-----------------------------------------------------------------------------
	bool bmx280_readCalibrationData(uint8_t chip_id)
	{
	bool ret = false;
	size_t dl = 24;

		if (chip_id == BME280_SENSOR) dl++;
		memset(data_bmp280, 0, sizeof(data_bmp280));
	    if (i2c_read_bmx280(BMP280_REG_CALIB, &data_bmp280[0], dl) == HAL_OK) {
	        memset(&calib, 0, sizeof(bmx280_calib_t));
	        calib.dig_T1 = (data_bmp280[1] << 8) | data_bmp280[0];
	        calib.dig_T2 = (data_bmp280[3] << 8) | data_bmp280[2];
	        calib.dig_T3 = (data_bmp280[5] << 8) | data_bmp280[4];
	        calib.dig_P1 = (data_bmp280[7] << 8) | data_bmp280[6];
	        calib.dig_P2 = (data_bmp280[9] << 8) | data_bmp280[8];
	        calib.dig_P3 = (data_bmp280[11] << 8) | data_bmp280[10];
	        calib.dig_P4 = (data_bmp280[13] << 8) | data_bmp280[12];
	        calib.dig_P5 = (data_bmp280[15] << 8) | data_bmp280[14];
	        calib.dig_P6 = (data_bmp280[17] << 8) | data_bmp280[16];
	        calib.dig_P7 = (data_bmp280[19] << 8) | data_bmp280[18];
	        calib.dig_P8 = (data_bmp280[21] << 8) | data_bmp280[20];
	        calib.dig_P9 = (data_bmp280[23] << 8) | data_bmp280[22];
	        if (chip_id == BME280_SENSOR) {//humidity
	            calib.dig_H1 = data_bmp280[24];
	            if (i2c_read_bmx280(0xE1, &data_bmp280[0], 7) == HAL_OK) {
	            	calib.dig_H2 = (data_bmp280[1] << 8) | data_bmp280[0];
	            	calib.dig_H3 = data_bmp280[2];
	            	calib.dig_H4 = (data_bmp280[3] << 4) | (0x0f & data_bmp280[4]);
	            	calib.dig_H5 = (data_bmp280[5] << 4) | ((data_bmp280[4] >> 4) & 0x0F);
	            	calib.dig_H6 = data_bmp280[6];
	            	ret = true;
	            } else devError |= devI2C1;
	        } else ret = true;
	    } else devError |= devI2C1;

	    return ret;
	}
#else
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef i2c_read_data_bmx280(uint8_t *data_rd, size_t size)
	{
	HAL_StatusTypeDef rt = HAL_ERROR;
	uint8_t adr = BMP280_REG_PRESSURE;

		siCmd = msg_readBMX;

		if ((rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, &adr, 1, min_wait_ms)) == HAL_OK)
			rt |= HAL_I2C_Master_Receive_DMA(portBMP, BMP280_ADDR << 1, data_rd, size);

		return rt;
	}
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef i2c_test_ready_bmx280(info_bmp280_t *info_bmp280)
	{
	HAL_StatusTypeDef rt = HAL_ERROR;
	uint8_t adr = BMP280_REG_STATUS;

		siCmd = msg_rdyStat;

		if ((rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, &adr, 1, min_wait_ms)) == HAL_OK)
			rt |= HAL_I2C_Master_Receive_DMA(portBMP, BMP280_ADDR << 1, (uint8_t *)&info_bmp280, 3);

		return rt;
	}
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef bmx280_readCalibr1Data(uint8_t chip_id)
	{
	size_t dl = 24;
	uint8_t adr = BMP280_REG_CALIB;
	HAL_StatusTypeDef rt = HAL_ERROR;

		if (chip_id == BME280_SENSOR) dl++;
		siCmd = msg_calibr1Done;

		memset(data_bmp280, 0, sizeof(data_bmp280));

		rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, &adr, 1, min_wait_ms);
		if (rt == HAL_OK) rt = HAL_I2C_Master_Receive_DMA(portBMP, BMP280_ADDR << 1, data_bmp280, dl);

		if (rt) devError |= devI2C1;

		return rt;
	}
	//-----------------------------------------------------------------------------
	void bmx280_calcCalibr1Data(uint8_t chip_id)
	{
		memset(&calib, 0, sizeof(bmx280_calib_t));

		calib.dig_T1 = (data_bmp280[1] << 8) | data_bmp280[0];
		calib.dig_T2 = (data_bmp280[3] << 8) | data_bmp280[2];
		calib.dig_T3 = (data_bmp280[5] << 8) | data_bmp280[4];
		calib.dig_P1 = (data_bmp280[7] << 8) | data_bmp280[6];
		calib.dig_P2 = (data_bmp280[9] << 8) | data_bmp280[8];
		calib.dig_P3 = (data_bmp280[11] << 8) | data_bmp280[10];
		calib.dig_P4 = (data_bmp280[13] << 8) | data_bmp280[12];
		calib.dig_P5 = (data_bmp280[15] << 8) | data_bmp280[14];
		calib.dig_P6 = (data_bmp280[17] << 8) | data_bmp280[16];
		calib.dig_P7 = (data_bmp280[19] << 8) | data_bmp280[18];
		calib.dig_P8 = (data_bmp280[21] << 8) | data_bmp280[20];
		calib.dig_P9 = (data_bmp280[23] << 8) | data_bmp280[22];

		if (chip_id == BME280_SENSOR) calib.dig_H1 = data_bmp280[24];//humidity
	}
	//-----------------------------------------------------------------------------
	HAL_StatusTypeDef bmx280_readCalibr2Data()
	{
	HAL_StatusTypeDef rt = HAL_ERROR;
	uint8_t adr = BMP280_REG_CALIB_EXT;

		siCmd = msg_calibr2Done;

		rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, &adr, 1, min_wait_ms);
		if (rt == HAL_OK) rt = HAL_I2C_Master_Receive_DMA(portBMP, BMP280_ADDR << 1, data_bmp280, 7);

		if (rt) devError |= devI2C1;

		return rt;
	}
	//-----------------------------------------------------------------------------
	void bmx280_calcCalibr2Data()
	{
		calib.dig_H2 = (data_bmp280[1] << 8) | data_bmp280[0];
		calib.dig_H3 = data_bmp280[2];
		calib.dig_H4 = (data_bmp280[3] << 4) | (0x0f & data_bmp280[4]);
		calib.dig_H5 = (data_bmp280[5] << 4) | ((data_bmp280[4] >> 4) & 0x0F);
		calib.dig_H6 = data_bmp280[6];
	}
	//-----------------------------------------------------------------------------
#endif

	//-----------------------------------------------------------------------------
	void bmx280_CalcAll(result_t *ssen, int32_t chip_id)//int32_t tp, uint32_t pp, uint32_t hh)
	{
	double var1, var2, p, var_H;
	double t1, p1, h1 = -1.0;

		//
		uint32_t hh = 0;
		uint32_t pp = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
		int32_t tp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
		if (chip_id == BME280_SENSOR) hh = (data_rdx[6] << 8) | data_rdx[7];
		//

	    //Temp // Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
	    var1 = (((double) tp) / 16384.0 - ((double)calib.dig_T1)/1024.0) * ((double)calib.dig_T2);
	    var2 = ((((double) tp) / 131072.0 - ((double)calib.dig_T1)/8192.0) * (((double)tp)/131072.0 - ((double) calib.dig_T1)/8192.0)) * ((double)calib.dig_T3);
	    // t_fine carries fine temperature as global value
	    int32_t t_fine = (int32_t)(var1 + var2);
	    t1 = (var1 + var2) / 5120.0;

	    //Press // Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
	    var1 = ((double)t_fine / 2.0) - 64000.0;
	    var2 = var1 * var1 * ((double) calib.dig_P6) / 32768.0;
	    var2 = var2 + var1 * ((double) calib.dig_P5) * 2.0;
	    var2 = (var2 / 4.0) + (((double) calib.dig_P4) * 65536.0);
	    var1 = (((double) calib.dig_P3) * var1 * var1 / 524288.0 + ((double) calib.dig_P2) * var1) / 524288.0;
	    var1 = (1.0 + var1 / 32768.0) * ((double) calib.dig_P1);
	    if (var1 == 0.0) {
	        p = 0;
	    } else {
	        p = 1048576.0 - (double)pp;
	        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	        var1 = ((double) calib.dig_P9) * p * p / 2147483648.0;
	        var2 = p * ((double) calib.dig_P8) / 32768.0;
	        p += (var1 + var2 + ((double) calib.dig_P7)) / 16.0;
	    }
	    p1 = (p/100) * 0.75006375541921;//convert hPa to mmHg

	    if (chip_id == BME280_SENSOR) {// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
	    	var_H = (((double)t_fine) - 76800.0);
	    	var_H = (hh - (((double)calib.dig_H4) * 64.0 + ((double)calib.dig_H5) / 16384.0 * var_H)) *
	    			(((double)calib.dig_H2) / 65536.0 * (1.0 + ((double)calib.dig_H6) / 67108864.0 * var_H *
	    			(1.0 + ((double)calib.dig_H3) / 67108864.0 * var_H)));
	        var_H = var_H * (1.0 - ((double)calib.dig_H1) * var_H / 524288.0);
	        if (var_H > 100.0) {
	        	var_H = 100.0;
	        } else {
	        	if (var_H < 0.0) var_H = 0.0;
	        }
	        h1 = var_H;
	    }

	    //ssen->chip = (float)chip_id;
	    ssen->bmx_temp = (float)t1;
	    ssen->bmx_pres = (float)p1;
	    ssen->bmx_humi = (float)h1;

	}

	//******************************************************************************************

#endif



