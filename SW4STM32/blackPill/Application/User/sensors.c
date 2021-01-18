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
uint8_t hmc_ini[] = {Mode_Continuous | ODR_200Hz | RNG_2G | OSR_512, 0, 1};
float qmc_temp_zero = -2.0;//  # Just a first value

uint8_t	magBuf[MAG_BUF_SIZE] = {0};
uint8_t cStat = 0;
xyz_t *xyz = NULL;


//******************************************************************************************

#ifdef SET_MPU

mpu_all_data_t mpu_all_data;
mpu_data_t mpu_data;

//-----------------------------------------------------------------------------
/*
HAL_StatusTypeDef mpuInit()
{
HAL_StatusTypeDef rt = HAL_OK;
uint8_t devAddr = mpu_data.adr;

	if (!devAddr) goto err;
	devAddr <<= 1;
	uint8_t byte;
	//
	//----- Wakeup MPU6050 -----
	uint8_t dat[] = {MPU6050_PWR_MGMT_1, 0x00};
	rt |= HAL_I2C_Master_Transmit(portMPU, devAddr, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;
	//--------------------------
	//
	// Set sample rate to 1kHz
	dat[0] = MPU6050_SMPLRT_DIV;
	dat[1] = SD_MPU6050_DataRate_8KHz;//SD_MPU6050_DataRate_1KHz;
	rt |= HAL_I2C_Master_Transmit(portMPU, devAddr, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;
	//
	// Config accelerometer
	uint8_t AccelerometerSensitivity = SD_MPU6050_Accelerometer_2G;
	rt |= HAL_I2C_Mem_Read(portMPU, devAddr, MPU6050_ACCEL_CONFIG, 1, &byte, 1, min_wait_ms);
	if (rt != HAL_OK) goto err;
	byte = (byte & 0xE7) | AccelerometerSensitivity << 3;
	rt |= HAL_I2C_Mem_Write(portMPU, devAddr, MPU6050_ACCEL_CONFIG, 1, &byte, 1, min_wait_ms);
	if (rt != HAL_OK) goto err;
	// Set sensitivities for multiplying gyro and accelerometer data
	switch (AccelerometerSensitivity) {
		case SD_MPU6050_Accelerometer_2G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_2;
		break;
		case SD_MPU6050_Accelerometer_4G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_4;
		break;
		case SD_MPU6050_Accelerometer_8G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_8;
		break;
		case SD_MPU6050_Accelerometer_16G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_16;
		break;
	}
	//
	// Config Gyroscope
	rt |= HAL_I2C_Mem_Read(portMPU, devAddr, MPU6050_GYRO_CONFIG, 1, &byte, 1, min_wait_ms);
	if (rt != HAL_OK) goto err;
	uint8_t GyroscopeSensitivity = SD_MPU6050_Gyroscope_250s;
	byte = (byte & 0xE7) | GyroscopeSensitivity << 3;
	rt |= HAL_I2C_Mem_Write(portMPU, devAddr, MPU6050_GYRO_CONFIG, 1, &byte, 1, min_wait_ms);
	if (rt != HAL_OK) goto err;
	switch (GyroscopeSensitivity) {
		case SD_MPU6050_Gyroscope_250s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_250;
		break;
		case SD_MPU6050_Gyroscope_500s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_500;
		break;
		case SD_MPU6050_Gyroscope_1000s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_1000;
		break;
		case SD_MPU6050_Gyroscope_2000s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_2000;
		break;
	}

	return HAL_OK;

err:
	devError |= devI2C1;
	return rt;
}
*/
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/**/
HAL_StatusTypeDef mpuInit()
{
HAL_StatusTypeDef rt = HAL_OK;
uint8_t devAddr = mpu_data.adr;

	if (!devAddr) goto err;
	devAddr <<= 1;
	//
	//----- Wakeup MPU6050 -----
	uint8_t dat[] = {MPU6050_PWR_MGMT_1, 0};
	rt |= HAL_I2C_Master_Transmit(portMPU, devAddr, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;
	//
	//
	// Save SD_MPU6050_DataRate_1KHz to MPU6050_SMPLRT_DIV   and 3 to MPU6050_CONFIG
	dat[0] = SD_MPU6050_DataRate_8KHz;
	dat[1] = 3;
	rt |= HAL_I2C_Mem_Write(portMPU, devAddr, MPU6050_SMPLRT_DIV, 1, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;
	//
	//--------------------------
	//
	// Config accelerometer and Gyroscope
	uint8_t AccelerometerSensitivity = SD_MPU6050_Accelerometer_2G;
	uint8_t GyroscopeSensitivity = SD_MPU6050_Gyroscope_250s;
	rt |= HAL_I2C_Mem_Read(portMPU, devAddr, MPU6050_ACCEL_CONFIG, 1, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;//c & ~0xE0  //c & ~0x18 //c | Gscale << 3
	dat[0] &= 0xE0; dat[0] |= AccelerometerSensitivity << 3;
	dat[1] &= 0xE0; dat[1] |= GyroscopeSensitivity << 3;
	//dat[0] = (dat[0] & 0xE0) | AccelerometerSensitivity << 3;
	//dat[1] = (dat[1] & 0xE0) | GyroscopeSensitivity << 3;
	rt |= HAL_I2C_Mem_Write(portMPU, devAddr, MPU6050_ACCEL_CONFIG, 1, dat, 2, min_wait_ms);
	if (rt != HAL_OK) goto err;
	// Set sensitivities for multiplying gyro and accelerometer data
	switch (AccelerometerSensitivity) {
		case SD_MPU6050_Accelerometer_2G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_2;
		break;
		case SD_MPU6050_Accelerometer_4G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_4;
		break;
		case SD_MPU6050_Accelerometer_8G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_8;
		break;
		case SD_MPU6050_Accelerometer_16G:
			mpu_data.Acce_Mult = 1.0 / MPU6050_ACCE_SENS_16;
		break;
	}

	//
	switch (GyroscopeSensitivity) {
		case SD_MPU6050_Gyroscope_250s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_250;
		break;
		case SD_MPU6050_Gyroscope_500s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_500;
		break;
		case SD_MPU6050_Gyroscope_1000s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_1000;
		break;
		case SD_MPU6050_Gyroscope_2000s:
			mpu_data.Gyro_Mult = 1.0 / MPU6050_GYRO_SENS_2000;
		break;
	}

	return HAL_OK;

err:
	devError |= devI2C1;
	return rt;
}
/**/
//-----------------------------------------------------------------------------
HAL_StatusTypeDef mpuID()
{
HAL_StatusTypeDef rt = HAL_OK;

	rt = HAL_I2C_IsDeviceReady(portMPU, MPU6050_DEFAULT_ADDRESS, 2, 10);
	if (rt == HAL_OK) {
		uint8_t byte;
		rt |= HAL_I2C_Mem_Read(portMPU, MPU6050_DEFAULT_ADDRESS, MPU6050_WHO_AM_I, 1, &byte, 1, min_wait_ms);
		if (rt == HAL_OK) mpu_data.adr = byte;
	}

	if (rt) devError |= devI2C1;
	return rt;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef mpuAllRead()
{
uint8_t adr = MPU6050_ACCEL_XOUT_H;

	HAL_StatusTypeDef rt = HAL_I2C_Master_Transmit(portMPU, mpu_data.adr << 1, &adr, 1, min_wait_ms);
	if (rt == HAL_OK) {
		siCmd = msg_mpuAllReady;
		rt |= HAL_I2C_Master_Receive_DMA(portMPU, mpu_data.adr << 1, (uint8_t *)&mpu_all_data, sizeof(mpu_all_data_t));
	} else rt |= devI2C1;

	devError |= rt;

	return rt;
}
//-----------------------------------------------------------------------------
void mpuConvData()
{
	mpu_data.xACCEL = ((int16_t)(htons(mpu_all_data.ACCEL_XOUT))) * mpu_data.Acce_Mult;
	mpu_data.yACCEL = ((int16_t)(htons(mpu_all_data.ACCEL_YOUT))) * mpu_data.Acce_Mult;
	mpu_data.zACCEL = ((int16_t)(htons(mpu_all_data.ACCEL_ZOUT))) * mpu_data.Acce_Mult;
	mpu_data.TEMP   = (float)((int16_t)(htons(mpu_all_data.TEMP_OUT)) / 340.0 + 36.53);
	mpu_data.xGYRO  = ((int16_t)(htons(mpu_all_data.GYRO_XOUT))) * mpu_data.Gyro_Mult;
	mpu_data.yGYRO  = ((int16_t)(htons(mpu_all_data.GYRO_YOUT))) * mpu_data.Gyro_Mult;
	mpu_data.zGYRO  = ((int16_t)(htons(mpu_all_data.GYRO_ZOUT))) * mpu_data.Gyro_Mult;
}
//-----------------------------------------------------------------------------
void mpuEnableInterrupts()
{
uint8_t reg[] = {0x8c, 1};

	//set 0x9c to MPU6050_INT_PIN_CFG and 1 to MPU6050_INT_ENABLE
	if (HAL_I2C_Mem_Write(portMPU, mpu_data.adr << 1, MPU6050_INT_PIN_CFG, 1, reg, 2, min_wait_ms) != HAL_OK) devError |= devI2C1;
}
//-----------------------------------------------------------------------------
void mpuDisableInterrupts()
{
uint8_t reg[] = {MPU6050_INT_ENABLE, 0};

	if (HAL_I2C_Master_Transmit(portMPU, mpu_data.adr << 1, reg, 2, min_wait_ms) != HAL_OK) devError |= devI2C1;

}
//-----------------------------------------------------------------------------
uint8_t mpuReadStatus()
{
uint8_t stat = 0;

	if (HAL_I2C_Mem_Read(portMPU, mpu_data.adr << 1, MPU6050_INT_STATUS, 1, &stat, 1, min_wait_ms) != HAL_OK) devError |= devI2C1;

	return stat;
}
//-----------------------------------------------------------------------------

#endif

//******************************************************************************************
//******************************************************************************************
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
double azimut, x, y, con = 2 * M_PI;

	xyz = (xyz_t *)&magBuf[0];

	compData.tempHMC = ((float)(xyz->temp & 0x3fff) / 520.0) + qmc_temp_zero;

	x = xyz->x;// * 0.92;
	y = xyz->y;// * 0.92;
	//z = xyz->z;// * 0.92;

	azimut = atan2(y, x);
	if (azimut < 0) azimut += con;
	else
	if (azimut > con) azimut -= con;

	/*float VX = atan2(x, z) / 0.0174532925;
	if (VX < 0) VX += 360;
	VX = 360 - VX - 180;
	if (VX < 0) VX += 360;
	compData.x = (int)VX;
	//
	float VY = atan2(y, z) / 0.0174532925;
	if (VY < 0) VY += 360;
	VY = 360 - VY - 180;
	if (VY < 0) VY += 360;
	compData.y = (int)VY;*/

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



