/*
 * sensors.c
 *
 *  Created on: Dec 2, 2020
 *      Author: alarm
 */

#include "hdr.h"
#include "sensors.h"


const uint32_t min_wait_ms = 150;
const uint32_t max_wait_ms = 500;
float qmc_temp_zero = 0;//-2.0;//  # Just a first value

#ifdef SET_COMPAS
	uint8_t hmc_ini[] = {Mode_Continuous | ODR_200Hz | RNG_2G | OSR_512, 0, 1};

	uint8_t	magBuf[MAG_BUF_SIZE] = {0};
	uint8_t cStat = 0;
	xyz_t *xyz = NULL;
#endif

//******************************************************************************************

#ifdef SET_MPU6050

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
HAL_StatusTypeDef mpuChipID()
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
#ifdef SET_COMPAS
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
#endif

//******************************************************************************************
#ifdef SET_MPU9250

const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint16_t _dev_add = 0x68;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_ST1 = 0x02;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;


static uint8_t _buffer[16];//21
static uint8_t _mag_adjust[3] = {0};

mpu_akm_data_t mpu_akm_data = {0};

//----------------------------------------------------------------

__weak void MPU9250_OnActivate()
{
}
//----------------------------------------------------------------
bool MPU9250_IsConnected()
{
	if (HAL_I2C_IsDeviceReady(portMPU, _dev_add << 1, 1, max_wait_ms) == HAL_OK)
		return true;
	else
		return false;
}
//----------------------------------------------------------------
void MPU_I2C_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	if (HAL_I2C_Mem_Write(portMPU, _dev_add << 1, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite, max_wait_ms) != HAL_OK)
		devError |= devI2C1;
}
//----------------------------------------------------------------
void MPU_I2C_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if (HAL_I2C_Mem_Read(portMPU, _dev_add << 1, ReadAddr | READWRITE_CMD, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, min_wait_ms) != HAL_OK)
		devError |= devI2C1;
}
//----------------------------------------------------------------
// writes a byte to MPU9250 register given a register address and data
void writeRegister(uint8_t subAddress, uint8_t data)
{
	//MPU_I2C_Write(&data, subAddress, 1);
	if (HAL_I2C_Mem_Write(portMPU, _dev_add << 1, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, min_wait_ms) != HAL_OK)
		devError |= devI2C1;

	HAL_Delay(10);
}
//----------------------------------------------------------------
// writes a byte to MPU9250 register given a register address and data
void writeRegisters(uint8_t subAddress, uint8_t count, uint8_t *data)
{
	if (HAL_I2C_Mem_Write(portMPU, _dev_add << 1, subAddress, I2C_MEMADD_SIZE_8BIT, data, count, max_wait_ms)!= HAL_OK)
		devError |= devI2C1;

	HAL_Delay(10);
}
//----------------------------------------------------------------
// reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
	if (HAL_I2C_Mem_Read(portMPU, _dev_add << 1, subAddress | READWRITE_CMD, I2C_MEMADD_SIZE_8BIT, dest, count, max_wait_ms) != HAL_OK)
		devError |= devI2C1;
}
//----------------------------------------------------------------
// writes a register to the AK8963 given a register address and data
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{

	/* set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG, subAddress);*/

	uint8_t dat[] = {AK8963_I2C_ADDR, subAddress};
	writeRegisters(I2C_SLV0_ADDR, sizeof(dat), dat);

	// store the data for write
	writeRegister(I2C_SLV0_DO, data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
}
//----------------------------------------------------------------
// reads registers from the AK8963
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
	/* set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG, subAddress);
	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);*/

	uint8_t dat[] = {AK8963_I2C_ADDR | I2C_READ_FLAG, subAddress, I2C_SLV0_EN | count};
	writeRegisters(I2C_SLV0_ADDR, sizeof(dat), dat);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00, count, dest);
}
//----------------------------------------------------------------
// gets the MPU9250 WHO_AM_I register value, expected to be 0x71
static uint8_t whoAmI()
{
	// read the WHO AM I register
	readRegisters(WHO_AM_I, 1, _buffer);

	// return the register value
	return _buffer[0];
}
//----------------------------------------------------------------
// gets the AK8963 WHO_AM_I register value, expected to be 0x48
static uint8_t whoAmIAK8963()
{
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer);
	// return the register value
	return _buffer[0];
}
//----------------------------------------------------------------
/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init(uint8_t *id1, uint8_t *id2)
{
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	//
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(100);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2, AK8963_RESET);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// check the WHO AM I byte, expected value is 0x71 or 0x73
	uint8_t byte = whoAmI();
	if (id1) *id1 = byte;
	if (byte != MPU_ID) return 1;//&& ( who != 0x73)) return 1;

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2, SEN_ENABLE);

	// setting accel range to 16G as default
	writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G);

	// setting the gyro range to 2000DPS as default
	writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS);

	// setting bandwidth to 184Hz as default
	writeRegister(ACCEL_CONFIG2, DLPF_184);

	// setting gyro bandwidth to 184Hz
	writeRegister(CONFIG, DLPF_184);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV, 0x00);

	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48
	byte = whoAmIAK8963();
	if (id2) *id2 = byte;
	if (byte != AKM_ID) return 1;

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL, 7, _buffer);

	// successful init, return 0
	return 0;
}
//----------------------------------------------------------------
/*
// sets the accelerometer full scale range to values other than default
void MPU9250_SetAccelRange(AccelRange range)
{
	writeRegister(ACCEL_CONFIG, range);
}
//----------------------------------------------------------------
// sets the gyro full scale range to values other than default
void MPU9250_SetGyroRange(GyroRange range)
{
	writeRegister(GYRO_CONFIG, range);
}
//----------------------------------------------------------------
// sets the DLPF bandwidth to values other than default
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2, bandwidth);
	writeRegister(CONFIG, bandwidth);
}
*/
//----------------------------------------------------------------
// sets the sample rate divider to values other than default
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
	// setting the sample rate divider to 19 to facilitate setting up magnetometer
	writeRegister(SMPDIV, 19);

	if (srd > 9) {
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer);
	} else {
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer);
	}

	writeRegister(SMPDIV, srd);
}
//----------------------------------------------------------------
void MPU9250_GetData(mpu_akm_data_t *data)
{
	readRegisters(ACCEL_OUT, 14, _buffer);

	data->accel_x = (((int16_t)_buffer[0])  << 8) | _buffer[1];
	data->accel_y = (((int16_t)_buffer[2])  << 8) | _buffer[3];
	data->accel_z = (((int16_t)_buffer[4])  << 8) | _buffer[5];

	data->temp = (((int16_t)_buffer[6]) << 8) | _buffer[7];

	data->gyro_x = (((int16_t)_buffer[8])  << 8) | _buffer[9];
	data->gyro_y = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	data->gyro_z = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	readAK8963Registers(AK8963_ST1, 8, _buffer);
	uint8_t st1 = _buffer[0];
	int16_t magx = (((int16_t)_buffer[1]) << 8) | _buffer[2];
	int16_t magy = (((int16_t)_buffer[3]) << 8) | _buffer[4];
	int16_t magz = (((int16_t)_buffer[5]) << 8) | _buffer[6];
	uint8_t st2 = _buffer[7];

	data->akm_x = (int16_t)( (float)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f) );
	data->akm_y = (int16_t)( (float)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f) );
	data->akm_z = (int16_t)( (float)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f) );


}
//------------------------------------------------------------------------------------------
uint8_t MPU9250_getStat()
{
uint8_t stat;

	readRegisters(MPU_STAT_REG, 1, &stat);

	return (stat & 1);
}
//------------------------------------------------------------------------------------------
void MPU9250_CalcAll(mpu_akm_data_t *data)
{
double azimut, x, y, con = 2 * M_PI;

	//compData.tempHMC = ((float)data->temp / 340.0) + 36.53;
	compData.tempHMC = (((float)data->temp - 21) / 333.87) + 21;

	x = data->akm_x;//xyz->x;// * 0.92;
	y = data->akm_y;//xyz->y;// * 0.92;
	//z = data->akm_z;//xyz->z;// * 0.92;

	azimut = atan2(y, x);
	if (azimut < 0) azimut += con;
	else
	if (azimut > con) azimut -= con;

	compData.angleHMC = (float)(azimut * (180 / M_PI));
}
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

#endif
//******************************************************************************************
#ifdef SET_BMx280

	//result_t sensors = {0.0, 0.0, 0.0};

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
#if defined(SET_COMPAS) && defined(SET_COMPAS_BLOCK)
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

#if defined(SET_COMPAS) && defined(SET_COMPAS_BLOCK)
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



