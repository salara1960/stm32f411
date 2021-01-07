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

#define htons(x) \
    ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))
#define htonl(x) \
    ((uint32_t)((x >> 24) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000) | ((x << 24) & 0xff000000)))


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


	//-----------------------------------------------
#ifdef SET_MPU

	#define MPU_BUF_SIZE                   20 //7 * 2 = количество байт для чтения OUT-регистров

	#define MPU6050_ADDRESS_AD0_LOW      0x68 // address pin low (GND), default for InvenSense evaluation board
	#define MPU6050_ADDRESS_AD0_HIGH     0x69 // address pin high (VCC)
	#define MPU6050_DEFAULT_ADDRESS      (MPU6050_ADDRESS_AD0_LOW<<1)
	#define MPU6050_RA_WHO_AM_I          0x75
	#define MPU6050_TEMP_OUT_H			 0x41

	/* MPU6050 registers */
	#define MPU6050_AUX_VDDIO			0x01
	#define MPU6050_SMPLRT_DIV			0x19
	#define MPU6050_CONFIG				0x1A
	#define MPU6050_GYRO_CONFIG			0x1B
	#define MPU6050_ACCEL_CONFIG		0x1C
	#define MPU6050_MOTION_THRESH		0x1F
	#define MPU6050_INT_PIN_CFG			0x37
	#define MPU6050_INT_ENABLE			0x38
	#define MPU6050_INT_STATUS			0x3A
	#define MPU6050_ACCEL_XOUT_H		0x3B
	#define MPU6050_ACCEL_XOUT_L		0x3C
	#define MPU6050_ACCEL_YOUT_H		0x3D
	#define MPU6050_ACCEL_YOUT_L		0x3E
	#define MPU6050_ACCEL_ZOUT_H		0x3F
	#define MPU6050_ACCEL_ZOUT_L		0x40
	#define MPU6050_TEMP_OUT_H			0x41
	#define MPU6050_TEMP_OUT_L			0x42
	#define MPU6050_GYRO_XOUT_H			0x43
	#define MPU6050_GYRO_XOUT_L			0x44
	#define MPU6050_GYRO_YOUT_H			0x45
	#define MPU6050_GYRO_YOUT_L			0x46
	#define MPU6050_GYRO_ZOUT_H			0x47
	#define MPU6050_GYRO_ZOUT_L			0x48
	#define MPU6050_MOT_DETECT_STATUS	0x61
	#define MPU6050_SIGNAL_PATH_RESET	0x68
	#define MPU6050_MOT_DETECT_CTRL		0x69
	#define MPU6050_USER_CTRL			0x6A
	#define MPU6050_PWR_MGMT_1			0x6B
	#define MPU6050_PWR_MGMT_2			0x6C
	#define MPU6050_FIFO_COUNTH			0x72
	#define MPU6050_FIFO_COUNTL			0x73
	#define MPU6050_FIFO_R_W			0x74
	#define MPU6050_WHO_AM_I			0x75

	/* Gyro sensitivities in degrees/s */
	#define MPU6050_GYRO_SENS_250		((float) 131)
	#define MPU6050_GYRO_SENS_500		((float) 65.5)
	#define MPU6050_GYRO_SENS_1000		((float) 32.8)
	#define MPU6050_GYRO_SENS_2000		((float) 16.4)

	/* Acce sensitivities in g/s */
	#define MPU6050_ACCE_SENS_2			((float) 16384)
	#define MPU6050_ACCE_SENS_4			((float) 8192)
	#define MPU6050_ACCE_SENS_8			((float) 4096)
	#define MPU6050_ACCE_SENS_16		((float) 2048)

	#define SD_MPU6050_DataRate_8KHz       0   // Sample rate set to 8 kHz
	#define SD_MPU6050_DataRate_4KHz       1   // Sample rate set to 4 kHz
	#define SD_MPU6050_DataRate_2KHz       3   // Sample rate set to 2 kHz
	#define SD_MPU6050_DataRate_1KHz       7   // Sample rate set to 1 kHz
	#define SD_MPU6050_DataRate_500Hz      15  // Sample rate set to 500 Hz
	#define SD_MPU6050_DataRate_250Hz      31  // Sample rate set to 250 Hz
	#define SD_MPU6050_DataRate_125Hz      63  // Sample rate set to 125 Hz
	#define SD_MPU6050_DataRate_100Hz      79  // Sample rate set to 100 Hz

	// Parameters for accelerometer range
	typedef enum  {
		SD_MPU6050_Accelerometer_2G = 0x00, // Range is +- 2G
		SD_MPU6050_Accelerometer_4G = 0x01, // Range is +- 4G
		SD_MPU6050_Accelerometer_8G = 0x02, // Range is +- 8G
		SD_MPU6050_Accelerometer_16G = 0x03 // Range is +- 16G
	} SD_MPU6050_Accelerometer;

	//Parameters for gyroscope range
	typedef enum {
		SD_MPU6050_Gyroscope_250s = 0x00,  // Range is +- 250 degrees/s
		SD_MPU6050_Gyroscope_500s = 0x01,  // Range is +- 500 degrees/s
		SD_MPU6050_Gyroscope_1000s = 0x02, // Range is +- 1000 degrees/s
		SD_MPU6050_Gyroscope_2000s = 0x03  // Range is +- 2000 degrees/s
	} SD_MPU6050_Gyroscope;



	#pragma pack(push,1)
		typedef struct {
			uint8_t adr;
			float Acce_Mult; // Accelerometer corrector from raw data to "g". Only for private use
			float Gyro_Mult; // Gyroscope corrector from raw data to "degrees/s". Only for private use
			float TEMP;
			int16_t xACCEL;
			int16_t yACCEL;
			int16_t zACCEL;
			int16_t xGYRO;
			int16_t yGYRO;
			int16_t zGYRO;
		} mpu_data_t;
	#pragma pack(pop)

	#pragma pack(push,1)
		typedef struct {
			uint8_t H;
			uint8_t L;
		} bit16_t;
	#pragma pack(pop)

	#pragma pack(push,1)
		typedef struct {
			int16_t ACCEL_XOUT; //0x3B
			int16_t ACCEL_YOUT; //0x3d
			int16_t ACCEL_ZOUT; //0x3f
			int16_t TEMP_OUT;   //0x41
			int16_t GYRO_XOUT;  //0x43
			int16_t GYRO_YOUT;  //0x45
			int16_t GYRO_ZOUT;  //0x47
		} mpu_all_data_t;
    #pragma pack(pop)

	typedef union {
		struct {
			uint8_t DataReady:1;       // Data ready interrupt
			uint8_t reserved2:2;       // Reserved bits
			uint8_t Master:1;          // Master interrupt. Not enabled with library
			uint8_t FifoOverflow:1;    // FIFO overflow interrupt. Not enabled with library
			uint8_t reserved1:1;       // Reserved bit
			uint8_t MotionDetection:1; // Motion detected interrupt
			uint8_t reserved0:1;       // Reserved bit
		} F;
		uint8_t Status;
	} mpu_interrupt_t;

	mpu_interrupt_t mpu_interrupt;
	mpu_all_data_t mpu_all_data;
	mpu_data_t mpu_data;

	HAL_StatusTypeDef mpuID();
	HAL_StatusTypeDef mpuInit();
	HAL_StatusTypeDef mpuAllRead();
	void mpuConvData();
	HAL_StatusTypeDef mpuEnableInterrupts();
	HAL_StatusTypeDef mpuDisableInterrupts();
	HAL_StatusTypeDef mpuReadInterruptsStatus();


#endif
	//-----------------------------------------------


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
	float COPMAS_CalcAngle();



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
