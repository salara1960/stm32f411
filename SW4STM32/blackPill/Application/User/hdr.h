/*
 * hdr.h
 *
 *  Created on: Dec 2, 2020
 *      Author: alarm
 */

#ifndef APPLICATION_USER_HDR_H_
#define APPLICATION_USER_HDR_H_

#define SET_BMx280

//#define SET_OLED_SPI
#define SET_IPS
#ifdef SET_IPS
	//#define SET_WITH_CS
	#define SET_WITH_DMA
#endif

//#define SET_COMPAS_BLOCK

#define SET_MPU
//#define SET_MPU_INTERRUPT

#define SET_cJSON
#ifdef SET_cJSON
	#define SET_LOW_PRECISION
	#define SET_FLOAT_FORMAT "%1.2f"
#endif

#define SET_IRED

#define SET_NET

#endif /* APPLICATION_USER_HDR_H_ */
