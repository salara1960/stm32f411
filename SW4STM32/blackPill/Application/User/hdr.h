/*
 * hdr.h
 *
 *  Created on: Dec 2, 2020
 *      Author: alarm
 */

#ifndef APPLICATION_USER_HDR_H_
#define APPLICATION_USER_HDR_H_

#define SET_BMx280

#define SET_OLED_SPI

//#define SET_COMPAS_BLOCK

//#define MIC_PRESENT

#define SET_MPU
//#define SET_MPU_INTERRUPT

//#define SET_JSON
#define SET_JFES
#ifdef SET_JFES
	#define JFES_DOUBLE_PRECISION 0.01//0.000000001
	#define SET_CALLOC_MEM
	//#define SET_MALLOC_MEM
#endif


#endif /* APPLICATION_USER_HDR_H_ */
