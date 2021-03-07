#ifndef WIFI__h
#define WIFI__h


#include "hdr.h"

#ifdef SET_WIFI

#include "main.h"

//-----------------------------------------------------------------------------

volatile uint8_t txDoneFlagWIFI;
volatile uint8_t rxDoneFlagWIFI;
extern SPI_HandleTypeDef *portNET;
extern uint32_t snd_pack;
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

void WIFI_ReadBuff(uint8_t* buff, uint16_t len);
void WIFI_WriteBuff(uint8_t* buff, uint16_t len);

//-----------------------------------------------------------------------------

#endif

#endif


