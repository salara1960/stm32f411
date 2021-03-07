
#include "wifi.h"


#ifdef SET_WIFI

//-----------------------------------------------------------------------------

volatile uint8_t txDoneFlagWIFI = 1;
volatile uint8_t rxDoneFlagWIFI = 0;

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void WIFI_ReadBuff(uint8_t* buff, uint16_t len)
{
	rxDoneFlagWIFI = 0;

	CS_NET_SELECT();
		if (HAL_SPI_Receive(portNET, buff, len, HAL_MAX_DELAY) == HAL_ERROR) devError |= devNet;
	CS_NET_DESELECT();

	rxDoneFlagWIFI = 1;

}
//-----------------------------------------------------------------------------
void WIFI_WriteBuff(uint8_t* buff, uint16_t len)
{
	/*
	txDoneFlagWIFI = 0;

	CS_NET_SELECT();
		if (HAL_SPI_Transmit(portNET, buff, len, HAL_MAX_DELAY) == HAL_ERROR)
			devError |= devNet;
		else
			snd_pack++;
	CS_NET_DESELECT();

	txDoneFlagWIFI = 1;
	*/

	/**/
	//if (!txDoneFlagWIFI) return;

	txDoneFlagWIFI = 0;

	CS_NET_SELECT();
		if (HAL_SPI_Transmit_DMA(portNET, buff, len) == HAL_ERROR) devError |= devNet;
		else snd_pack++;
		//while (1) {
		//	HAL_SPI_StateTypeDef stat = HAL_SPI_GetState(portNET);
		//	if ((stat == HAL_SPI_STATE_BUSY_RX) || (stat == HAL_SPI_STATE_READY)) break;
		//}

	 /**/
}
//-----------------------------------------------------------------------------


#endif







