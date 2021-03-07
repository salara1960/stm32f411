

#include "hdr.h"
#include "main.h"
#include "ble.h"

#ifdef SET_BLE

//------------------------------------------------------------------------

const char *at_cmd[MAX_AT_CMD] = {
		"AT+VER\r\n",    // | +VER:JDY-23-V1.2
        "AT+BAUD0\r\n",  // | +OK ; //AT+BAUD<Param> Param:0——11520,1——57600,2——38400,3——19200,4——9600,5——4800,6——2400,	Default: 4
		"AT+MAC\r\n",    // AT+MAC<Param> | +OK Param: (MAC address string)
		"AT+RST\r\n",    // | +OK
		"AT+DISC\r\n",   // | +OK"
		"AT+STAT\r\n",   // +STAT:<Param> - 00: indicates not connected ,	01: indicates connected
		"AT+SLEEP1\r\n", // | +OK //	1: light sleep (with broadcast)
		"AT+SLEEP2\r\n", // | +OK // 2: Deep sleep (no broadcast)
		"AT+NAME\r\n",   // | +OK // Param: module Bluetooth,Maximum: 24 bytes,Default name: JDY-23
		"AT+ADVIN\r\n",  // | +ADVIN:<Param> ; Param:(0-9) : 0:100ms,1:200ms,2:300ms,3:400ms,4:500ms,5:600ms,6:700ms,7:800ms,8:900ms,9:10000ms;Default: 1
		"AT+IBUUID\r\n", // | +OK ; Param:Hex +IBUUID:<Param> ; Default: FDA50693A4E24FB1AFCFC6EB07647825 - FDA50693A4E24FB1AFCFC6EB07647825
		"AT+ALED0\r\n",  // | +ALED:<Param> ; 0: turn off the broadcast LED indicator
		"AT+ALED1\r\n",  // | +ALED:<Param> ; 1: turn on the broadcast LED indicator ; Default: 1
		"AT+DEFAULT\r\n",// | +OK
		"AT+MTU1\r\n",   // | +OK ; Param:(1-2) ; 1: 20 byte
		"AT+MTU2\r\n"    // | +OK ; Param:(1-2) ; 2: 128 byte, Default: 1
};

const char *at_ack[MAX_AT_ACK] = {
	"+OK",
	"+Ready",
	"+VER:",    	// +VER:JDY-23-V1.2
    "+CONNECTED",
	"+DISCONNECT",
	"+MTU:", // +MTU:<Param>
	"+ALED:", // +ALED:<Param>
	"+IBUUID:", // +IBUUID:<Param>
	"+ADVIN:", // +ADVIN:<Param>
	"+STARTEN:", // +STARTEN:<Param
	"+NAME:", // +NAME:<Param>
	"+BAUD:", // +BAUD:<Param> : 0——11520,1——57600,2——38400,3——19200,4——9600,5——4800,6——2400 	Default: 4
	"+MAC:" // +MAC:<Param>
	"+STAT:" // +STAT:<Param> :  00: indicates not connected , 01: indicates connected
};

const char *blePassword = "qwerty";

char bleTxBuf[256] = {0};
int bleCmd = iNone;
volatile uint8_t ble_stat = 0;

//******************************************************************************************

void bleWrite(int cd, char *str)
{
	if ((cd <= iNone) || (cd >= iLast)) return;

	ble_ack_wait = 1;
	txDoneFlagBLE = 0;
	if (!str) {
		HAL_UART_Transmit_DMA(blePort, (uint8_t *)at_cmd[cd], strlen(at_cmd[cd]));
	} else {
		HAL_UART_Transmit_DMA(blePort, (uint8_t *)str, strlen(str));
	}
}
//-----------------------------------------------------------------------------------------
void bleReset(uint8_t hard)
{
	if (hard) {
		BLE_RST_ON();
		HAL_Delay(1);
		BLE_RST_OFF();
		HAL_Delay(1);
	} else {
		bleCmd = iRST;
		bleWrite(bleCmd, NULL);
	}
}
//-----------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------
void bleGetVer()
{

}
//-----------------------------------------------------------------------------------------
int bleAckParse(char *uk)
{
int ret = -1;

	if (strstr(uk, at_ack[ackCON])) {
		ret = ackCON;
	} else if (strstr(uk, at_ack[ackDISC])) {
		ret = ackDISC;
	} else if (strstr(uk, blePassword)) {
		ret = ackPWD;
	}

	return ret;
}
//-----------------------------------------------------------------------------------------
void bleDisconnect()
{
	if (con_ble) {
		bleCmd = iDISC;
		bleWrite(bleCmd, NULL);
	}
}
//-----------------------------------------------------------------------------------------
void bleSpeed()
{
	bleCmd = iBAUD0;
	bleWrite(bleCmd, NULL);
}
//******************************************************************************************

#endif

