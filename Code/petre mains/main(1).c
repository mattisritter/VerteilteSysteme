/*
 * CAN-Test.c
 *
 * Created: 16.11.2021 14:14:49
 * Author : PS_HF
 */ 

#include "main.h"


int main(void)
{
    Init();
    while (1) 
    {
		if(Timer1_get_1sState() == TIMER_TRIGGERED)
		{
			//PORTD ^= 1 << PD6;
			ucTimer++;
			sSendFrame.EIDE_Bit = STANDARD_ID;
			sSendFrame.RTR_Bit = DATA_FRAME;
			sSendFrame.ulID = 0x30;
			sSendFrame.ucLength = 1;
			sSendFrame.ucData[0] = ucTimer;
			MCP2515_Send_Message(MCP2515_1, &sSendFrame);
		}
		
		if(Timer1_get_100msState() == TIMER_TRIGGERED)
		{
			if(MCP2515_Check_Message(MCP2515_1, &sRecFrame) == MESSAGE_RECEIVED)
			{//es wird geprüft ob eine Botschaft über CAN empfangen wurde
				//wenn ja, wird die Botschaft in sRecFrame gespeichert
				//PORTD ^= 1 << PD6; //dient nur zum Testen
				if(sRecFrame.ulID == 0x11)//eine Botschaft mit der ID 0x11 wurde empfangen
				{
					if(sRecFrame.ucData[0] == 0x40) LED_red_On(); //wenn das erste Byte der 
					//Botschaft 0x40 ist, wird die grüne LED eingeschaltet
				}
				else if(sRecFrame.ulID == 0x22)
				{
					LED_red_Off();
				}
				
			}
		}
    }
}

void Init(void)
{
	Display_Init();
	keys_Init();
	LED_Init();
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);
	Timer1_Init();
	CAN_Filter_Init();
	MCP2515_Set_Filter_Mask(MCP2515_1, &sFilter);
}


void CAN_Filter_Init(void)
{
	sFilter.Rec_Buff0_Rollover = ROLLOVER_ON;
	sFilter.RecBuff_ID[0] = STANDARD_ID;
	sFilter.RecBuff_ID[1] = STANDARD_ID;
	sFilter.Filter_RecBuff[0] = FILTER_ON;
	sFilter.Filter_RecBuff[1] = FILTER_ON;
	for(uint8_t ucI = 0; ucI < 6; ucI++)
	{
		sFilter.ulRecBuff_Filter[ucI] = ulReceiveFilter[ucI];
	}
	for(uint8_t ucI = 0; ucI < 2; ucI++)
	{
		sFilter.ulRecBuff_Mask[ucI] = ulReceiveMask[ucI];
	}
}