/*
 * Heizkoerperthermostat.c
 *
 * Created: 01.10.2022 10:35:38
 * Author : matti
 */ 

//Includes
#include "main.h"

//Main function
int main(void)
{
	

	
	GerneralInit();
	while (1)
	{
		//von Petre sora; timer1 muss erstellt werden
		//if(Timer1_get_1sState() == TIMER_TRIGGERED)
		//{
			////PORTD ^= 1 << PD6;
			//ucTimer++;
			//sSendFrame.EIDE_Bit = STANDARD_ID;
			//sSendFrame.RTR_Bit = DATA_FRAME;
			//sSendFrame.ulID = 0x30;
			//sSendFrame.ucLength = 1;
			//sSendFrame.ucData[0] = ucTimer;
			//MCP2515_Send_Message(MCP2515_1, &sSendFrame);
		//}
		//
		//if(Timer1_get_100msState() == TIMER_TRIGGERED)
		//{
			//if(MCP2515_Check_Message(MCP2515_1, &sRecFrame) == MESSAGE_RECEIVED)
			//{//es wird geprüft ob eine Botschaft über CAN empfangen wurde
				////wenn ja, wird die Botschaft in sRecFrame gespeichert
				////PORTD ^= 1 << PD6; //dient nur zum Testen
				//if(sRecFrame.ulID == 0x11)//eine Botschaft mit der ID 0x11 wurde empfangen
				//{
					//if(sRecFrame.ucData[0] == 0x40) LED_red_On(); //wenn das erste Byte der
					////Botschaft 0x40 ist, wird die grüne LED eingeschaltet
				//}
				//else if(sRecFrame.ulID == 0x22)
				//{
					//LED_red_Off();
				//}
				//
			//}
		//}
	}
	

}

