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
	
	char i = 0;
	
	GerneralInit();
	//Can
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);
	//MCP2515_Set_Filter_Mask(MCP2515_1, &sFilter);
	while (1)
	{
		
		if(Timer1_get_100msState() == TIMER_TRIGGERED)
		{
			
		
			
			
			uint8_t uGreen[3] = {7/*green*/,0/*red*/,0/*blue*/};
			uint8_t uBrightGreen[3] = {5/*green*/,2/*red*/,0/*blue*/};
			uint8_t uYellow[3] = {3/*green*/,4/*red*/,0/*blue*/};
			uint8_t uOrange[3] = {2/*green*/,4/*red*/,1/*blue*/};
			uint8_t uMagenta[3] = {0/*green*/,4/*red*/,3/*blue*/};
			uint8_t uRed[3] = {0/*green*/,7/*red*/,0/*blue*/};
				
			
			if (i == 0)
			{
				WS2812_set_off();
				WS2812_Set_Colour(uGreen,2);
				i=1;
			}
			else if (i == 1)
			{
				//WS2812_set_off();
				WS2812_Set_Colour(uBrightGreen,2);
				i=2;
			}
			else if (i == 2)
			{
				WS2812_set_off();
				WS2812_Set_Colour(uYellow,2);
				i=3;
			}
			else if (i == 3)
			{
				WS2812_set_off();
				WS2812_Set_Colour(uOrange,2);
				i=4;
			}
			else if (i == 4)
			{
				WS2812_set_off();
				WS2812_Set_Colour(uMagenta,2);
				i=5;
			}
			else if (i == 5)
			{
				WS2812_set_off();
				WS2812_Set_Colour(uRed,2);
				i=0;
			}
			
				
				
			unsigned char ucKey = keys_get_state();
			//durchlauf alle 100ms
			//testen der LED
			if (ucKey == S1_PRESSED)
			{
				WS2812_Set_Colour(uRed,2);
				LED_rd_toggle();	//ECU Board red green
			}
			
			if (ucKey == S2_PRESSED)
			{
				LED_rd_toggle();	//ECU Board red green
			}
			if (ucKey == S3_PRESSED)
			{
				LED_rd_toggle();	//ECU Board red green
			}	
			
		}
		
		
		
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

