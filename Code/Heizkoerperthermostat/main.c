/*
 * Heizkoerperthermostat.c
 *
 * Created: 01.10.2022 10:35:38
 * Author : matti
 */ 

//Includes
#include "main.h"

void Disp_PrintTemperature(int ui2print);

//Main function
int main(void)
{
	GerneralInit();
	char i = 0;//Zählvariable für farbenverlauf
	
	//Can
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);
	//MCP2515_Set_Filter_Mask(MCP2515_1, &sFilter);
	
	//Temp read======================================================
	uint8_t tmp75Adress =  0b1001000; //all address pins to gnd/  address according to book: 0x90;
	uint8_t tmp75TempReg = 0b00000000;	//Read temperture register
	
	

	while (1)
	{
		
		
		//===============================================================
		
		if(Timer1_get_10msState() == TIMER_TRIGGERED)
		{
			TMP75_Read_Temperature(tmp75Adress, tmp75TempReg);
			Disp_PrintTemperature(TMP75_Get_Temperature());//Asks for temp value; prints to display
			
			
			
			//Color definition===========================================
			uint8_t uGreen[3] = {7/*green*/,0/*red*/,0/*blue*/};
			uint8_t uBrightGreen[3] = {5/*green*/,2/*red*/,0/*blue*/};
			uint8_t uYellow[3] = {3/*green*/,4/*red*/,0/*blue*/};
			uint8_t uOrange[3] = {2/*green*/,4/*red*/,1/*blue*/};
			uint8_t uMagenta[3] = {0/*green*/,4/*red*/,3/*blue*/};
			uint8_t uRed[3] = {0/*green*/,7/*red*/,0/*blue*/};
			//============================================================
			
			if (i == 0)
			{
				WS2812_Set_Colour(uGreen,2);
				i=1;
			}
			else if (i == 1)
			{
				WS2812_Set_Colour(uBrightGreen,2);
				i=2;
			}
			else if (i == 2)
			{
				WS2812_Set_Colour(uYellow,2);
				i=3;
			}
			else if (i == 3)
			{
				WS2812_Set_Colour(uOrange,2);
				i=4;
			}
			else if (i == 4)
			{
				WS2812_Set_Colour(uMagenta,2);
				i=5;
			}
			else if (i == 5)
			{
				WS2812_Set_Colour(uRed,2);
				i=0;
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

void Disp_PrintTemperature(int ui2print){
	//Beschreibung:		Anzeigen der aktuellen temperatur
	//Aufrufparameter:	None
	//Rückgabewert:		None
	//Version:			2
	//Datum:			3.11.22
	//Autor:			mh
	//Status:			ok
	//--------------------------------
	//Prefill of array-----------------------------
	unsigned char ucDisp[16] = "Isttemp:       C";		
	//---------------------------------------------
	//Handling negative numbers-------------------------------------
	unsigned char ucNegFlag = 0;	//rememberes if number negative
	if (ui2print < 0)
	{
		ui2print = ui2print *(-1);
		ucNegFlag = 1;
		
	}
	//---------------------------------------------------------------
	//add temp value to array---------------------------------------------------------------------
	for(unsigned char i = 0; i < 6; i++){
		if (i == 1)
		{
			ucDisp[(13-i)] = '.';
			i++;
		}
		//Geht stelle für stelle der int Zahl durch und schreibt sie in array, von hinten beginnend
		ucDisp[(13 - i)] = (char)(ui2print % 10) + 48;
		ui2print = ui2print / 10;
	}
	//---------------------------------------------------------------------------------------------
	//Replace zeros by spaces-----------------------------------------------------------------------
	unsigned char ucNotZero = 0; //varaible to break while loop at first number not 0
	unsigned char ucPosDispArray = 8; //Position in array for while loop
	while (ucNotZero == 0)
	{
		//Breaks if number is not 0 or last digit is reached
		if((ucDisp[ucPosDispArray] != '0') || (ucPosDispArray >= 11) )
		{
			ucNotZero = 1;
			if (ucNegFlag == 1) /*showing - if at negative temp*/
			{
				ucDisp[(ucPosDispArray-1)] = '-';
			}
		}
		
		else if (ucDisp[ucPosDispArray] == '0')
		{
			ucDisp[ucPosDispArray] = ' ';
			ucPosDispArray++;
		}
	}
	//------------------------------------------------------------------------------------------------
	
	//Print to Display-----------------------------------------------
	Display_SetCursor(0,0);
	Display_Print(ucDisp,16);
	Display_SetCursor(2,0);		//Platzieren des Cursors auserhalb
	//---------------------------------------------------------------
}