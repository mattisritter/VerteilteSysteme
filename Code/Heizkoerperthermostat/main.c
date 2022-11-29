/*
 * Heizkoerperthermostat.c
 *
 * Created: 01.10.2022 10:35:38
 * Author : matti
 */ 

//Includes
#include "main.h"

//Variables
unsigned char ucStep = 0;
unsigned char ucH = 5;
unsigned char ucStepOld = 0;
int iTargetTemp = 230;	//Initial set of target to avoid comma value is multiplied with 10
int iActualTemp;

//Main function
int main(void)
{
	GerneralInit();
	unsigned char ucCANStatus = CAN_NOT_RECEIVED;
	//CAN
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);
	//MCP2515_Set_Filter_Mask(MCP2515_1, &sFilter);
	// short delay for TMP75 to settle --> not allowed
// 	while (Timer1_get_10msState() == TIMER_RUNNING)
// 	{	
// 	}
	// Measurement for initializing first step
	TMP75_Read_Temperature();
	iActualTemp = TMP75_Get_Temperature();	//Asks for temp value;
	ucStep = TempController(iActualTemp,iTargetTemp, ucStepOld, 0);	// without hysteresis
	ucStepOld = ucStep; // save old heating step
	while (1)
	{
		// measure actual temperature
		if (Timer1_get_100msState() == TIMER_TRIGGERED)	/*Prevent tmp75 from overheating*/
		{
			TMP75_Read_Temperature();
			
			
			iActualTemp = TMP75_Get_Temperature();	//Asks for temp value;
			Display_Output(iActualTemp, 0, 255);
			// heating controller
			ucStep = TempController(iActualTemp,iTargetTemp, ucStepOld, ucH);
			WS2812_Step(ucStep); // set LED color
			Servo_Step(ucStep); // set servo position
			ucStepOld = ucStep; // save old heating step
		}
		
		// set target temperature		
		if(Timer1_get_10msState() == TIMER_TRIGGERED)
		{
			unsigned char ucKeyStatus = keys_get_state();	//Asks for key status
			switch(ucCANStatus)
			{
				case CAN_NOT_RECEIVED:
					Display_Output(iTargetTemp, 1, ucCANStatus);
					//Test if changes wished--------------
					if (ucKeyStatus == S2_PRESSED)
					{
						iTargetTemp += 5;
					}
					else if (ucKeyStatus == S1_PRESSED)
					{
						iTargetTemp -= 5;
					}
				break;
				
				case CAN_RECEIVED:
					iTargetTemp = iTargetTemp + 0; //moritz:platzhalter
				break;
			}
		}
		
		
		
		//sollte so eventuell schon laufen
		
		//if(Timer1_get_1sState() == TIMER_TRIGGERED)
		//{
			//sSendFrame.EIDE_Bit = STANDARD_ID;
			//sSendFrame.RTR_Bit = DATA_FRAME;
			//sSendFrame.ulID = 0x400;
			//sSendFrame.ucLength = 3;
			//sSendFrame.ucData[0] = (char)(iTemp/10);
			//sSendFrame.ucData[0] = (unsigned char)(iTemp%10);
			//sSendFrame.ucData[0] = ucStep;
			//MCP2515_Send_Message(MCP2515_1, &sSendFrame);
		//}
		//
		//if(Timer1_get_100msState() == TIMER_TRIGGERED)
		//{
			//if(MCP2515_Check_Message(MCP2515_1, &sRecFrame) == MESSAGE_RECEIVED)
			//{//es wird geprüft ob eine Botschaft über CAN empfangen wurde
				////wenn ja, wird die Botschaft in sRecFrame gespeichert
				////PORTD ^= 1 << PD6; //dient nur zum Testen
				//if(sRecFrame.ulID == 0x401)//eine Botschaft mit der ID 0x11 wurde empfangen
				//{
					// iTarget = (int)sRecFrame.ucDat*10;
				//
			//}
		//}
	}
	
}
