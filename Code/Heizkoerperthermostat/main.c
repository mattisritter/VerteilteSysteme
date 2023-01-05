/**
 * @file Heizkoerperthermostat main.c
 * @brief C file for main
 * @date 26.10.2022 19:54:57
 * @author Hoehnel and Ritter
 */ 

//Includes======================================================
#include "main.h"

//Variables=====================================================
unsigned char ucStep = 0; /**<Variable that stores the heating step*/
unsigned char ucH = 5; /**<Variable to implement hysteresis*/
unsigned char ucStepOld = 0; /**<Variable that stores the old value of heating step*/
int iTargetTemp = 230;	/**<Initial set of target; To avoid comma value it is multiplied with 10*/
int iActualTemp; /**<Current temperature*/

can_frame sSendFrame;
can_frame sRecFrame;

//Main function=====================================================
/** @brief Main function
* @param[in] None
* @return None
*/
int main(void)
{
	GerneralInit();
	unsigned char ucCANStatus = CAN_NOT_RECEIVED;
	//CAN
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);
	//CAN_Filter_Init();
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
		if (Timer1_get_100msState() == TIMER_TRIGGERED)	/*Prevent tmp75 from overheating, only start every 100ms*/
		{
			TMP75_Read_Temperature();							
			iActualTemp = TMP75_Get_Temperature();	//Asks for temp value;
			Display_Output(iActualTemp, 0, 255);
			
			if(MCP2515_Check_Message(MCP2515_1, &sRecFrame) == MESSAGE_RECEIVED)
			{//es wird gepr�ft ob eine Botschaft �ber CAN empfangen wurde
				//wenn ja, wird die Botschaft in sRecFrame gespeichert
				if(sRecFrame.ulID == 0x401)//eine Botschaft mit der ID 0x401 wurde empfangen
				{
					iTargetTemp = (int)(sRecFrame.ucData[0]*10);
					Display_Output(iTargetTemp, 1, 255);
				}
			}
			// heating controller
			ucStep = TempController(iActualTemp,iTargetTemp, ucStepOld, ucH);
			WS2812_Step(ucStep); // set LED color
			Servo_Step(ucStep); // set servo position
			ucStepOld = ucStep; // save old heating step		
		}
		
		if(Timer1_get_1sState() == TIMER_TRIGGERED)
		{
			sSendFrame.EIDE_Bit = STANDARD_ID;
			sSendFrame.RTR_Bit = DATA_FRAME;
			sSendFrame.ulID = 0x400;
			sSendFrame.ucLength = 3;
			sSendFrame.ucData[0] = (char)(iActualTemp/10);
			sSendFrame.ucData[1] = (unsigned char)(iActualTemp%10);
			sSendFrame.ucData[2] = ucStep;
			MCP2515_Send_Message(MCP2515_1, &sSendFrame);
		}
		
		// set target temperature
		//if(Timer1_get_10msState() == TIMER_TRIGGERED)
		//{
		//unsigned char ucKeyStatus = keys_get_state();	//Asks for key status
		//switch(ucCANStatus)
		//{
		//case CAN_NOT_RECEIVED:
		//Display_Output(iTargetTemp, 1, ucCANStatus);
		////Test if changes wished--------------
		//if (ucKeyStatus == S2_PRESSED)
		//{
		//iTargetTemp += 5;
		//}
		//else if (ucKeyStatus == S1_PRESSED)
		//{
		//iTargetTemp -= 5;
		//}
		//break;
		//
		//case CAN_RECEIVED:
		//
		//break;
		//}
		//}
	}
	
}

//void CAN_Filter_Init(void)
//{
	//sFilter.Rec_Buff0_Rollover = ROLLOVER_ON;
	//sFilter.RecBuff_ID[0] = STANDARD_ID;
	//sFilter.RecBuff_ID[1] = STANDARD_ID;
	//sFilter.Filter_RecBuff[0] = FILTER_ON;
	//sFilter.Filter_RecBuff[1] = FILTER_ON;
	//for(uint8_t ucI = 0; ucI < 6; ucI++)
	//{
		//sFilter.ulRecBuff_Filter[ucI] = ulReceiveFilter[ucI];
	//}
	//for(uint8_t ucI = 0; ucI < 2; ucI++)
	//{
		//sFilter.ulRecBuff_Mask[ucI] = ulReceiveMask[ucI];
	//}
//}
