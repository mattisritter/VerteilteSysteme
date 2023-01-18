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
unsigned char ucFlagCanReceived = CAN_NOT_RECEIVED; /**<Flag that Can communication has startet*/

can_frame sSendFrame;
can_frame sRecFrame;

//Main function=====================================================
/** @brief Main function
* @param[in] None
* @return None
* @date 14.01.2023
* @author Hoehnel and Ritter
* @version 1.0
*/
int main(void)
{
	GerneralInit(); //Execute Init functioncs of modules
	
	MCP2515_Init(MCP2515_1, BAUDRATE_250_KBPS);//CAN

	// Measurement for initializing first step
	TMP75_Read_Temperature();
	iActualTemp = TMP75_Get_Temperature(); //Asks for temp value, inital measurement that should not be considered 
	ucStep = TempController(iActualTemp,iTargetTemp, ucStepOld, 0);	//Value of Heating-Step without hysteresis
	ucStepOld = ucStep; //Old heating step
	
	while (1)
	{
		// measure actual temperature
		if (Timer1_get_100msState() == TIMER_TRIGGERED)	/*Prevent tmp75 from overheating, only start every 100ms*/
		{
			TMP75_Read_Temperature();							
			iActualTemp = TMP75_Get_Temperature();	//Asks for temp value;
			Display_Output(iActualTemp, 0, ucFlagCanReceived);
			
			if(MCP2515_Check_Message(MCP2515_1, &sRecFrame) == MESSAGE_RECEIVED)
			{
				// check if a CAN message was received, if yes, save in sRecFrame
				ucFlagCanReceived = CAN_RECEIVED;
				if(sRecFrame.ulID == 0x401)// a message with the ID 0x401 was received
				{
					iTargetTemp = (int)(sRecFrame.ucData[0]*10);
					Display_Output(iTargetTemp, 1, ucFlagCanReceived);
				}
			}
			// heating controller
			ucStep = TempController(iActualTemp, iTargetTemp, ucStepOld, ucH);
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
		
		//set target temperature with keys
		if((Timer1_get_10msState() == TIMER_TRIGGERED) && (ucFlagCanReceived == CAN_NOT_RECEIVED))
		{
			unsigned char ucKeyStatus = keys_get_state();	//Asks for key status
		
			if (ucKeyStatus == S2_PRESSED)
			{
				iTargetTemp += 5;
			}
			else if (ucKeyStatus == S1_PRESSED)
			{
				iTargetTemp -= 5;
			}
			
			Display_Output(iTargetTemp, 1, ucFlagCanReceived);
		}
	}	
}
