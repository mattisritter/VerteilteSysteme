/*
 * Controller.c
 *
 * Created: 05.11.2022 13:01:33
 *  Author: Moritz
 */ 

//Includes
#include "Controller.h"

//Variables
//Color definition===========================================
uint8_t uGreen[3] = {7/*green*/,0/*red*/,0/*blue*/};
uint8_t uBrightGreen[3] = {5/*green*/,2/*red*/,0/*blue*/};
uint8_t uYellow[3] = {3/*green*/,4/*red*/,0/*blue*/};
uint8_t uOrange[3] = {2/*green*/,4/*red*/,1/*blue*/};
uint8_t uMagenta[3] = {0/*green*/,4/*red*/,3/*blue*/};
uint8_t uRed[3] = {0/*green*/,7/*red*/,0/*blue*/};
//============================================================

//Definition of funcitons
void TempController(int actualTemp, int targetTemp)
{
	//Description:		Controlls heating
	//Call_parameter:	actual temp, target temp
	//Return_parameter:	None
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:
	//Status:			released
	//--------------------------------
	int iDelta = targetTemp - actualTemp;
	if (iDelta <= 20)/*Heating Step 1*/
	{
		WS2812_Set_Colour(uGreen,2);
		Servo_Set_Position(0);
	}
	else if (iDelta <= 40)/*Heating Step 2*/
	{
		WS2812_Set_Colour(uBrightGreen,2);
		Servo_Set_Position(1);
	}
	else if (iDelta <= 60)/*Heating Step 3*/
	{
		WS2812_Set_Colour(uYellow,2);
		Servo_Set_Position(2);
	}
	else if (iDelta <= 80)/*Heating Step 4*/
	{
		WS2812_Set_Colour(uOrange,2);
		Servo_Set_Position(3);
	}
	else if (iDelta <= 100)/*Heating Step 5*/
	{
		WS2812_Set_Colour(uMagenta,2);
		Servo_Set_Position(4);
	}
	else if (iDelta <= 120)/*Heating Step 6*/
	{
		WS2812_Set_Colour(uRed,2);
		Servo_Set_Position(5);	
	}
}