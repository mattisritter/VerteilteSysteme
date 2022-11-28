/*
 * Controller.c
 *
 * Created: 05.11.2022 13:01:33
 *  Author: Moritz
 */ 

//Includes
#include "Controller.h"

//Variables
//Color definition ===== g  r  b =============================
uint8_t uColor[6][3] = {{7, 0 ,0},  /*green*/
						{5, 2 ,0},  /*brightgreen*/
						{3, 4, 0},  /*yellow*/
						{2, 4, 1},  /*orange*/
						{0, 4, 3},  /*magenta*/
						{0, 7, 0}}; /*red*/
//============================================================

//Definition of funcitons
void TempController(int actualTemp, int targetTemp)
{
	//Description:		Controls heating
	//Call_parameter:	actual temp, target temp
	//Return_parameter:	None
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:
	//Status:			released
	//--------------------------------
	unsigned char ucStep;
	int iDelta = targetTemp - actualTemp;
	if (iDelta <= 20)/*Heating Step 1*/
	{
		ucStep = 0;
	}
	else if (iDelta <= 40)/*Heating Step 2*/
	{
		ucStep = 1;
	}
	else if (iDelta <= 60)/*Heating Step 3*/
	{
		ucStep = 2;
	}
	else if (iDelta <= 80)/*Heating Step 4*/
	{
		ucStep = 3;
	}
	else if (iDelta <= 100)/*Heating Step 5*/
	{
		ucStep = 4;
	}
	else if (iDelta <= 120)/*Heating Step 6*/
	{
		ucStep = 5;	
	}
	WS2812_Set_Colour(uColor[ucStep],2);
	Servo_Set_Position(ucStep);
}