/*
 * Controller.c
 *
 * Created: 05.11.2022 13:01:33
 *  Author: Moritz
 * Mulitple position controller with hysteresis
 */ 

//Includes
#include "Controller.h"

//Variables


//Definition of functions
unsigned char TempController(int actualTemp, int targetTemp, unsigned char stepOld, unsigned char ucHysteresis)
{
	//Description:		Controls heating
	//Call_parameter:	actual temp, target temp, old heating step
	//Return_parameter:	step
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:
	//Status:			released
	//--------------------------------
	int iDelta = targetTemp - actualTemp;
	// Step 0
	if (iDelta < (0 - ucHysteresis) )
	{
		return 0;
	}
	// Step 1
	else if (iDelta >= (0 + ucHysteresis) && iDelta < (20 - ucHysteresis) )
	{
		return 1;
	}
	// Step 2
	else if (iDelta >= (20 + ucHysteresis) && iDelta < (40 - ucHysteresis))
	{
		return 2;
	}
	// Step 3
	else if (iDelta >= (40 + ucHysteresis) && iDelta < (60 - ucHysteresis))
	{
		return 3;
	}
	// Step 4
	else if (iDelta >= (60 + ucHysteresis) && iDelta < (80 - ucHysteresis))
	{
		return 4;
	}
	// Step 5
	else if (iDelta >= (80 + ucHysteresis))
	{
		return 5;
	}
	// default	
	return stepOld;	
}