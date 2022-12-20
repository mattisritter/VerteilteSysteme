/**
 * @file Controller.c
 * @brief C file to implement Mulitple position controller with hysteresis
 * @date 05.11.2022 13:01:33
 * @author Hoehnel and Ritter
 * 
 */ 

//Includes======================================================
#include "Controller.h"

//Variables=====================================================


//Definition of functions=======================================
/** @brief Initialization of keys
* @param[in] int actualTemp: Measured temperature
* @param[in] int targetTemp: target Temp: Wished temperature
* @param[in] unsigned_char stepOld: Previous heating step
* @param[in] unsigned_char ucHysteresis: Variable to implement hysteresis
* @return [unsigned char] Value which heating should be at
*/
//Date :			4.11.22
//Author:			Moritz
//Source:
//Status:			released
//--------------------------------
unsigned char TempController(int actualTemp, int targetTemp, unsigned char stepOld, unsigned char ucHysteresis)
{
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