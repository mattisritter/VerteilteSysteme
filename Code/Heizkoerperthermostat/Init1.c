/*
 * Init1.c
 *
 * Created: 26.10.2022 20:38:13
 *  Author: Moritz
 */ 

//Includes
#include "Init1.h"


//Variables


//Definition of funcitons

void InitThermostat(void){
	//Description:		starts all processes
	//Call_parameter:	void
	//Return_parameter:	void
	//Version:			1
	//Date :			26.1022
	//Autor:			Moritz
	//Source:			-
	//Status:			not testet
	//--------------------------------
	TWI_Init(sTWI_InitParam);
}