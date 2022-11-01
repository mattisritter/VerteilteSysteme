/*
 * Keys.c
 *
 * Created: 08.04.2022 10:05:15
 *  Author: Moritz Hoehnel
 */ 

//includes======================================================
#include "Keys.h"

//Variablen=====================================================
unsigned char ucS1_old, ucS1_new = 0xFF;
unsigned char ucS2_old, ucS2_new = 0xFF;
unsigned char ucS3_old, ucS3_new = 0xFF;
unsigned char ucS4_old, ucS4_new = 0xFF;


//Definition of functions=======================================
void keys_Init(void){
	//Beschreibung:		Initialization
	//Aufrufparameter:	None
	//Rückgabewert:		None
	//Version:			1
	//Datum:			220408
	//Autor:			mh
	//Status:			ok
	//--------------------------------
	DDRD &= ~((1<<PD5) | (1<<PD4) | (1<<PD3) | (1<<PD2)); //Pin 5,4,3 und 2 wird zu eingang
}
//==============================================================
unsigned char keys_get_state(void){
	//Beschreibung:		Prüft welcher taster gedrück ist
	//Aufrufparameter:	None
	//Rückgabewert:		unsigned char
	//Version:			3
	//Datum:			220408
	//Autor:			mh
	//Status:			ok
	//----------------------------------------------------
	
	ucS1_old = ucS1_new;
	ucS2_old = ucS2_new;
	ucS3_old = ucS3_new;
	ucS4_old = ucS4_new;
	
	ucS1_new = PIND & (1<<PD5);
	ucS2_new = PIND & (1<<PD4);
	ucS3_new = PIND & (1<<PD3);
	ucS4_new = PIND & (1<<PD2);
	
	if((!ucS1_new && ucS1_old)){
		return S1_PRESSED;
	}
	
	if((!ucS2_new && ucS2_old)){
		 return S2_PRESSED;
	}
	
	if((!ucS3_new && ucS3_old)){
		return S3_PRESSED;
	}
	
	if((!ucS4_new && ucS4_old)){
		return S4_PRESSED;
	}
	
	return KEYS_NOT_PRESSED;
}