/*
 * LED.c
 *
 * Created: 30.03.2022 12:27:38
 *  Author: Moritz Hoehnel
 * Comment: GREEN LED switch off, clashing with servo
 */ 


//include
#include "LED.h"

//Dekalaration der globalen Variablen im Modul LED

//Defintionen der Fnen
void LED_Init(void){
	//Beschreibung:		Initialisierung der LEDs
	//Aufrufparameter:	None    
	//Rückgabewert:		None
	//Version:			1
	//Datum:			220330
	//Autor:			mh
	//Status:			ok
	//--------------------------------------------
	//DDRD |= 1 << PD6;
	DDRB |= 1 << PB2;
}


//void LED_gn_on(void){
		////Beschreibung:		Einschalten grüne LED
		////Aufrufparameter:	None
		////Rückgabewert:		None
		////Version:			1
		////Datum:			220330
		////Autor:			mh
		////Status:			ok
		////--------------------------------------------
	//PORTD |= 1 << PD6;
//}


//void LED_gn_off(void){
		////Beschreibung:		Ausschalten grüne LED
		////Aufrufparameter:	None
		////Rückgabewert:		None
		////Version:			1
		////Datum:			220330
		////Autor:			mh
		////Status:			ok
		////--------------------------------------------
	//PORTD &= ~(1 << PD6);
//}

void LED_rd_on(void){
	//Beschreibung:		Einschalten rote LED
	//Aufrufparameter:	None
	//Rückgabewert:		None
	//Version:			1
	//Datum:			220330
	//Autor:			mh
	//Status:			ok
	//--------------------------------------------
	PORTB |= 1 << PB2;
}


void LED_rd_off(void){
	//Beschreibung:		Ausschalten rote LED
	//Aufrufparameter:	None
	//Rückgabewert:		None
	//Version:			1
	//Datum:			220330
	//Autor:			mh
	//Status:			ok
	//--------------------------------------------
	PORTB &= ~(1 << PB2);
}



void LED_rd_toggle(void){
		//Beschreibung:		Togglen rote LED
		//Aufrufparameter:	None
		//Rückgabewert:		None
		//Version:			1
		//Datum:			220330
		//Autor:			mh
		//Status:			ok
		//--------------------------------------------
	PORTB ^= 1 << PB2;
}

//void LED_gn_toggle(void){
		////Beschreibung:		Togglen grüne LED
		////Aufrufparameter:	None
		////Rückgabewert:		None
		////Version:			1
		////Datum:			220330
		////Autor:			mh
		////Status:			ok
		////--------------------------------------------
	//PORTD ^= 1 << PD6;
//}
//
	//
