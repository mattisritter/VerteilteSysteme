/**
 * @file LED.c
 * @brief C file to control LEDs
 * @section Comment
 * GREEN LED switched off, clashing with servo
 * @date 30.03.2022 12:17:13
 * @author Hoehnel and Ritter
 */ 


//Includes======================================================
#include "LED.h"

//Variables=====================================================

//Definition of functions=======================================
/** @brief Initialization of LEDs
* @param[in] None
* @return None
*/
//Version:			1
//Date:			220330
//Autor:			mh
//Status:			ok
//--------------------------------------------
void LED_Init(void){
	//DDRD |= 1 << PD6;
	DDRB |= 1 << PB2;
}

/** @brief Switch on red LED
* @param[in] None
* @return None
*/
//Version:			1
//Date:			220330
//Autor:			mh
//Status:			ok
//--------------------------------------------
void LED_rd_on(void){
	PORTB |= 1 << PB2;
}

/** @brief Switch off red LED
* @param[in] None
* @return None
*/
//Version:			1
//Date:			220330
//Autor:			mh
//Status:			ok
//--------------------------------------------
void LED_rd_off(void){
	PORTB &= ~(1 << PB2);
}


/** @brief Toggle red LED
* @param[in] None
* @return None
*/
//Version:			1
//Date:			220330
//Autor:			mh
//Status:			ok
//--------------------------------------------
void LED_rd_toggle(void){
	PORTB ^= 1 << PB2;
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
