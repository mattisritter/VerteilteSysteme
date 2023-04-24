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
* @date 30.03.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------------------
void LED_Init(void){
	//DDRD |= 1 << PD6;
	DDRB |= 1 << PB2;
}

/** @brief Switch on red LED
* @param[in] None
* @return None
* @date 30.03.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------------------
void LED_rd_on(void){
	PORTB |= 1 << PB2;
}

/** @brief Switch off red LED
* @param[in] None
* @return None
* @date 30.03.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------------------
void LED_rd_off(void){
	PORTB &= ~(1 << PB2);
}


/** @brief Toggle red LED
* @param[in] None
* @return None
* @date 30.03.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------------------
void LED_rd_toggle(void){
	PORTB ^= 1 << PB2;
}