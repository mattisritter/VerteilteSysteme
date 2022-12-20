/**
 * @file Servo.c
 * @brief C file to controll servomotor
 * @date 24.10.2022 11:17:37
 * @author Hoehnel and Ritter
 */ 

//Includes======================================================
#include "Servo.h"

//Variables=====================================================
unsigned char ucServoPosition[6] = {44, 38, 30, 23, 16, 10}; /**<Defines six positions for servomotor*/

//Definition of functions=======================================
/** @brief Initialize servo by using Fast-PWM mode from Timer0
* @param[in] None
* @return None
*/
//Version:			1
//Date :			31.10.2022
//Author:			Mattis Ritter
//Source:
//Status:			released
//--------------------------------
void Servo_Init(void)
{
	DDRD |= (1 << DDD6); //Port B Bit 1 – PWM-Ausgang
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); //Fast-PWM, Clear OC0A on compare match, set OC0A at BOTTOM, (non-inverting mode)
	TCCR0B = ( 1<< CS00) | ( 1<< CS02); //Prescaler 1024, fPWM = 18.432MHz/1024*256 = 79.3125Hz, TPWM = 14.222ms
	OCR0A = ucServoPosition[0]; //Pulsweite auf 2.5ms setzen --> Stellung 0
}

/** @brief Set servo position
* @param[in] unsigned_char ucPosition: Heater position from 0 to 5
* @return None
*/
//Version:			1
//Date :			31.10.2022
//Author:			Mattis Ritter
//Source:
//Status:			released
//--------------------------------
void Servo_Step(unsigned char ucPosition)
{
	OCR0A = ucServoPosition[ucPosition];
}