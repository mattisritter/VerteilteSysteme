/*
 * Servo.c
 *
 * Created: 24.10.2022 11:17:59
 *  Author: Mattis Ritter
 */ 

#include "Servo.h"

unsigned char ucServoPosition[6] = {44, 38, 30, 23, 16, 10};

void Servo_Init(void)

{
	//Description:		initialize servo by using Fast-PWM mode from Timer0
	//Call_parameter:	void
	//Return_parameter:	void
	//Version:			1
	//Date :			31.10.2022
	//Author:			Mattis Ritter
	//Source:
	//Status:			released
	//--------------------------------
	DDRD |= (1 << DDD6); //Port B Bit 1 – PWM-Ausgang
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); //Fast-PWM, Clear OC0A on compare match, set OC0A at BOTTOM, (non-inverting mode)
	TCCR0B = ( 1<< CS00) | ( 1<< CS02); //Prescaler 1024, fPWM = 18.432MHz/1024*256 = 79.3125Hz, TPWM = 14.222ms
	OCR0A = ucServoPosition[0]; //Pulsweite auf 2.5ms setzen --> Stellung 0
}

void Servo_Step(unsigned char ucPosition)
{
	//Description:		sets servo position
	//Call_parameter:	ucPosition: heater position from 0 to 5
	//Return_parameter:	void
	//Version:			1
	//Date :			31.10.2022
	//Author:			Mattis Ritter
	//Source:
	//Status:			released
	//--------------------------------
	OCR0A = ucServoPosition[ucPosition];
}