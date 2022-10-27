/*
 * LED.c
 *
 * Created: 12.04.2020 14:19:03
 *  Author: Meroth
 */ 
 #include <avr/io.h>
 #include "LED.h"

 void LEDInit()
 {
	DDRD |= (1<<DDD6);
	DDRB |= (1<<DDB2);
 }
 void LEDOn(char color)
 {
	if (color==GREEN)
	{
		PORTB |= (1<<PB2);
	}
	else if (color==RED)
	{
		PORTD |= (1<<PD6);
	}
 }
 void LEDOff(char color)
 {
 	if (color==GREEN)
 	{
	 	PORTB &= ~(1<<PB2);
 	}
 	else if (color==RED)
 	{
	 	PORTD &= ~(1<<PD6);
 	}
 }
 void LEDToggle(char color)
 {
 	if (color==GREEN)
 	{
	 	PORTB ^= (1<<PB2);
 	}
 	else if (color==RED)
 	{
	 	PORTD ^= (1<<PD6);
 	}
 }