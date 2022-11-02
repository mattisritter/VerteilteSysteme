/*
 * Timer2.c
 *
 * Created: 26.04.2020 12:07:40
 *  Author: Meroth
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include "Timer2.h"
// #include "LED.h"

 // global variables
 unsigned char uc10MsFlag=0;
 unsigned char uc100MsFlag=0;
 unsigned char uc10MsCnt=0;
 unsigned char uc100MsCnt=0;
 unsigned char uc1sFlag=0;

unsigned char Check10msFlag()
{
	if (uc10MsFlag)
	{
		uc10MsFlag=0;
		return TRIGGERED;
	}
	else return IDLE;
}

unsigned char Check100msFlag()
{
	if (uc100MsFlag)
	{
		uc100MsFlag=0;
		return TRIGGERED;
	}
	else return IDLE;
}

unsigned char Check1sFlag()
{
	if (uc1sFlag)
	{
		uc1sFlag=0;
		return TRIGGERED;
	}
	else return IDLE;
}

void InitTimer2CTC(void)
{
	TCCR2A = (1 << WGM21);
	TCCR2B = (1<<CS21) | (1<<CS22) | (1<<CS20);
	OCR2A = 179;
	TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
	uc10MsFlag=1;
	uc10MsCnt++;

	if (uc10MsCnt==10)
	{
		uc100MsFlag=1;
		uc100MsCnt++;
		uc10MsCnt=0;
		if (uc100MsCnt==10)
		{
			uc1sFlag=1;
			uc100MsCnt=0;
		}
	}
}