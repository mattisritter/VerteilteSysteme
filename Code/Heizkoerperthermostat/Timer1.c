/*
 * Timer0.c
 *
 * Created: 27.04.2022 12:02:28
 *  Author: Moritz Hoehnel
 */ 
//
//includes
#include "Timer1.h"

//Variablen
unsigned char ucFlag10ms = 0;
unsigned char ucCNt_1s = 0;
unsigned char ucFlag_1s = 0;
unsigned char ucCNt_100ms = 0;
unsigned char ucFlag_100ms = 0;

//Definition of functions
void Timer1_Init(){
	//Beschreibung:		Initialization of timer0 mit 10ms
	//Aufrufparameter:	void
	//Rückgabewert:		void
	//Version:			1
	//Datum:			220427
	//Autor:			mh
	//Status:			ok
	//-------------------------------------------------------------------
	TCCR1A |= (1<<WGM11);					//CTC-Modus
	TCCR1B |= ((1<<CS10) | (1<<CS12));		//Prescaler=1024
	OCR0A = 179;							//10ms @f=18.432MHz
	TIMSK1 |= 1<<OCIE1A;					//Freigabe des Timerinterrupt
	sei();									//alle Interrupts freigegeben
}

ISR(TIMER1_COMPA_vect){
	//Beschreibung:		Interrupt service Routine getriggered by timer
	//Aufrufparameter:	Rücksetztzeitpunkt des Counters
	//Rückgabewert:		-
	//Version:			2
	//Datum:			220427
	//Autor:			mh
	//Status:			ok
	//-------------------------------------------------------------------
	ucFlag10ms = 1;			//wird jede 10ms gesetzt
	ucCNt_1s++;
	ucCNt_100ms++;
	
	if(ucCNt_1s == 100){
		ucCNt_1s = 0;
		ucFlag_1s = 1;		//wird jede sekunde gesetzt
	}
	
	if(ucCNt_100ms == 10){
		ucCNt_100ms = 0;
		ucFlag_100ms = 1;		//wird jede sekunde gesetzt
	}
}

unsigned char Timer1_get_10msState(){
	//Beschreibung:		Beantwortet ob 10ms verstrichen sind
	//Aufrufparameter:	none
	//Rückgabewert:		unsigned char
	//Version:			1
	//Datum:			220427
	//Autor:			mh
	//Status:			ok
	//-------------------------------------------------------------------
	if(ucFlag10ms == 1){
		ucFlag10ms = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
}

unsigned char Timer1_get_1sState(void){
	//Beschreibung:		Beantwortet ob 1s verstrichen ist
	//Aufrufparameter:	none
	//Rückgabewert:		unsigned char
	//Version:			1
	//Datum:			220427
	//Autor:			mh
	//Status:			ok
	//-------------------------------------------------------------------
	if(ucFlag_1s == 1){
		ucFlag_1s = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
	
}

unsigned char Timer1_get_100msState(void){
	//Beschreibung:		Beantwortet ob 100ms verstrichen sind
	//Aufrufparameter:	none
	//Rückgabewert:		unsigned char
	//Version:			1
	//Datum:			220531
	//Autor:			mh
	//Status:			ok
	//-------------------------------------------------------------------
	if(ucFlag_100ms == 1){
		ucFlag_100ms = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
}