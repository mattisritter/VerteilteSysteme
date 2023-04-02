/**
 * @file Timer1.c
 * @brief C file to initiate Timer 1
 * @date 27.04.2022 11:59:47
 * @author Hoehnel and Ritter
 */ 

//Includes======================================================
#include "Timer1.h"

//Variables=====================================================
unsigned char ucFlag10ms = 0; /**<Flag that becomes 1 every 10 ms*/
unsigned char ucCNt_1s = 0; /**<Counts to 1000 ms*/
unsigned char ucFlag_1s = 0; /**<Flag that becomes 1 every 1000 ms*/
unsigned char ucCNt_100ms = 0; /**<Counts to 100 ms*/
unsigned char ucFlag_100ms = 0; /**<Flag that becomes 1 every 100 ms*/

//Definition of functions=======================================
/** @brief Initialization of timer1 with 10ms
* @param[in] None
* @return None
* @date 27.04.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//-------------------------------------------------------------------
void Timer1_Init(){
	TCCR1A |= (1<<WGM11);					//CTC-Modus
	TCCR1B |= ((1<<CS10) | (1<<CS12));		//Prescaler=1024
	OCR0A = 179;							//10ms @f=18.432MHz
	TIMSK1 |= 1<<OCIE1A;					//Freigabe des Timerinterrupt
	sei();									//alle Interrupts freigegeben
}
/** @brief Interrupt service Routine triggered by timer
* @param[in] vector Reset point of counter
* @return None
* @date 22.04.2022
* @author Hoehnel and Ritter
* @version 2.0
*/
//-------------------------------------------------------------------
ISR(TIMER1_COMPA_vect){
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

/** @brief Says if 10ms have been passed
* @param[in] None
* @return [unsigned char] {TIMER_RUNNING, TIMER_TRIGGERED}
* @date 27.04.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//-------------------------------------------------------------------
unsigned char Timer1_get_10msState(){
	if(ucFlag10ms == 1){
		ucFlag10ms = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
}

/** @brief Says if 1000ms have been passed
* @param[in] None
* @return [unsigned char] {TIMER_RUNNING, TIMER_TRIGGERED}
* @date 27.04.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//-------------------------------------------------------------------
unsigned char Timer1_get_1sState(void){
	if(ucFlag_1s == 1){
		ucFlag_1s = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
}

/** @brief Says if 100ms have been passed
* @param[in] None
* @return [unsigned char] {TIMER_RUNNING, TIMER_TRIGGERED}
* @date 27.04.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//-------------------------------------------------------------------
unsigned char Timer1_get_100msState(void){
	if(ucFlag_100ms == 1){
		ucFlag_100ms = 0;
		return TIMER_TRIGGERED;
	}
	return TIMER_RUNNING;
}