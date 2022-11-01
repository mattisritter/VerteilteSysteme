/*
 * Timer1.h
 *
 * Created: 27.04.2022 11:59:47
 *  Author: Moritz Hoehnel
 */ 


#ifndef TIMER1_H_
#define TIMER1_H_

//includes
#include <avr/io.h>
#include <avr/interrupt.h>


//defines
#define TIMER_TRIGGERED 1
#define TIMER_RUNNING 0

//declaration of functions
void Timer1_Init(void);

unsigned char Timer1_get_10msState(void);

unsigned char Timer1_get_1sState(void);

unsigned char Timer1_get_100msState(void);





#endif /* TIMER0_H_ */