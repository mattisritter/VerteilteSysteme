/**
 * @file Timer1.h
 * @brief Include file to initiate Timer 1
 * @date 27.04.2022 11:59:47
 * @author Hoehnel and Ritter 
 */ 


#ifndef TIMER1_H_
#define TIMER1_H_

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>


//Defines
#define TIMER_TRIGGERED 1
#define TIMER_RUNNING 0

//Declaration of functions
void Timer1_Init(void);
unsigned char Timer1_get_10msState(void);
unsigned char Timer1_get_1sState(void);
unsigned char Timer1_get_100msState(void);

#endif /* TIMER1_H_ */