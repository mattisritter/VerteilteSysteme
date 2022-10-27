/*
 * Timer2.h
 *
 * Created: 26.04.2020 12:07:25
 *  Author: Meroth
 */ 


#ifndef TIMER2_H_
#define TIMER2_H_

void InitTimer2CTC(void);
unsigned char Check10msFlag();
unsigned char Check100msFlag();
unsigned char Check1sFlag();

#define TRIGGERED 1
#define IDLE 0

#endif /* TIMER2_H_ */