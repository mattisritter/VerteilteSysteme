/*
 * Servo.h
 *
 * Created: 24.10.2022 11:17:37
 *  Author: Mattis Ritter
 */ 


#ifndef SERVO_H_
#define SERVO_H_

// include's
#include <avr/io.h>

//declaration of variable's

	
// definition of function's
void Servo_Init(void);
void Servo_Step(unsigned char ucPosition);


#endif /* SERVO_H_ */