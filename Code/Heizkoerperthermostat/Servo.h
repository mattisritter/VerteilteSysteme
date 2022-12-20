/**
 * @file Servo.h
 * @brief Include file to controll servomotor
 * @date 24.10.2022 11:17:37
 * @author Hoehnel and Ritter
 */ 


#ifndef SERVO_H_
#define SERVO_H_

//Includes
#include <avr/io.h>

//Defines

	
//Declaration of funktions
void Servo_Init(void);
void Servo_Step(unsigned char ucPosition);


#endif /* SERVO_H_ */