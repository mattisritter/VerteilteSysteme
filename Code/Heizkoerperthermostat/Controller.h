/**
 * @file Controller.h
 * @brief Include File for Software Controller for thermostat
 * @date 05.11.2022 13:01:15
 * @author Hoehnel and Ritter 
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//Includes
#include <avr/io.h>


//Defines


//Deklaration of functions
unsigned char TempController(int actualTemp, int targetTemp, unsigned char stepOld, unsigned char ucHysteresis);


#endif /* CONTROLLER_H_ */