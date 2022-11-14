/*
 * TMP75.h
 *
 * Created: 26.10.2022 20:07:03
 *  Author: Moritz
 * Communication with TMP75 via I^2C
 */ 


#ifndef TMP75_H_
#define TMP75_H_

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_ATMEGA.h"
#include "display_funktionen.h"

//Defines
#define TMP75_DEVICE_TYPE_ADDRESS 0x00

//Deklaration of functions

uint8_t TMP75_Read_Temperature(void);
int TMP75_Get_Temperature(void);
//void Disp_PrintTemperature(int ui2print);
//void Disp_PrintTarget(int ui2print, unsigned char CANStatus);
void Display_Output(int iTemp2print, unsigned char ucLine, unsigned char ucCAN);


#endif /* TMP75_H_ */