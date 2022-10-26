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
#include "TWI.h"

//Defines

//Deklaration of functions

uint8_t TMP75_Read_Temperature(uint8_t ucdevice_address, uint8_tuctemp2read);
int TMP75_Get_Temperature(void);



#endif /* TMP75_H_ */