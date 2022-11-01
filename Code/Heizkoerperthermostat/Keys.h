/*
 * Keys.h
 *
 * Created: 08.04.2022 10:00:02
 *  Author: Moritz Hoehnel
 */ 


#ifndef KEYS_H_
#define KEYS_H_

//includes
#include <avr/io.h>

//defines (Zu beginn der Programmausführung)
#define S1_PRESSED			1
#define S2_PRESSED			2
#define S3_PRESSED			3
#define S4_PRESSED			4
#define KEYS_NOT_PRESSED	0

//Deklaration der Funktionen
void keys_Init(void);
unsigned char keys_get_state(void);


#endif /* KEYS_H_ */