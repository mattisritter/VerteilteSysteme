/**
 * @file Keys.h
 * @brief Include file to Read Keys 
 * @date 08.04.2022 10:00:02
 * @author Hoehnel and Ritter  
 */ 


#ifndef KEYS_H_
#define KEYS_H_

//Includes
#include <avr/io.h>

//Defines (Zu beginn der Programmausführung)
#define S1_PRESSED			1  /**<define that 1 is key 1*/
#define S2_PRESSED			2  /**<define that 2 is key 2*/
#define S3_PRESSED			3  /**<define that 3 is key 3*/
//#define S4_PRESSED			4
#define KEYS_NOT_PRESSED	0  /**<define that 0 is no key pressed*/

//Declaration of funktions
void keys_Init(void);
unsigned char keys_get_state(void);


#endif /* KEYS_H_ */