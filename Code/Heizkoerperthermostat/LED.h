/**
 * @file LED.h
 * @brief Include file to control LEDs
 * @date 30.03.2022 12:17:13
 * @author Hoehnel and Ritter
 */ 


#ifndef LED_H_
#define LED_H_

//Includes
#include <avr/io.h>

//Defines


//Declaration of funktions
void LED_Init(void);
//void LED_gn_on(void);
//void LED_gn_off(void);
void LED_rd_on(void);
void LED_rd_off(void);
void LED_rd_toggle(void);
//void LED_gn_toggle(void);



#endif /* LED_H_ */