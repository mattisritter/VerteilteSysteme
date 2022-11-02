/*
 * LED.h
 *
 * Created: 30.03.2022 12:17:13
 *  Author: Moritz Hoehnel
  Funktion: Steuerung LED
 */ 


#ifndef LED_H_
#define LED_H_

//include
#include <avr/io.h>



//define


//Deklaration der Fnen
void LED_Init(void);
//void LED_gn_on(void);
//void LED_gn_off(void);
void LED_rd_on(void);
void LED_rd_off(void);
void LED_rd_toggle(void);
//void LED_gn_toggle(void);



#endif /* LED_H_ */