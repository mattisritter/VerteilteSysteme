/*
 * LED.h
 *
 * Created: 12.04.2020 14:19:23
 *  Author: Meroth
 */ 


#ifndef LED_H_
#define LED_H_

#define RED 10
#define GREEN 20

void LEDInit();
void LEDOn(char color);
void LEDOff(char color);
void LEDToggle(char color);



#endif /* LED_H_ */