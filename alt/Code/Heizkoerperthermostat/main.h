/**
 * @file main.h
 * @brief Include file for main
 * @date 26.10.2022 19:54:57
 * @author Hoehnel and Ritter
 */ 

#ifndef MAIN_H_
#define MAIN_H_

//Includes
#include <avr/io.h>


#include "Init1.h"
#include "Servo.h"
#include "WS2812.h"
#include "display_funktionen.h"
#include "LED.h"
#include "MCP2515_HHN.h"
#include "Servo.h"
#include "SPI.h"
#include "TMP75.h"
#include "TWI_ATMEGA.h"
#include "Keys.h"
#include "Timer1.h"

//Defines
#define CAN_NOT_RECEIVED 0 /**<define that 0, no message received via can*/
#define CAN_RECEIVED 1 /**<define that 1, message received via can*/

//Definition von ws2812_1
WS2812_pin WS2812_1 =	{/*DDR register*/	&DDRD,
						/*PORT register*/	&PORTD,
						/*Pin*/				PD2}; /**<define that WS2812 is using PD2 of PORT D*/
						
//
MCP2515_pins  MCP2515_1 = {{/*CS_DDR*/		&DDRD,
						    /*CS_PORT*/		&PORTD,
							/*CS_pin*/		PD0,
							/*CS_state*/	ON}}; /**<define that MCP2515 is using PD0 of PORT D*/


#endif /* MAIN_H_ */