/*
 * main.h
 *
 * Created: 26.10.2022 19:54:57
 *  Author: Moritz
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
//#include "Timer2.h"
#include "TMP75.h"
#include "TWI_ATMEGA.h"
#include "Keys.h"
#include "Timer1.h"

//Defines
#define CAN_NOT_RECEIVED 101
#define CAN_RECEIVED 111

//Definition von ws2812_1
WS2812_pin WS2812_1 =	{/*DDR register*/	&DDRD,
						/*PORT register*/	&PORTD,
						/*Pin*/				PD2};
						
//
MCP2515_pins  MCP2515_1 = {{/*CS_DDR*/	&DDRB,
						    /*CS_PORT*/		&PORTB,
							/*CS_pin*/		PB3,
							/*CS_state*/	ON}};
	
	
////Deklaration of global variables
//can_frame sSendFrame, sRecFrame;
//can_filter sFilter;
//
////Definition of global variables
//MCP2515_pins  MCP2515_1 = {{/*CS_DDR*/	&DDRB,
	///*CS_PORT*/		&PORTB,
	///*CS_pin*/		PB2,
///*CS_state*/	ON}};
//
//uint32_t ulReceiveFilter[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
//uint32_t ulReceiveMask[2] = {0x7FF, 0x7FF};

//unsigned char ucTimer = 0;


#endif /* MAIN_H_ */