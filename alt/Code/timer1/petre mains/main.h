/*
 * main.h
 *
 * Created: 16.11.2021 14:19:05
 *  Author: PS_HF
 */ 


#ifndef MAIN_H_
#define MAIN_H_

//include's
#include <avr/io.h>
#include "display_funktionen.h"
#include "keys.h"
#include "LED.h"
#include "MCP2515_HHN.h"
#include "SPI.h"
#include "Timer1.h"

MCP2515_pins  MCP2515_1 = {{/*CS_DDR*/	&DDRB,
							/*CS_PORT*/		&PORTB,
							/*CS_pin*/		PB2,
							/*CS_state*/	ON}};

void Init(void); 
void CAN_Filter_Init(void);

can_frame sSendFrame, sRecFrame;
can_filter sFilter;

uint32_t ulReceiveFilter[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint32_t ulReceiveMask[2] = {0x7FF, 0x7FF};

unsigned char ucTimer = 0;

#endif /* MAIN_H_ */