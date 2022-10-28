/*
 * WS2812.h
 *
 * Created: 30.04.2018 11:46:02
 *  Author: Petre Sora
 * Disclaimer: This code sample is just a prototype. It is in no way suitable
 * for safe and reliable code, since the authors have intentionally abstained
 * from error or plausibility checks.
 * Therefore, the code examples must not be used in military or commercial or
 * safety-related products. Running this software can potentially be dangerous.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted under license CC-BY-NC-SA provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the above disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the above disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 */ 


#ifndef WS2812_H_
#define WS2812_H_

#include <avr/io.h>


//****************************************************************************************
//in der main wird eine Variable vom Typ WS2812_pin definiert mit dem ansteuernden Pin nach
//dem Beispiel:	WS2812_pin WS2812_1 =	{/*DDR register*/	&DDRD,
//										/*PORT register*/	&PORTD,
//										/*Pin*/				PD2};
//****************************************************************************************
typedef struct
{
	volatile uint8_t *WS2812_DDRReg;
	volatile uint8_t *WS2812_PORTReg;
	uint8_t WS2812_Pin;
}WS2812_pin;

//****************************************************************************************
//Funktionen
//****************************************************************************************
void WS2812_Init(void);

void WS2812_Set_Colour(uint8_t *uccolour_list, uint16_t ucled_number);

#endif /* WS2812_H_ */