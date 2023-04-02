/**
 * @file Init1.c
 * @brief C file to initiate modules
 * @date 26.10.2022 20:38:01
 * @author Hoehnel and Ritter
 */ 

//Includes======================================================
#include "Init1.h"

//Variables=====================================================

//Definition of functions=======================================
/** @brief Execute all inits
* @param[in] None
* @return None
* @date 27.10.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------
void GerneralInit(void){
	Display_Init();
	LED_Init();
	
	WS2812_Init();
	keys_Init();
	Timer1_Init();
	Servo_Init();
	
	//Calculate TWI register clock=========================================================
	unsigned int F_CPU = 20/*[MHz]*/;			//Clock of uC_Board in Hz
	unsigned int TWI_SCL_FREQ = 400/*[kHz]*/;	//needed TWI frequency for fast speed TWI
	unsigned int itwi_clock = (((F_CPU*1000 / TWI_SCL_FREQ)- 16) / 2 + 1);
	unsigned char uctwi_clock = (unsigned char)itwi_clock;
	//=====================================================================================	
	TWI_Master_Init(uctwi_clock);
}

/** @brief Changes hex to ascii
* @param[in] unsigned_char input:  Element to convert 
* @param[out] unsigned_char output: Converted element
* @return None
* @date 27.10.2022
* @author Meroth
* @version 1.0
*/
//--------------------------------
void HexToAscii(unsigned char input, unsigned char* output)
{
	char c;
	c=input & 0x0f;
	if (c<10) output[1]=c+48;
	else output[1]=c+55;
	c=input>>4;
	if (c<10) output[0]=c+48;
	else output[0]=c+55;
}