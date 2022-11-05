/*
 * Init1.h
 *
 * Created: 26.10.2022 20:38:01
 *  Author: Moritz
 */ 


#ifndef INIT1_H_
#define INIT1_H_


//Includes
#include <avr/io.h>


//Defines

//Deklaration of funtions
void GeneralInit(void);

//void CAN_Filter_Init(void);
//void ShowMessage(can_frame *showFrame);
void HexToAscii(unsigned char input, unsigned char* output);



#endif /* INIT1_H_ */