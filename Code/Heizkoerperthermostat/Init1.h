-/**
 * @file Init1.h
 * @brief Include file to initiate modules
 * @date 26.10.2022 20:38:01
 * @author Hoehnel and Ritter
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