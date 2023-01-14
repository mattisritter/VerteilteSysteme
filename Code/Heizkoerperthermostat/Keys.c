/**
 * @file Keys.c
 * @brief C file to Read Keys
 * @section Comment
 * S4 used for LED -> deactivated
 * @date 08.04.2022 10:05:15
 * @author Hoehnel and Ritter
 */ 

//Includes======================================================
#include "Keys.h"

//Variables=====================================================
unsigned char ucS1_old; /**<Variable to store old value of S1*/ 
unsigned char ucS1_new = 0xFF;  /**<Variable for S1*/
unsigned char ucS2_old; /**<Variable to store old value of S2*/
unsigned char ucS2_new = 0xFF;  /**<Variable for S2*/
unsigned char ucS3_old; /**<Variable to store old value of S2*/
unsigned char ucS3_new = 0xFF;  /**<Variable for S3*/
//unsigned char ucS4_old, ucS4_new = 0xFF;


//Definition of functions=======================================
/** @brief Initialization of keys
* @param[in] None
* @return None
* @date 08.04.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------
void keys_Init(void){

	DDRD &= ~((1<<PD5) | (1<<PD4) | (1<<PD3) /*| (1<<PD2)*/); //Pin 3,2 wird zu eingang
}
//==============================================================
/** @brief Gets current state of keys
* @param[in] None
* @return [unsigned char] says which key is pressed
* @date 08.04.2022
* @author Hoehnel and Ritter
* @version 3.0
*/
//----------------------------------------------------
unsigned char keys_get_state(void){
	ucS1_old = ucS1_new;
	ucS2_old = ucS2_new;
	ucS3_old = ucS3_new;
	//ucS4_old = ucS4_new;
	
	ucS1_new = PIND & (1<<PD5);
	ucS2_new = PIND & (1<<PD4);
	ucS3_new = PIND & (1<<PD3);
	//ucS4_new = PIND & (1<<PD2);
	
	if((!ucS1_new && ucS1_old)){
		return S1_PRESSED;
	}
	
	if((!ucS2_new && ucS2_old)){
		 return S2_PRESSED;
	}
	
	if((!ucS3_new && ucS3_old)){
		return S3_PRESSED;
	}
	
	//if((!ucS4_new && ucS4_old)){
		//return S4_PRESSED;
	//}
	
	return KEYS_NOT_PRESSED;
}