/*
 * TMP75.h
 *
 * Created: 26.10.2022 20:07:03
 *  Author: Moritz
 * Communication with TMP75 via I^2C
 */ 


#ifndef TMP75_H_
#define TMP75_H_

//Includes
#include "TMP75.h"

//Defines
#define F_CPU 20000000UL //Clock des Board-?C in Hz
#define TWI_SCL_FREQ 400000UL /*gewünschte TWI-Taktfrequenz in Hz; wird entsprechend der konkreten Anwendung geändert*/
#define TWI_MASTER_CLOCK (((F_CPU / TWI_SCL_FREQ) - 16 ) / 2 +1) //Wert des TWBR-Registers
#define TWI_STATUS_REGISTER (TWSR &0xF8)

typedef struct{	//configuration TWI
	uint8_t ucDevice; //TWI_MASTER oder TWI_SLAVE
	uint8_t ucTwiClock; //Übertragungsrate für den Master
	uint8_t ucSlaveAddress; //Slave-Adresse
	uint8_t ucGenAddress; //enable/disable general call
	uint8_t ucAddressMask; //ev. Adressmaske für den Slave
} TWI_InitParam;

//Deklaration of functions
<<<<<<< Updated upstream
void TWI_Init(TWI_InitParam sinit_param);
=======
>>>>>>> Stashed changes
uint8_t TMP75_Read_Temperature(uint8_t ucdevice_address, uint8_tuctemp2read);
int TMP75_Get_Temperature(void);



#endif /* TMP75_H_ */