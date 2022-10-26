/*
 * TWI.h
 *
 * Created: 26.10.2022 22:31:56
 *  Author: Moritz
 */ 


#ifndef TWI_H_
#define TWI_H_

//Includes
#include <avr/io.h>

//Defines
#define F_CPU 20000000UL //Clock des Board-uC in Hz
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

typedef struct
{
	uint8_t ucAddress; //für den Slave: private oder allgemeine Adresse
	uint8_t ucTWIData[4];
	uint8_t ucTWIDataLength;
}twi_frame;

//Deklaration of functions
void TWI_Init(void);
void TWI_Master_Start(void);
void TWI_Master_Transmit(unsigned char ucdata);
uint8_t TWI_Send_Frame(uint8_t ucdevice_address, twi_frame *sframe);
unsigned char TWI_Master_Read_Ack(void);
unsigned char TWI_Master_Read_NAck(void);






#endif /* TWI_H_ */