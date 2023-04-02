///*
 //* TWI.c
 //*
 //* Created: 26.10.2022 22:32:11
 //*  Author: Moritz
 //*/ 
//
///*
 //* TMP75.c
 //*
 //* Created: 26.10.2022 20:07:28
 //*  Author: Moritz
 //*/ 
//
//
////Includes
//#include "TWI.h"
//
////Variables
//
//
////Definition of funcitons
//
//void TWI_Master_Start(void)
//{
	////Description:		starts twi communication
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	////Start: SDA-Leitung vor der SCL-Leitung auf Low setzen
	//while(!(TWCR & (1 << TWINT))); //warten bis SDA gesetzt wird
//}
//void TWI_Init()
//{
	////Description:		Init as twi master
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//TWI_InitParam sTWI_InitParam = {/*ucDevice*/ TWI_MASTER,
		///*ucTwiClock*/ TWI_MASTER_CLOCK,
		///*ucSlaveAddress*/ 0x80,
		///*ucGenAddress*/ CGA_DISABLE,
	///*ucAddressMask*/ 0x00};
//}
//
//void TWI_Master_Transmit(unsigned char ucdata)
//{
	////Description:		twi transmit
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//TWDR = ucdata;
	//TWCR = (1 << TWINT) | (1 << TWEN);
	//while(!(TWCR & (1 << TWINT)));
	///*die Hardware setzt das Bit TWINT wenn das Byte ucdata vollständig
	//übertragen wurde*/
//}
//
//uint8_t TWI_Send_Frame(uint8_t ucdevice_address, twi_frame *sframe)
//{
	////Description:		twi sending
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//uint8_t ucDeviceAddress;
	//ucDeviceAddress = (ucdevice_address << 1);
	//ucDeviceAddress |= TWI_WRITE; //Write-Modus
	//TWI_Master_Start(); //Start
	//if((TWI_STATUS_REGISTER) != TWI_START) return TWI_ERROR;
	//// Device Adresse senden
	//TWI_Master_Transmit(ucDeviceAddress);
	//if((TWI_STATUS_REGISTER) != TWI_MT_SLA_ACK)
	//{
		//TWI_Master_Stop();
		//return TWI_ERROR;
	//}
	//for(uint8_t i = 0; i < sframe->ucTWIDataLength; i++)
	//{
		//TWI_Master_Transmit(sframe->ucTWIData[i]);
		//if(TWI_STATUS_REGISTER == TWI_MT_DATA_NACK)
		//{
			//TWI_Master_Stop();
			//return TWI_NACK;
		//}
	//}
	//TWI_Master_Stop(); // Stop
	//return TWI_OK;
//}
//
//
//unsigned char TWI_Master_Read_Ack(void)
//{
	////Description:		Master respondes with ACK to recieved Byte
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	//while(!(TWCR & (1 << TWINT)));
	//return TWDR;
//}
//
//unsigned char TWI_Master_Read_NAck(void)
//{
	////Description:		Master respondes with NACK to recieved Byte
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			220408
	////Autor:			Meroth, Sora
	////Source:			Sensornetzwerke In Theorie Und Praxis
	////Status:			not testet
	////--------------------------------
	//TWCR = (1<<TWINT)|(1<<TWEN);
	//while(!(TWCR & (1 << TWINT)));
	//return TWDR;
//}
