/*
 * TMP75.c
 *
 * Created: 26.10.2022 20:07:28
 *  Author: Moritz
 */ 


//Includes
#include "TMP75.h"

//Variables
int iTemperature;

//Definition of funcitons


uint8_t TMP75_Read_Temperature(uint8_t ucdevice_address, uint8_t uctemp2read)
{
	//Description:		Read temperature from TMP75
	//Call_parameter:	ucdevice_address, uctemp2read
	//Return_parameter:	TMP75_Read_Temperature
	//Version:			1
	//Date :			220408
	//Autor:			Meroth, Sora
	//Source:			Sensornetzwerke In Theorie Und Praxis
	//Status:			not testet
	//--------------------------------
	uint8_t ucDeviceAddress, ucTempHigh, ucTempLow;
	//Adresse des TMP75-Temperatursensors bilden
	ucDeviceAddress = (ucdevice_address << 1) | TMP75_DEVICE_TYPE_ADDRESS;
	ucDeviceAddress |= TWI_WRITE;//Write-Modus
	TWI_Master_Start();//Start
	if((TWI_STATUS_REGISTER) != TWI_START) return TWI_ERROR;
	TWI_Master_Transmit(ucDeviceAddress); //Device-Adresse senden
	if((TWI_STATUS_REGISTER) != TWI_MT_SLA_ACK) return TWI_ERROR;
	//die Adresse des gewünschten Temperaturregisters wird gesendet
	TWI_Master_Transmit(uctemp2read);
	if((TWI_STATUS_REGISTER) != TWI_MT_DATA_ACK) return TWI_ERROR;
	TWI_Master_Start(); //Restart
	if((TWI_STATUS_REGISTER) != TWI_RESTART) return TWI_ERROR;
	ucDeviceAddress = (ucdevice_address << 1) | TMP75_DEVICE_TYPE_ADDRESS | TWI_READ;
	TWI_Master_Transmit(ucDeviceAddress); //Device-Adresse im Read-Modus senden
	if((TWI_STATUS_REGISTER) != TWI_MR_SLA_ACK) return TWI_ERROR;
	//Inhalt des adressierten Registers wird eingelesen
	ucTempHigh = TWI_Master_Read_Ack();
	if((TWI_STATUS_REGISTER) != TWI_MR_DATA_ACK) return TWI_ERROR;
	ucTempLow = TWI_Master_Read_NAck();
	if((TWI_STATUS_REGISTER) != TWI_MR_DATA_NACK) return TWI_ERROR;
	TWI_Master_Stop(); //Stopp
	iTemperature = (ucTempHigh << 8) + ucTempLow;
	return TWI_OK;
}

int TMP75_Get_Temperature(void)
{
	//Description:		Read temperature from TMP75
	//Call_parameter:	void
	//Return_parameter:	TMP75_Read_Temperature
	//Version:			1
	//Date :			220408
	//Autor:			Meroth, Sora
	//Source:			Sensornetzwerke In Theorie Und Praxis
	//Status:			not testet
	//--------------------------------
	return iTemperature;
}