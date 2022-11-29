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
uint8_t ucdevice_address =  0b1001000; //all address pins to gnd/  address according to book: 0x90;
uint8_t uctemp2read = 0b00000000;	//Read temperature register


//Definition of funcitons
//======================================================================================
uint8_t TMP75_Read_Temperature(void)
{
	//Description:		Read temperature from TMP75
	//Call_parameter:	ucdevice_address, uctemp2read register adress
	//Return_parameter:	TMP75_Read_Temperature
	//Version:			1
	//Date :			220408
	//Autor:			Meroth, Sora
	//Source:			Sensornetzwerke In Theorie Und Praxis
	//Status:			modified by Moritz
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
//======================================================================================
//======================================================================================
int TMP75_Get_Temperature(void)
{
	//Description:		Read temperature from TMP75
	//Call_parameter:	void
	//Return_parameter:	Temperature with one decimal number
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:			
	//Status:			released
	//--------------------------------	
	if (iTemperature >> 15)
	{
		iTemperature = (~iTemperature+1) * (-1);
	}
	return (iTemperature >> 8) * 10 + (iTemperature & 0b11110000) * 5/128;
}
//======================================================================================
//======================================================================================
void Display_Output(int iTemp2print, unsigned char ucLine, unsigned char ucCAN)
{
	//Description:		prints temperature to display
	//Call_parameter:	iTemp2print: temperature to print
	//					ucLine: line in which is printed
	//					ucCAN: CAN status
	//Return_parameter:	void
	//Version:			1
	//Date :			10.11.2022
	//Author:			Mattis Ritter
	//Source:
	//Status:			released
	//--------------------------------
	unsigned char ucDisp[2][16] = {" Actual:   0.0C ", " Target:   0.0C "};
	unsigned char j = 0;
	unsigned char ucNegFlag = 0;	//rememberes if number negative
	if (ucCAN == 111)
	{
		ucDisp[2][0] = '[';
		ucDisp[2][15] = ']';
	}
	if (iTemp2print < 0)
	{
		iTemp2print = iTemp2print *(-1);
		ucNegFlag = 1;		
	}
	while (iTemp2print != 0)
	{
		if (j == 1)
		{
			j++;
		}
		//Geht stelle für stelle der int Zahl durch und schreibt sie in array, von hinten beginnend
		ucDisp[ucLine][(13 - j)] = (char)(iTemp2print % 10) + 48;
		iTemp2print = iTemp2print / 10;
		j++;
	}
	if (ucNegFlag == 1)
	{
		if (j == 1)
		{
			j += 2;
		}
		ucDisp[ucLine][(13 - j)] = '-';
	}
	//Print to Display-----------------------------------------------
	Display_SetCursor(ucLine,0);
	Display_Print(ucDisp[ucLine],16);
	Display_SetCursor(2,0);		//Platzieren des Cursors auserhalb
	//---------------------------------------------------------------
}