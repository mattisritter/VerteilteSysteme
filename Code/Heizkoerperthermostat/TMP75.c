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
	//die Adresse des gew�nschten Temperaturregisters wird gesendet
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
	return (iTemperature >> 8) * 10 + ((iTemperature >> 7) & 1) * 5;
}
//======================================================================================
//=====================================================================================================
void Disp_PrintTemperature(int ui2print){
	//Description:		Prints actual temp
	//Call_parameter:	void
	//Return_parameter:	Temperature with one decimal number
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:
	//Status:			released
	//--------------------------------
	//Prefill of array-----------------------------
	unsigned char ucDisp[16] = " Actual:      C ";
	//---------------------------------------------
	//Handling negative numbers-------------------------------------
	unsigned char ucNegFlag = 0;	//rememberes if number negative
	if (ui2print < 0)
	{
		ui2print = ui2print *(-1);
		ucNegFlag = 1;
		
	}
	//---------------------------------------------------------------
	//add temp value to array---------------------------------------------------------------------
	for(unsigned char i = 0; i < 6; i++){
		if (i == 1)
		{
			ucDisp[(13-i)] = '.';
			i++;
		}
		//Geht stelle f�r stelle der int Zahl durch und schreibt sie in array, von hinten beginnend
		ucDisp[(13 - i)] = (char)(ui2print % 10) + 48;
		ui2print = ui2print / 10;
	}
	//---------------------------------------------------------------------------------------------
	//Replace zeros by spaces-----------------------------------------------------------------------
	unsigned char ucNotZero = 0; //varaible to break while loop at first number not 0
	unsigned char ucPosDispArray = 8; //Position in array for while loop
	while (ucNotZero == 0)
	{
		//Breaks if number is not 0 or last digit is reached
		if((ucDisp[ucPosDispArray] != '0') || (ucPosDispArray >= 11) )
		{
			ucNotZero = 1;
			if (ucNegFlag == 1) /*showing - if at negative temp*/
			{
				ucDisp[(ucPosDispArray-1)] = '-';
			}
		}
		else if (ucDisp[ucPosDispArray] == '0')
		{
			ucDisp[ucPosDispArray] = ' ';
			ucPosDispArray++;
		}
	}
	//------------------------------------------------------------------------------------------------
	//Print to Display-----------------------------------------------
	Display_SetCursor(0,0);
	Display_Print(ucDisp,16);
	Display_SetCursor(2,0);		//Platzieren des Cursors auserhalb
	//---------------------------------------------------------------
}
//=====================================================================================================
//=====================================================================================================
void Disp_PrintTarget(int ui2print, unsigned char CANStatus){
	//Description:		Print target temp
	//Call_parameter:	target temp , status if can already received
	//Return_parameter:	none
	//Version:			1
	//Date :			4.11.22
	//Author:			Moritz
	//Source:
	//Status:			released
	//--------------------------------
	//Prefill of array-----------------------------
	unsigned char ucDisp[16] = " Target:      C ";
	if (CANStatus == 111)
	{
		ucDisp[0] = '[';
		ucDisp[15] = ']';
	}
	
	//---------------------------------------------
	//Handling negative numbers-------------------------------------
	unsigned char ucNegFlag = 0;	//rememberes if number negative
	if (ui2print < 0)
	{
		ui2print = ui2print *(-1);
		ucNegFlag = 1;
	}
	//---------------------------------------------------------------
	//add temp value to array---------------------------------------------------------------------
	for(unsigned char i = 0; i < 6; i++){
		if (i == 1)
		{
			ucDisp[(13-i)] = '.';
			i++;
		}
		//Geht stelle f�r stelle der int Zahl durch und schreibt sie in array, von hinten beginnend
		ucDisp[(13 - i)] = (char)(ui2print % 10) + 48;
		ui2print = ui2print / 10;
	}
	//---------------------------------------------------------------------------------------------
	//Replace zeros by spaces-----------------------------------------------------------------------
	unsigned char ucNotZero = 0; //varaible to break while loop at first number not 0
	unsigned char ucPosDispArray = 8; //Position in array for while loop
	while (ucNotZero == 0)
	{
		//Breaks if number is not 0 or last digit is reached
		if((ucDisp[ucPosDispArray] != '0') || (ucPosDispArray >= 10) )
		{
			ucNotZero = 1;
			if (ucNegFlag == 1) /*showing - if at negative temp*/
			{
				ucDisp[(ucPosDispArray-1)] = '-';
			}
		}
		else if (ucDisp[ucPosDispArray] == '0')
		{
			ucDisp[ucPosDispArray] = ' ';
			ucPosDispArray++;
		}
	}
	//------------------------------------------------------------------------------------------------
	//Print to Display-----------------------------------------------
	Display_SetCursor(1,0);
	Display_Print(ucDisp,16);
	Display_SetCursor(2,0);		//Platzieren des Cursors auserhalb
	//---------------------------------------------------------------
}
//=====================================================================================================
