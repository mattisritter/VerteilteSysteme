/**
 * @file TMP75.c
 * @brief C file to measure temperature with TMP75
 * @section Comment
 * Communication with TMP75 via I^2C
 * @date 26.10.2022 20:07:03
 * @author Hoehnel and Ritter
 */ 


//Includes======================================================
#include "TMP75.h"

//Variables=====================================================
int iTemperature;
uint8_t ucdevice_address =  0b1001000; /**<Device address: all address pins to gnd*/  //address according to book: 0x90;
uint8_t uctemp2read = 0b00000000;	/**<Select register to read: Read temperature register*/

//Definition of functions=======================================
/** @brief Read temperature from TMP75
* @param[in] None
* @return [uint8_t] Value of Temperature measurement
* @date 04.11.2022
* @author Meroth, Sora
* @version 1.0
*/
//Source:			Sensornetzwerke In Theorie Und Praxis
//--------------------------------
uint8_t TMP75_Read_Temperature(void)
{
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
/** @brief Get method to get current temperature value
* @param[in] None
* @return [int] Last read temperature, with one decimal number
* @date 04.11.2022
* @author Hoehnel and Ritter
* @version 1.0
*/
//--------------------------------
int TMP75_Get_Temperature(void)
{
	if (iTemperature >> 15)
	{
		iTemperature = (~iTemperature+1) * (-1);
	}
	return (iTemperature >> 8) * 10 + (iTemperature & 0b11110000) * 5/128;
}
//======================================================================================
