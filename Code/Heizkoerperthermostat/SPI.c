/*
 * SPI.c
 *
 * Created: 11.01.2015 12:29:19
 *  Author: Petre Sora
 */ 

//****************************************************************************************
//Modul: SPI.c
//Datum: 11.01.2015
//Autor: Meroth/Sora
//Version: 2.0 
//Beschreibung: das Modul beinhaltet eine Bibliothek mit Funktionen für die SPI-Schnittstelle
//Revision: 05.07.2017
//Status: freigegeben
//****************************************************************************************

//****************************************************************************************
//include Dateien
//****************************************************************************************

#include "SPI.h"

//****************************************************************************************
//Variablen
//****************************************************************************************


//****************************************************************************************
//Funktion: SPI_Master_Init
//Datum: 11.01.2015
//Autor: Meroth/Sora
//Version: 1.0
//Beschreibung:	die Funktion initialisiert die SPI-Schnittstelle des Mikrocontrollers
//				als Master
//Aufgerufene Funktionen:	keine
//Übergabeparameter:		ucspi_interrupt:	- SPI_INTERRUPT_ENABLE
//												- SPI_INTERRUPT_DISABLE
//							ucspi_data_order:	- SPI_LSB_FIRST
//												- SPI_MSB_FIRST
//							ucspi_mode:			- SPI_MODE_0
//												- SPI_MODE_1
//												- SPI_MODE_2
//												- SPI_MODE_3
//							ucspi_sck_freq:		- SPI_FOSC_DIV_2
//												- SPI_FOSC_DIV_4
//												- SPI_FOSC_DIV_8
//												- SPI_FOSC_DIV_16
//												- SPI_FOSC_DIV_32
//												- SPI_FOSC_DIV_64
//												- SPI_FOSC_DIV_128
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void SPI_Master_Init(uint8_t ucspi_interrupt, uint8_t ucspi_data_order, uint8_t ucspi_mode, uint8_t ucspi_sck_freq)
{
	SPI_MOSI_DDR_REG |= 1 << SPI_MOSI_BIT;		//MOSI-Pin wird auf Ausgang deklariert
	SPI_MISO_DDR_REG &= ~(1 << SPI_MISO_BIT);	//MISO-Pin wird auf Eingang deklariert
	SPI_CLK_DDR_REG |= 1 << SPI_CLK_BIT;		//CLK-Pin wird auf Ausgang deklariert
	
	SPI_CONTROL_REGISTER = SPI_MASTER | SPI_ENABLE | ucspi_interrupt | ucspi_data_order | (ucspi_mode << 2) | (ucspi_sck_freq % 4);
		//µC wird als Master deklariert, die SPI-Schnittstelle wird freigegeben
		//falls gewünscht, es wird das SPI-Interrupt freigegeben und die Bitreihenfolge wird bestimmt
		//der Übertragungsmodus wird bestimmt

	SPI_STATUS_REGISTER = ucspi_sck_freq / 4;
		//die gewünschte SPI-Taktfrequenz wird gewählt
}


//****************************************************************************************
//Funktion: SPI_Master_SlaveSelectInit
//Datum: 03.07.2017
//Autor: P. Sora
//Version: 1.2
//Beschreibung:	die Funktion initialisiert (setzt den gewünschten Pin auf Ausgang und der Ausgang
//				wird auf high gesetzt) eine ausgewählte Slave Select Leitung 
//Aufgerufene Funktionen:	keine
//Übergabeparameter:	tspiHandle tspi_pins: ist eine Struktur um die Chip-Select Leitung 
//						eines Slaves ansteuern zu können; (siehe SPI.h)
//							1. Element - die Adresse des DDR-Registers an dem die Slave Select Leitung angeschlossen ist
//							2. Element - die Adresse des PORT-Registers an dem die Slave Select Leitung angeschlossen ist
//							3. Element - das entsprechende Bit der Slave Select Leitung
//							4. Element - angesteuerte Leitung ON (OFF fest angeschlossen 
//						diese Struktur ist ein Element einer weiteren Struktur die die 
//						ansteuerbaren Pins eines SPI-Slaves enthält. Diese Struktur wird für jeden Slave 
//						deklariert. In der Headerdatei eines Slave-Moduls kann diese Struktur 
//						folgendermaßen deklariert werden:
//						typedef struct
// 						{
//							tspiHandle modul1spi;

// 							volatile uint8_t* Pin1_DDR;
// 							volatile uint8_t* Pin1_PORT;
// 							uint8_t Pin1_pin;
// 							uint8_t Pin1_state;
// 						} modul1_pins;
//						Für diesen Slave könnte die Struktur in der Hauptfunktion folgendermaßen
//						initialisiert werden:
// 						modul1_pins main_pins = {{	/*CS_DDR*/ &DDRC,
// 													/*CS_PORT*/ &PORTC,
// 													/*CS_pin*/ PC1,
// 													/*CS_state*/ 1},
//													/*CS_DDR*/ &DDRB,
// 													/*CS_PORT*/ &PORTB,
// 													/*CS_pin*/ PB2,
// 													/*CS_state*/ 1
// 													};
//						Das Element State eines Pins wird mit 0 initialisiert wenn dieser
//						Pin fest angeschlossen ist
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void SPI_Master_SlaveSelectInit(tspiHandle tspi_pins)
{
	*tspi_pins.CS_DDR |= 1 << tspi_pins.CS_pin;
	*tspi_pins.CS_PORT |= 1 << tspi_pins.CS_pin;
}


//****************************************************************************************
//Funktion: SPI_Master_Start
//Datum: 03.07.2017
//Autor: Meroth/Sora
//Version: 1.2
//Beschreibung:	die Funktion startet die SPI-Datenübertragung (SlaveSelect Pin wird auf 
//				Low gesetzt)
//Aufgerufene Funktionen:	keine
//Übergabeparameter:		tspi_Pins: siehe SPI_Init
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void SPI_Master_Start(tspiHandle tspi_pins)
{
	*tspi_pins.CS_PORT &= ~(1 << tspi_pins.CS_pin);
}


//****************************************************************************************
//Funktion: SPI_Master_Stop
//Datum: 03.07.2017
//Autor: Meroth/Sora
//Version: 1.1
//Beschreibung:	die Funktion beendet die SPI-Datenübertragung (SlaveSelect Pin wird auf
//				High gesetzt)
//Aufgerufene Funktionen:	keine
//Übergabeparameter:		tspi_Pins: siehe SPI_Init
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void SPI_Master_Stop(tspiHandle tspi_pins)
{
	*tspi_pins.CS_PORT |= (1 << tspi_pins.CS_pin);
}


//****************************************************************************************
//Funktion: SPI_Master_Write
//Datum: 12.01.2015
//Autor: Meroth/Sora
//Version: 1.0
//Beschreibung:	unter den, mit der Funktion SPI_Master_Init festgesetzten Vereinbarungen
//				wird ein Byte übertragen; gleichzeitig wird von dem Slave in das Empfangsregister
//				ein Byte geschoben
//Aufgerufene Funktionen:	keine
//Übergabeparameter:		ucdata - das zu übertragende Byte
//Rückgabeparameter:		das empfangene Byte
//Revision:
//Status: freigegeben
//****************************************************************************************
uint8_t SPI_Master_Write(uint8_t ucdata)
{
	SPI_DATA_REGISTER = ucdata;
	while(SPI_RUNNING);
	return SPI_DATA_REGISTER;
}