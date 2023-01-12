#include "display_funktionen.h"


#define _1US_DELAY (20000000/FREQ_CPU)

unsigned char ucDataBit[8] = {DISPLAY_DATA_DB7_BIT, DISPLAY_DATA_DB6_BIT, DISPLAY_DATA_DB5_BIT, DISPLAY_DATA_DB4_BIT,
							DISPLAY_DATA_DB7_BIT, DISPLAY_DATA_DB6_BIT, DISPLAY_DATA_DB5_BIT, DISPLAY_DATA_DB4_BIT};

unsigned char ucRAM_OffsetAdresse[4] = {0x00, 0x40, 0x10, 0x50};


//----------------------------------------------------------------------------------------------
//Funktion: Display_HardwareInit
//Die Funktion initialisiert den �C f�r die Ansteuerung vom Display
//Aufgerufene Funktionen: 	Display_DATA_Output
//						  	Display_RS_Output
//							Display_EN_Output
//�bergebene Parameter	: 	keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_HardwareInit(void)
{
	Display_DATA_Output();	
	Display_RS_Output();								//Pin 0 bis 3 von Port C als Ausg�nge definiert
	Display_EN_Output();
}



//----------------------------------------------------------------------------------------------
//Funktion: Display_delay
//Die Funktion erm�glicht eine Verz�gerung um einen Vielfachen von einer Mikrosekunde
//Aufgerufene Funktionen:	keine
//Zur�ckgegebene Werte 	: keine
//�bergebene Parameter	: time vom Typ unsigned long, Mikrosekundenzahl
//----------------------------------------------------------------------------------------------
void Display_delay(unsigned long delay_time_us)
{
	unsigned long delay_time;

	delay_time = delay_time_us / _1US_DELAY;
	for(unsigned long j = 0; j < delay_time; j++)
	{
	}
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Aus()
//Die Funktion schaltet zwecks der Initialisierung das Display aus
//Aufgerufene Funktionen:	keine
//Zur�ckgegebene Werte:		keine
//----------------------------------------------------------------------------------------------
void Display_Aus(void)
{
	DISPLAY_SW_DDR_REG |= 1 << DISPLAY_SW_BIT;		//Bit 7 von Port D wird als Ausgang deklariert
	DISPLAY_SW_PORT_REG &= ~(1 << DISPLAY_SW_BIT);	//Display-Modul wird abgeschaltet
}

//----------------------------------------------------------------------------------------------
//Funktion: Display_An()
//Die Funktion schaltet nach der Initialisierung das Display an
//Aufgerufene Funktionen:	keine
//Zur�ckgegebene Werte:		keine
//----------------------------------------------------------------------------------------------
void Display_An(void)
{
	DISPLAY_SW_PORT_REG |= 1 << DISPLAY_SW_BIT;	//Display-Modul wird eingeschaltet
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_RS_Output
//Die Funktion setzt den Pin f�r die Ansteuerung der RS-Leitung auf Ausgang
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_RS_Output(void)
{
	DISPLAY_RS_DDR_REG |= 1 << DISPLAY_RS_BIT;
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_RS_High
//Die Funktion setzt das Signal RS f�r die Programmierung des Displays
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_RS_High(void)
{
	DISPLAY_RS_PORT_REG |= 1 << DISPLAY_RS_BIT;
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_RS_Low
//Die Funktion setzt das Signal RS f�r die Programmierung des Displays zur�ck
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_RS_Low(void)
{
	DISPLAY_RS_PORT_REG &= ~(1 << DISPLAY_RS_BIT);
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_EN_Output
//Die Funktion setzt den Pin f�r die Ansteuerung der EN-Leitung auf Ausgang
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_EN_Output(void)
{
	DISPLAY_EN_DDR_REG |= 1 << DISPLAY_EN_BIT;
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_EN_High
//Die Funktion setzt das Signal EN f�r die Programmierung des Displays
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_EN_High(void)
{
	DISPLAY_EN_PORT_REG |= 1 << DISPLAY_EN_BIT;
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_EN_Low
//Die Funktion setzt das Signal EN f�r die Programmierung des Displays zur�ck
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_EN_Low(void)
{
	DISPLAY_EN_PORT_REG &= ~(1 << DISPLAY_EN_BIT);
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_DATA_Output
//Die Funktion setzt die Pins f�r die Ansteuerung der Daten-Leitungen auf Ausgang
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_DATA_Output(void)
{
	DISPLAY_DATA_DDR_REG |= (1 << DISPLAY_DATA_DB7_BIT) | (1 << DISPLAY_DATA_DB6_BIT) |
							(1 << DISPLAY_DATA_DB5_BIT) | (1 << DISPLAY_DATA_DB4_BIT);
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_DATA_BitHigh
//Die Funktion setzt das gew�nschte Datenbit auf high
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_DATA_BitHigh(unsigned char DataBit)
{
	DISPLAY_DATA_PORT_REG |= (1 << DataBit);
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_DATA_BitLow
//Die Funktion setzt das gew�nschte Datenbit auf low
//Aufgerufene Funktionen: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_DATA_BitLow(unsigned char DataBit)
{
	DISPLAY_DATA_PORT_REG &= ~(1 << DataBit);
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Clear
//Die Funktion: l�scht den gesamten Inhalt vom Display und
//							setzt die Adresse vom Display-RAM auf 0x00
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0x01
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_Clear(void)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(DISPLAY_CLEAR_FUNCTION);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_CLEAR_DISPLAY_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_ReturnHome
//Die Funktion: setzt die Adresse vom Display-RAM auf 0x00 und 
//							kehrt auf die urspr�ngliche Position des Cursors zur�ck
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0x02
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: keine
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_ReturnHome(void)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(DISPLAY_CLEAR_FUNCTION);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_RETURN_HOME_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_ModeEntry
//Die Funktion: legt die Richtung der Verschiebung vom Cursor und Display fest 
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0 0 0 0 0 1 I/D SH
//								I/D = 1 der Cursor wird um eine Position nach rechts verschoben und
//												die Adresse vom DDRAM (CGRAM) wird inkrementiert
//								I/D	= 0 der Cursor wird um eine Position nach links verschoben und
//												die Adresse vom DDRAM (CGRAM) wird dekrementiert
//
//								SH  = 0 die angezeigten Zeichen werden nicht verschoben
//								SH  = 1 die angezeigten Zeichen werden nach rechts (I/D = 1) oder 
//												nach links (I/D = 0) um eine Stelle verschoben
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: Options 	- DISPLAY_MODE_INCR_SHIFT_OFF
//									- DISPLAY_MODE_INCR_SHIFT_ON
//									- DISPLAY_MODE_DECR_SHIFT_OFF
//									- DISPLAY_MODE_DECR_SHIFT_ON
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_ModeEntry(unsigned char Options)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(Options);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_MODE_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Control
//Die Funktion: steuert das Display und den Cursor 
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0 0 0 0 1 D C B
//								D = 1 Die Daten aus DDRAM werden eingeblendet
//								D	= 0 die Daten aus DDRAM werden ausgeblendet, die Daten bleiben erhalten
//
//								C = 1 blendet den Cursor ein
//								C = 0 blendet den Cursor aus
//				
//								B = 1 das Blinken des Cursors ist eingeblendet
//								B = 0 das Blinken des Cursor ist ausgeblendet			
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: Options	- DISPLAY_OFF
//									- DISPLAY_ON_CURSOR_OFF
//									- DISPLAY_ON_CURSOR_ON_BLINK_OFF
//									- DISPLAY_ON_CURSOR_ON_BLINK_ON
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_Control(unsigned char Options)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(Options);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_CONTROL_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: CursorOrDisplayShift
//Die Funktion: ohne Daten ein- oder auszulesen wird der Cursor oder die Anzeige nach rechts/links verschoben 
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0 0 0 1 S/C R/L X X
//
//								    S/C		R/L
//									0		0		der Cursor wird um eine Stelle nach links verschoben, der Adressez�hler wird dekrementiert
//									0		1		der Cursor wird um eine Stelle nach rechts verschoben, der Adressez�hler wird inkrementiert
//									1		0		verschiebt die gesamte Anzeige nach links, der Cursor wird dementsprechend auch verschoben
//									1		1		verschiebt die gesamte Anzeige nach rechts, der Cursor wird dementsprechend auch verschoben
//				
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: Options	- SHIFT_CURSOR_RECHTS
//									- SHIFT_CURSOR_LINKS
//									- SHIFT_DISPLAY_RECHTS
//									- SHIFT_DISPLAY_LINKS
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_CursorOrDisplayShift(unsigned char Options)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(Options);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_CURSOR_OR_DISPLAY_SHIFT_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_SetMPUInterface
//Die Funktion: legt die Parameter der Kommunikation zwischen Mikrocontroller und Display fest
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0 0 1 DL N F X X
//
//									DL = 0		8 Bit Datenbus
//									DL = 1		4 Bit Datenbus
//									 N = 0		1 Zeile
//									 N = 1		2 Zeilen
//									 F = 0		5 x 7 Pixel Display Modus
//									 F = 1		5 x 10 Pixel Display Modus
//				
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_EN_High
//							Display_DATA_BitHigh
//							Display_DATA_BitLow
//							Display_Transfer4BitData
//							Display_EN_Low
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: Options
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_SetMPUInterface(unsigned char Options)
{
	unsigned char dummy = 0x80, i;	//dummy dient zur Maskierung der einzelnen Bits

	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt
	Display_EN_High();		//EN-Leitung (Read/Write Freigabe) wird auf high gesetzt
							//die einzelne Datenbits werden beschrieben
	for(i = 0; i < 4; i++)	//die ersten 4 Datenbits werden beschrieben
	{
		if(Options & dummy)
		{
			Display_DATA_BitHigh(ucDataBit[i]);	//wenn das Bit im Options entsprechend dem dummy-Bit
		}					//1 ist, so wird das entsprechende Datenbit auf high gesetzt
		else
		{
			Display_DATA_BitLow(ucDataBit[i]);
		}

		dummy = dummy >> 1;
	}			

	Display_EN_Low();	//der Zustand auf den 4 Datenbit-Leitungen wird gespeichert
	Display_EN_High();	//EN-Leitung (Read/Write Freigabe) wird wieder auf high gesetzt	

	Display_Transfer4BitData(Options);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_SET_MPU_INTERFACE_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_SetCursor
//Die Funktion: setzt den Cursor an die gew�hlte Stelle auf einem 2- oder 4-zeiliges Display				
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 1 AC6 AC5 AC4 AC3 AC2 AC1 AC0
//Aufgerufene Funktionen: 	Display_HardwareInit
//							Display_RS_Low
//							Display_Transfer4BitData
//							Display_RS_High
//							Display_delay
//�bergebene Parameter	: row ist die Zeile (0 bis 3), column die Spalte (0 bis 15)
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_SetCursor(unsigned char row, unsigned char column)
{
	unsigned char adresse = DISPLAY_FUNKTION_SET_DDRAM_ADRESSE;	
	//adresse wird mit dem Instruction Code des Befehls initialisiert
	adresse += ucRAM_OffsetAdresse[row] + column;

	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(adresse);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_SET_RAM_ADRESSE_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Transfer_4Bit_Data()
//Die Funktion: sendet in 2 Schritten das Byte _8BitData an das Display
//Aufgerufene Funktionen:	Display_EN_High	
//							Display_DATA_BitHigh
//							Display_DATA_BitLow
//							Display_EN_Low
//�bergebene Parameter	: _8BitData
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_Transfer4BitData(unsigned char _8BitData)
{
	unsigned char dummy = 0x80, i;

	Display_EN_High();	////EN-Leitung (Read/Write Freigabe) wird auf high gesetzt

	for(i = 0; i < 8; i++)	//die ersten 4 Datenbits werden beschrieben
	{
		if(_8BitData & dummy)
		{
			Display_DATA_BitHigh(ucDataBit[i]);	//wenn das im Options entsprechend dem dummy Bit
		}					//1 ist, so wird das entsprechende Datenbit auf high gesetzt
		else				//ansonsten wird es auf low gesetzt
		{
			Display_DATA_BitLow(ucDataBit[i]);
		}

		dummy = dummy >> 1;
		if(i == 3)		//wenn die ersten 4 Bits beschrieben sind,
		{
			Display_EN_Low();	//so werden sie vom Displaycontroller gespeichert
			Display_EN_High();	//EN-Leitung (Read/Write Freigabe) wird wieder auf high gesetzt			
		}
	}

	Display_EN_Low();	//das low-nibble des Byte wird gespeichert
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Init
//die Funktion initialisiert das Display
//Aufgerufene Funktionen: 	Display_Aus
//							Display_delay
//							Display_An
//							Display_FunctionSet
//							Display_Clear
//							Display_ModeEntry
//							Display_Control
//�bergebene Parameter	: keine
//----------------------------------------------------------------------------------------------
void Display_Init(void)
{		

	Display_Aus();										//Display-Modul wird ausgeschaltet
	Display_delay(50000);										//0,05 s Wartezeit f�r Display-Reset
	Display_An();										//Display-Modul wird eingeschaltet

	//Initialisierung beginnt													
	Display_delay(30000);										//Verz�gerung > 30 ms
	Display_SetMPUInterface(DISPLAY_MPU_4BIT_2_LINES_5x7_DOTS);
	//die Kommunikation findet auf 4 Bit statt, 2zeiliges Display, 5x7 Pixel gro�e Buchstaben
	Display_Clear();	//der gesamte Inhalt des Displays wird gel�scht
	Display_ModeEntry(DISPLAY_MODE_INCR_SHIFT_OFF);	//der Cursor wird um eine stelle inkrementiert,
													//die Anzeige wird nicht verschoben
	Display_Control(DISPLAY_ON_CURSOR_ON_BLINK_OFF);//Anzeige und Cursor sind eingeblendet,
													//Cursor blinkt nicht
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Write
//Die Funktion schreibt an die aktuelle Position des Cursors das Zeichen char das als Parameter
//�bergeben wird
//Aufgerufene Funktionen: 	Display_HardwareInit
//						  	Display_RS_High
//							Display_Transfer4BitData
//							Display_RS_Low
//						  	Display_delay
//�bergebene Parameter	: ASCII_of_car - ASCII-Code des Zeichens das angezeigt werden soll
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_Write(unsigned char ASCII_of_char)
{
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_High();	//RS-Leitung auf high, Data-Register vom Display wird angew�hlt		

	Display_Transfer4BitData(ASCII_of_char);	//ASCII-code wird �bertragen
	Display_RS_Low();	//RS-Leitung wird auf low gesetzt
	Display_delay(DISPLAY_SET_RAM_ADRESSE_DELAY);	//warten auf das Ausf�hren des Befehls
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Print
//Die Funktion schreibt an die aktuelle Position des Cursors length-Zeichen aus dem Textarray
//text2print
//Aufgerufene Funktionen: 	Display_HardwareInit
//						  	Display_RS_High
//							Display_Write
//							Display_RS_Low
//�bergebene Parameter	: 	text2print - ein Zeiger auf die Adresse vom Textarray
//							length - Zeichenzahl
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_Print(unsigned char* text2print, unsigned char length)
{
	unsigned char i = 0;

	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_High();	//RS-Leitung auf high, Data-Register vom Display wird angew�hlt
	for(i = 0; i < length; i++)
	{
		Display_Write(text2print[i]);	//das i-te Zeichen wird angezeigt
	}
	Display_RS_Low();	//RS-Leitung wird auf low gesetzt
}


//----------------------------------------------------------------------------------------------
//Funktion: Display_Generate_New_Char
//Die Funktion: generiert ein neues Zeichen
//Instruction Code: DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 = 0 1 AC5 AC4 AC3 AC2 AC1 AC0
//Aufgerufene Funktionen: 	
//�bergebene Parameter	: 	address - ASCII-Code des neuen Zeichens
//							Pattern_New_Char - ein Zeiger auf das Bitmuster des neuen Zeichens
//Zur�ckgegebene Werte	: keine
//----------------------------------------------------------------------------------------------
void Display_GenerateNewChar(unsigned char address, unsigned char* Pattern_New_Char)
{
	unsigned char adresse = DISPLAY_FUNKTION_SET_CGRAM_ADRESSE;
	unsigned char i;

	adresse |= address << 3;
	Display_HardwareInit();	//der �C wird f�r die Ansteuerung vom Display initialisiert
	Display_RS_Low();		//RS-Leitung auf low, Befehls-Register vom Display wird angew�hlt

	Display_Transfer4BitData(adresse);	//Funktionscode wird �bertragen
	Display_RS_High();		//RS-Leitung geht auf high
	Display_delay(DISPLAY_SET_RAM_ADRESSE_DELAY);	//warten auf das Ausf�hren des Befehls

	for(i = 0; i < 8; i++)
	{
		Display_Write(Pattern_New_Char[i]);	//die i-te Zeile wird abgespeichert
	}
	Display_RS_Low();	//RS-Leitung wird auf low gesetzt
	
}

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
	if (ucCAN == 0)
	{
		ucDisp[1][0] = '[';
		ucDisp[1][15] = ']';
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
		//Geht stelle f�r stelle der int Zahl durch und schreibt sie in array, von hinten beginnend
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
