/*
 * Cantest2020-01.c
 *
 * Created: 05.05.2020 11:25:07
 * Author : Meroth
 */ 

#include <avr/io.h>
#include "Timer2.h"
#include "display_funktionen.h"
#include "SPI.h"
#include "LED.h"
#include "MCP2515_HHN.h"

MCP2515_pins MCP2515_1 ={{	/*CS_DDR*/		&DDRB,
							/*CS_PORT*/		&PORTB,
							/*CS_pin*/		PB2,
							/*CS_state*/	ON}};

unsigned int MeineTemperatur;
unsigned int MeinWert;

void HexToAscii(unsigned char input, unsigned char* output)
{
	char c;
	c=input & 0x0f;
	if (c<10) output[1]=c+48;
	else output[1]=c+55;
	c=input>>4;
	if (c<10) output[0]=c+48;
	else output[0]=c+55;
}

void ShowMessage(can_frame *showFrame)
{
	unsigned char zeile1[16]="ID:0x       DL: ";
	unsigned char zeile2[16];
	HexToAscii(showFrame->ulID>>16, &zeile1[5]);
	HexToAscii(showFrame->ulID>>8, &zeile1[7]);
	HexToAscii(showFrame->ulID, &zeile1[9]);
	zeile1[15]=showFrame->ucLength+48;
	Display_Clear();
	Display_SetCursor(0,0);
	Display_Print(zeile1,16);
	for (int i=0; i<16;i++) zeile2[i]=' ';
	for (int i=0; i<showFrame->ucLength;i++)
	{
		HexToAscii(showFrame->ucData[i],&zeile2[i*2]);
	}
	Display_SetCursor(1,0);
	Display_Print(zeile2,16);
}

unsigned int GetTemperature()
{
	return MeinWert;
}


void Call_function_1(can_frame *sFrame)
{
	MeinWert = sFrame->ucData[0] | (unsigned int)sFrame->ucData[1]<<8;
}

int main(void)
{
	can_frame sSendFrame;
	can_frame sRecFrame;
	LEDInit();
	InitTimer2CTC();
	Display_Init();
	MCP2515_Init(MCP2515_1,BAUDRATE_250_KBPS);
	sei();
	
    /* Replace with your application code */
    while (1) 
    {
		if(MCP2515_Check_Message(MCP2515_1,&sRecFrame))
		{
			ShowMessage(&sRecFrame);
			switch(sRecFrame.ulID)
			{
				case 0x10: Call_function_1(&sRecFrame); break ;
				default: break;
			}
		}
		if (Check100msFlag())
		{
			sSendFrame.EIDE_Bit = STANDARD_ID;
			sSendFrame.RTR_Bit = DATA_FRAME;
			sSendFrame.ulID = 0x19; // bitte entsprechend eintragen!!
			sSendFrame.ucLength=2;
			/********************************/
			sSendFrame.ucData[0]=(unsigned char) MeineTemperatur;
			sSendFrame.ucData[1]=(unsigned char) (MeineTemperatur>>8); //Klammer ist wichtig, da typecast höhere Prio hat als shift
			/********************************/
			MCP2515_Send_Message(MCP2515_1, &sSendFrame);

			
		}
		if(Check1sFlag())
		{
	
			MeineTemperatur=GetTemperature();// hier die Funktion eintragen zum Auslesen der Temperatur
		}
    }
}

