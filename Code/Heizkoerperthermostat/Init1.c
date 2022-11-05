/*
 * Init1.c
 *
 * Created: 26.10.2022 20:38:13
 *  Author: Moritz
 */ 

//Includes
#include "Init1.h"

//Variables

//Definition of funcitons

void GerneralInit(void){
	//Description:		execute all inits
	//Call_parameter:	void
	//Return_parameter:	void
	//Version:			1
	//Date :			27.10.2022
	//Autor:			Moritz
	//Source:			
	//Status:			not testet
	//--------------------------------
	Display_Init();
	LED_Init();
	
	WS2812_Init();
	keys_Init();
	Timer1_Init();
	
	//Calculate TWI register clock=========================================================
	unsigned int F_CPU = 20/*[MHz]*/;			//Clock of uC_Board in Hz
	unsigned int TWI_SCL_FREQ = 400/*[kHz]*/;	//needed TWI frequency for fast speed TWI
	unsigned int itwi_clock = (((F_CPU*1000 / TWI_SCL_FREQ)- 16) / 2 + 1);
	unsigned char uctwi_clock = (unsigned char)itwi_clock;
	//=====================================================================================	
	TWI_Master_Init(uctwi_clock);

	//CAN_Filter_Init();
	//InitTimer2CTC();
	
}



//void CAN_Filter_Init(void)
//{
	////Description:		can filter
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			27.10.2022
	////Autor:			Sora
	////Source:			Ilias
	////Status:			not testet
	////--------------------------------
	//sFilter.Rec_Buff0_Rollover = ROLLOVER_ON;
	//sFilter.RecBuff_ID[0] = STANDARD_ID;
	//sFilter.RecBuff_ID[1] = STANDARD_ID;
	//sFilter.Filter_RecBuff[0] = FILTER_ON;
	//sFilter.Filter_RecBuff[1] = FILTER_ON;
	//for(uint8_t ucI = 0; ucI < 6; ucI++)
	//{
		//sFilter.ulRecBuff_Filter[ucI] = ulReceiveFilter[ucI];
	//}
	//for(uint8_t ucI = 0; ucI < 2; ucI++)
	//{
		//sFilter.ulRecBuff_Mask[ucI] = ulReceiveMask[ucI];
	//}
//}


void HexToAscii(unsigned char input, unsigned char* output)
{
	//Description:		Wandelt hex to ascii
	//Call_parameter:	void
	//Return_parameter:	void
	//Version:			1
	//Date :			27.10.2022
	//Autor:			meroth
	//Source:			Ilias
	//Status:			not testet
	//--------------------------------
	char c;
	c=input & 0x0f;
	if (c<10) output[1]=c+48;
	else output[1]=c+55;
	c=input>>4;
	if (c<10) output[0]=c+48;
	else output[0]=c+55;
}

//void ShowMessage(can_frame *showFrame)
//{
	////Description:		shows message on board display
	////Call_parameter:	void
	////Return_parameter:	void
	////Version:			1
	////Date :			27.10.2022
	////Autor:			meroth
	////Source:			Ilias
	////Status:			not testet
	////--------------------------------
	//unsigned char zeile1[16]="ID:0x       DL: ";
	//unsigned char zeile2[16];
	//HexToAscii(showFrame->ulID>>16, &zeile1[5]);
	//HexToAscii(showFrame->ulID>>8, &zeile1[7]);
	//HexToAscii(showFrame->ulID, &zeile1[9]);
	//zeile1[15]=showFrame->ucLength+48;
	//Display_Clear();
	//Display_SetCursor(0,0);
	//Display_Print(zeile1,16);
	//for (int i=0; i<16;i++) zeile2[i]=' ';
	//for (int i=0; i<showFrame->ucLength;i++)
	//{
		//HexToAscii(showFrame->ucData[i],&zeile2[i*2]);
	//}
	//Display_SetCursor(1,0);
	//Display_Print(zeile2,16);
//}