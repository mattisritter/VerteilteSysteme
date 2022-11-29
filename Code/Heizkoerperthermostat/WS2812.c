/*
 * WS2812.c
 *
 * Created: 30.04.2018 11:47:18
 *  Author: Petre Sora
 * Disclaimer: This code sample is just a prototype. It is in no way suitable
 * for safe and reliable code, since the authors have intentionally abstained
 * from error or plausibility checks.
 * Therefore, the code examples must not be used in military or commercial or
 * safety-related products. Running this software can potentially be dangerous.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted under license CC-BY-NC-SA provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the above disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the above disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 */ 

//****************************************************************************************
//Modul: WS2812.c
//Datum: 30.04.2018
//Autor: P. Sora
//Version: 1.0
//Beschreibung: das Modul beinhaltet eine Bibliothek für die Ansteuerung der volfarbigen
//				LED's vom Typ WW2812
//Status: freigegeben
//****************************************************************************************

//****************************************************************************************
//include Dateien
//****************************************************************************************
#include "WS2812.h"



//****************************************************************************************
//Variablen
//****************************************************************************************
//uint8_t *PtrColourList; //lokale Adresse der Farbliste
//uint16_t *PtrLedNumber; //Adresse der Variable die die Anzahl der Farben Speichert (Vielfach von 3)-1

uint8_t *PtrColourList, ucPin;
volatile uint8_t ucPort;
uint16_t *PtrLedNumber, PtrPort;
extern WS2812_pin WS2812_1;
//Color definition ===== g  r  b =============================
uint8_t uColor[6][3] = {{0, 0 ,7},  /*blue*/
						{7, 0 ,0},  /*green*/
						{3, 4, 0},  /*yellow*/
						{2, 4, 1},  /*orange*/
						{0, 4, 3},  /*magenta*/
						{0, 7, 0}}; /*red*/
//============================================================


//uint8_t ucTresetTime[2] = {PRESCALER_64, 9};	//52µs @ 12,288MHz

//****************************************************************************************
//Funktionen
//****************************************************************************************
//****************************************************************************************
//Funktion: WS2812_Init
//Datum: 01.01.2019
//Autor: P. Sora
//Version: 1.0
//Beschreibung:	die Funktion initialisiert den Steuerpin für das LED-Band
//Aufgerufene Funktionen:	
//Übergabeparameter:		keine
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void WS2812_Init(void)
{
	*WS2812_1.WS2812_DDRReg |= 1 << WS2812_1.WS2812_Pin; //der anzusteuernde Pin wird auf Ausgang gesetzt
}


//****************************************************************************************
//Funktion: WS2812_set_off
//Datum: 01.01.2019
//Autor: P. Sora
//Version: 1.0
//Beschreibung:	die Funktion isetzt den Steuerpin für das LED-Band auf Low
//Aufgerufene Funktionen:
//Übergabeparameter:		keine
//Rückgabeparameter:		keine
//Revision:
//Status: freigegeben
//****************************************************************************************
void WS2812_set_off(void)
{
	*WS2812_1.WS2812_PORTReg &= ~(1 << WS2812_1.WS2812_Pin); //der anzusteuernde Pin wird auf Low gesetzt
}


//****************************************************************************************
//Funktion: WS2812_Set_Colour
//Datum: 30.04.2018
//Autor: P. Sora
//Version: 1.0
//Beschreibung:	Die Funktion initialisiert den µC für die Ansteuerung einer LED-Kette (WS2812)
//				
//Aufgerufene Funktionen: 	
//Übergabeparameter:		*uccolour_list - Adresse eines Arrays das die Farben aller LED's speichert
//							uiled_number - Anzahl der Leuchtdioden (Vielfach von 3)-1
//Rückgabeparameter:		keine
//Revision:					
//Status: freigegeben
//****************************************************************************************
void WS2812_Set_Colour(uint8_t *uccolour_list, uint16_t uiled_number)
{
	*WS2812_1.WS2812_PORTReg &= ~(1 << WS2812_1.WS2812_Pin); //der anzusteuernde Pin wird 
	//zwecks Reset auf Low geschaltet
	
	PtrPort = (uint16_t) WS2812_1.WS2812_PORTReg; //die Adresse des Port Registers wird gespeichert
	ucPin = WS2812_1.WS2812_Pin; //die Pinnummer wird gespeichert
	PtrColourList = uccolour_list; //die Adresse der Liste der LED-Farben wird gespeichert
	PtrLedNumber = &uiled_number; //die Anzahl der LED's wird gespeichert
	
	__asm__ __volatile__ (
	"\n"
	"cli"	"\n\t" //alle Interrupts werden deaktiviert um die Übertragung nicht zu stören
				"push	r19"	"\n\t"
				"push	r20"	"\n\t"
				"push	r21"	"\n\t"
				"push	r22"	"\n\t"
				"push	r23"	"\n\t"
				"push	r24"	"\n\t"
				"push	r25"	"\n\t"
				"push	r26"	"\n\t"
				"push	r27"	"\n\t"
				"push	r28"	"\n\t"
				"push	r29"	"\n\t"
				"push	r30"	"\n\t"
				"push	r31"	"\n\t"
				"lds	r25, ucPin"	"\n\t" //in r25 wird die Pinzahl geladen (z.B. 6)
				"lds	r26, PtrColourList"	"\n\t" //in das XRegister wird die Anfangsadresse der Farbenliste gespeichert
				"lds	r27, PtrColourList+1"	"\n\t"
				"lds	r30, PtrLedNumber"	"\n\t" //in das Register Z wird die Adresse der LED-Anzahl gespeichert
				"lds	r31, PtrLedNumber+1"	"\n\t"
				"ld		r28, Z+"	"\n\t" //in das Register Y wird die Anzahl der LED's gespeichert
				"ld		r29, Z"	"\n\t"
				"lds	r30, PtrPort"	"\n\t" //in Z wird die Adresse des Portregisters gespeichert
				"ldi	r31, 0x00"	"\n\t"
				"ldi	r20, 0x01"	"\n\t" //r20 wird vorbereitet für die Maskierung des PORT-Registers
				
	"repeat:"	"cpi	r25, 0"	"\n\t"	//in der Schleife wird die Verschiebung 1 << Pxy durchgeführt
				"breq	label1"	"\n\t"
				"dec	r25"	"\n\t"
				"lsl	r20"	"\n\t"
				"rjmp	repeat"	"\n\t"
				
	"label1:"	"mov	r19,r20"	"\n\t"
				"com	r19"	"\n\t"
				
	"loop:"		"ldi	r23, 0xFE"	"\n\t" //r23 speichert die Bitmaske (0x80 bis 0x01); Schleife für die Übertragung aller Farbenbytes aus der Liste
				"ld		r24, X+"	"\n\t" //in r24 wird ein Farbe-Byte aus der Liste geladen
	"out:"		"lsl	r24"	"\n\t" //ein Bit wird auf den Wert getestet; Schleife für die Übertragung eines FarbenByte

				"brcs	set1"	"\n\t" //wenn das getestete Bit 1 ist, springe zu set1
	"set0:"		"ld		r21, Z"	"\n\t" //in r21 wird der Inhalt des Registers PORT geladen und; ein 0-Bit wird übertragen
				"or		r21, r20"	"\n\t" //mit dem zu steuernden Bit ver-odert
				"st		Z, r21"	"\n\t" //der neue Wert wird in das Register PORT geladen

				"ld		r21, Z"	"\n\t" //der Inhalt des Registers PORT wird geladen und
				"and	r21, r19"	"\n\t" // mit der entsprechenden Maske ver-undet
				"st		Z, r21"	"\n\t" //das gewünschte Bit wird zurückgesetzt
				//Die Taktfrequenz des Quarzes beträgt 18,432 MHz
				//für 12,288MHz. ("0"- Hi-5 Takte, Lo-12 Takte) werden die weiteren 2 Zeilen auskommentiert
				"nop"	"\n\t"
				"nop"	"\n\t"

				"nop"	"\n\t"
				"nop"	"\n\t"
				"nop"	"\n\t"
				"rjmp	rot1"	"\n\t"
	"set1:"		"ld		r21, Z"	"\n\t" //ein 1-Bit wird übertragen
				"or		r21, r20"	"\n\t"
				"st		Z, r21"	"\n\t"
				
				//Die Taktfrequenz des Quarzes beträgt 18,432 MHz
				//für 12,288MHz. ("0"- Hi-5 Takte, Lo-12 Takte) werden die weiteren 2 Zeilen auskommentiert
				"nop"	"\n\t"
				"nop"	"\n\t"
				
				"nop"	"\n\t"
				"nop"	"\n\t"
				"nop"	"\n\t"
				"nop"	"\n\t"
				"ld		r21, Z"	"\n\t"
				"and	r21, r19"	"\n\t"
				"st		Z, r21"	"\n\t"

	"rot1:"		"lsl	r23"	"\n\t" //es wird geprüft ob das LSB-Bit des Farbenbyte übertragen wurde
				"brcs	out"	"\n\t" //wenn nicht, dann Sprung zu out_
				"sbiw	r28, 1"	"\n\t" //die Anzahl der noch zu übertragenden Farbenbytes wird dekrementiert
				"brcc	loop" "\n\t" //wenn die Zahl noch nicht Null ist, Sprung zu loop
				"pop	r31"	"\n\t"
				"pop	r30"	"\n\t"
				"pop	r29"	"\n\t"
				"pop	r28"	"\n\t"
				"pop	r27"	"\n\t"
				"pop	r26"	"\n\t"
				"pop	r25"	"\n\t"
				"pop	r24"	"\n\t"
				"pop	r23"	"\n\t"
				"pop	r22"	"\n\t"
				"pop	r21"	"\n\t"
				"pop	r20"	"\n\t"
				"pop	r19"	"\n\t"
				"sei" //die Interrupts werden aktiviert
	::);
}

void WS2812_Step(unsigned char ucStep){
	//Description:		sets LED color
	//Call_parameter:	ucStep: heater position from 0 to 5
	//Return_parameter:	void
	//Version:			1
	//Date :			31.10.2022
	//Author:			Mattis Ritter
	//Source:
	//Status:			released
	//--------------------------------
	WS2812_Set_Colour(uColor[ucStep],2);
}