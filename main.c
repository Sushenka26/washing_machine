/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL25Z4.h"
#include "drv_pracka.h"
#include "drv_systick.h"
#include "drv_gpio.h"
#include "drv_lcd.h"
#include <stdio.h>
#include <stdbool.h>

#define SWITCH_PRESSED		(1)
#define SWITCH_NOT_PRESSED	(0)
#define START				1
#define NAPOUSTENI			2
#define OHREV				3
#define TOCENI				4
#define VYCERPANI			5
void ADCInit();
void switch_init();
void TaskPracuj();
void holdManager(void);


uint32_t ADCCalibrate(void);

int switch1_read();
int switch2_read();
int switch3_read();
int switch4_read();
void TaskSwithces(int SW);
static int i = 0;
int stav = 0;
int pocetCyklu = 2;
int teplota = 30;
int temperatureEnd = -10;

bool zapnuto = false;
bool boost = false;
bool smer = false;
bool ONOtoBEZI = false;

wodaCas;
startWaitNapousteni;
stopWaitNapousteni;
startWaitVypousteni;
stopWaitVypousteni;

int hladina;
int teplo;
int waitTime;

int main(void)
{
	/* Write your code here */
	GPIO_Initialize();
	SYSTICK_initialize();
	PRACKA_init();
	switch_init();

	// Inicializace A/D prevodniku
	ADCInit();

	// Kalibrace a nova inicializace
	// Pro dosazeni udavane presnosti musi byt prevodnik kalibrovan po
	// kazdem resetu.
	// Nova inicializace je potreba protoze pri kalibraci
	// je prevodnik prenastaven.

	ADCCalibrate();
	ADCInit();

	LCD_initialize();
	LCD_clear();


	//Potenciometr
	// 1. Povolime hodinovy signal pro port C
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	// 2. NAstavime funkci pinu na ALT0 = vstup A/D prevodniku
	PORTC->PCR[2] = PORT_PCR_MUX(0);




	while(true)
	{
		TaskPracuj();
	}


	/* int idk = PRACKA_CtiTeplotu();
	char str[3];
	sprintf(str, "%d", idk);
	LCD_clear();
	LCD_puts(str);

	PRACKA_NastavTopeni(true);
	SYSTICK_delay_ms(10000);
	PRACKA_NastavTopeni(false);
	SYSTICK_delay_ms(50000);
	*/
    return 0;
}

void ADCInit(void)
{
	// Povolit hodinovy signal pro ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Zakazeme preruseni, nastavime kanal 31 = A/D prevodnik vypnut, jinak by zapisem
	// doslo ke spusteni prevodu
	// Vybereme single-ended mode
	ADC0->SC1[0] =  ADC_SC1_ADCH(31);

	// Vyber hodinoveho signalu, preddelicky a rozliseni
	// Clock pro ADC nastavime <= 4 MHz, coz je doporuceno pro kalibraci.
	// Pri max. CPU frekvenci 48 MHz je bus clock 24 MHz, pri delicce = 8
	// bude clock pro ADC 3 MHz
	ADC0->CFG1 = ADC_CFG1_ADICLK(0)		/* ADICLK = 0 -> bus clock */
		| ADC_CFG1_ADIV(3)				/* ADIV = 3 -> clock/8 */
		| ADC_CFG1_MODE(2);				/* MODE = 2 -> rozliseni 10-bit */

	// Do ostatnich registru zapiseme vychozi hodnoty:
	// Vybereme sadu kanalu "a", vychozi nejdelsi cas prevodu (24 clocks)
	ADC0->CFG2 = 0;

	// Softwarove spousteni prevodu, vychozi reference
	ADC0->SC2 = 0;

	// Hardwarove prumerovani vypnuto
	ADC0->SC3 = 0;	/* default values, no averaging */

}

void switch_init()
{
	pinMode(SW1, INPUT_PULLUP);
	pinMode(SW2, INPUT_PULLUP);
	pinMode(SW3, INPUT_PULLUP);
	pinMode(SW4, INPUT_PULLUP);
}

int switch1_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW1) == LOW)
	{
		SYSTICK_delay_ms(200);
		if(pinRead(SW1) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch2_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW2) == LOW)
	{
		SYSTICK_delay_ms(200);
		if(pinRead(SW2) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch3_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW3) == LOW)
	{
		SYSTICK_delay_ms(200);
		if(pinRead(SW3) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

int switch4_read()
{
	int switch_state = SWITCH_NOT_PRESSED;
	if(pinRead(SW4) == LOW)
	{
		SYSTICK_delay_ms(200);
		if(pinRead(SW4) == LOW)
		{
			switch_state = SWITCH_PRESSED;
		}
	}

	return switch_state;
}

void TaskSwithces(int SW)
{
	static int lastState;
	int swState = switch1_read();
	if(swState == SWITCH_PRESSED && lastState == SWITCH_NOT_PRESSED){
		zapnuto = !zapnuto;
	}
	lastState = swState;

}

uint32_t ADCCalibrate(void)
{
	 unsigned short cal_var;

	  ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;	/* Enable Software Conversion Trigger for Calibration Process */
	  ADC0->SC3 &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK );    /* set single conversion, clear avgs bitfield for next writing */

	  ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(32) ); /* turn averaging ON and set desired value */

	  ADC0->SC3 |= ADC_SC3_CAL_MASK;      /* Start CAL */

	  /* Wait calibration end */
	  while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 )
		  ;

	  /* Check for Calibration fail error and return */
	  if ( (ADC0->SC3 & ADC_SC3_CALF_MASK) != 0 )
		  return 1;

	  // Calculate plus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLP0;
	  cal_var += ADC0->CLP1;
	  cal_var += ADC0->CLP2;
	  cal_var += ADC0->CLP3;
	  cal_var += ADC0->CLP4;
	  cal_var += ADC0->CLPS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->PG = ADC_PG_PG(cal_var);

	  // Calculate minus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLM0;
	  cal_var += ADC0->CLM1;
	  cal_var += ADC0->CLM2;
	  cal_var += ADC0->CLM3;
	  cal_var += ADC0->CLM4;
	  cal_var += ADC0->CLMS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->MG = ADC_MG_MG(cal_var);

	  ADC0->SC3 &= ~ADC_SC3_CAL_MASK;

	  return 0;
}

void TaskPracuj() {

	switch (stav) {
	        case 10: // MEZI meta-game case <3
	            ONOtoBEZI = false;
	            stav = 0;
	            break;
	        case 0: // start

	        	LCD_set_cursor(3,1);
	        	LCD_puts("Stav: START");
	        	teplota = PRACKA_temperatureSetUpDisplay(teplota, boost);

	        	if((switch2_read() == SWITCH_PRESSED) && (boost == false))
	        	{
	        		boost = true;
	        	}
	        	if((switch2_read() == SWITCH_PRESSED) && (boost == true))
	        	{
	        		boost = false;
	        	}


	        	if(switch1_read() == SWITCH_PRESSED && (ONOtoBEZI == false))
	        	{
	        		ONOtoBEZI = true;
	        	}


	            //TODO dodelat nastaveni
	            //TODO nastavení Teplota,Boost
	            //TODO Display Teplota;Boost;STAV
	            if(ONOtoBEZI){
	                stav = NAPOUSTENI;
	                PRACKA_DisplayOutput(teplota, boost);
	                LCD_set_cursor(3,1);
	                LCD_puts("Stav: NAPOUSTENI");
	            }

	            break;
	        case NAPOUSTENI:

	        ONOtoBEZI = PRACKA_Bezi(ONOtoBEZI);

	        if(ONOtoBEZI){

	            hladina = PRACKA_CtiHladinu();
	            startWaitNapousteni = SYSTICK_millis();
	            PRACKA_NastavNapousteni(true);
	            if(hladina == 50) //TODO test co se stane na 100%, jen pokud bude cas
	            {
	                PRACKA_NastavNapousteni(false);
	                stopWaitNapousteni = SYSTICK_millis();
	                wodaCas = startWaitNapousteni - stopWaitNapousteni;
	                stav = OHREV;
	                PRACKA_DisplayOutput(teplota, boost);
	                LCD_set_cursor(3,1);
	                LCD_puts("Stav: OHREV");
	            }
	        } else {

	            PRACKA_NastavNapousteni(false);

	            PRACKA_DisplayOutput(teplota, boost);
	            LCD_set_cursor(3,1);
	            LCD_puts("Stav: VYCERPANI");
	            stav = VYCERPANI;
	        }


	            break;
	        case OHREV:

	        	ONOtoBEZI = PRACKA_Bezi(ONOtoBEZI);

	        if(ONOtoBEZI){

	            teplo = PRACKA_CtiTeplotu();
	            PRACKA_NastavTopeni(true);
	            if(teplo == teplota)
	            {
	                PRACKA_NastavTopeni(false);
	                stav = TOCENI;

	                PRACKA_DisplayOutput(teplota, boost);
	                LCD_set_cursor(3,1);
	                LCD_puts("Stav: TOCENI");
	            }
	        } else {
	            //TODO Nastaveni displaye
	            PRACKA_NastavTopeni(false);

	            PRACKA_DisplayOutput(teplota, boost);
	            LCD_set_cursor(3,1);
	            LCD_puts("Stav: VYCERPANI");
	            stav = VYCERPANI;
	        }

	        	break;


	        case TOCENI:


	            waitTime = SYSTICK_millis();
	            while(pocetCyklu > 0){
	            	ONOtoBEZI = PRACKA_Bezi(ONOtoBEZI);
	                if(ONOtoBEZI){



	                    PRACKA_NastavBuben(smer, boost);


	                    int curentTime = SYSTICK_millis();
	                    if(curentTime - waitTime >= 1000 ){
	                        pocetCyklu--;
	                        smer = !smer;
	                        waitTime = SYSTICK_millis();
	                    }
	                }
	                else{
	                    //TODO OK udìlat funkci PRACKA_VYPNOUTbuben();
	                    PRACKA_VYPNOUTbuben();

	                    PRACKA_DisplayOutput(teplota, boost);
	                    LCD_set_cursor(3,1);
	                    LCD_puts("Stav: VYCERPANI");
	                    stav = VYCERPANI;
	                }

	            }
	            //TODO OK udìlat funkci PRACKA_VYPNOUTbuben();
	            //sem hodit funkci na vypnutí
	            PRACKA_VYPNOUTbuben();
	            startWaitVypousteni = SYSTICK_millis();
	            stav = VYCERPANI;

	            PRACKA_DisplayOutput(teplota, boost);
	            LCD_set_cursor(3,1);
	            LCD_puts("Stav: VYCERPANI");
	            break;

	        case VYCERPANI:
	        //TODO Nastaveni displaye

	            hladina = PRACKA_CtiHladinu();
	            stopWaitVypousteni = SYSTICK_millis();
	            PRACKA_NastavCerpadlo(true);
	            if(hladina == 0 && (stopWaitVypousteni - startWaitVypousteni) >= wodaCas)
	            {
	                PRACKA_NastavCerpadlo(false);
	                stav = 10;
	            }


	            break;

	    }


}




////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
