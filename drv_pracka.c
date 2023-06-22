/*
 * drv_pracka.c
 *
 *  Created on: 2. 11. 2022
 *      Author: student
 */
#include "MKL25Z4.h"
#include "drv_pracka.h"

//VYSTUPY
#define BUBEN_VLEVO_PIN		1	// PTC1
#define VYPUSTENI_PIN		0	// PTD0
#define	NAPUSTENI_PIN		0	// PTE0
#define	TOPENI_PIN			1 	// PTE1
#define OTACKY_PIN			4	// PTE4
#define	BUBEN_VPRAVO_PIN	5	// PTE5
#define SWITCH_PRESSED		(1)
#define SWITCH_NOT_PRESSED	(0)

//VSTUPY
#define TEPLOTA_90_PIN		5	// PTC5
#define HLADINA_50_PIN		6	// PTC6
#define TEPLOTA_40_PIN		16	// PTC16
#define TEPLOTA_60_PIN		2	// PTD2
#define TEPLOTA_30_PIN		3	// PTD3
#define HLADINA_100_PIN		4	// PTD4

void PRACKA_init()
{
	// Enable clock for ports A, B, C, D, E
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
	SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK );

	// Set pin function to GPIO

	PORTC->PCR[BUBEN_VLEVO_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[VYPUSTENI_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[NAPUSTENI_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[TOPENI_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[OTACKY_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[BUBEN_VPRAVO_PIN] = PORT_PCR_MUX(1);

	PORTC->PCR[TEPLOTA_90_PIN] = PORT_PCR_MUX(1);
	PORTC->PCR[HLADINA_50_PIN] = PORT_PCR_MUX(1);
	PORTC->PCR[TEPLOTA_40_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[TEPLOTA_60_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[TEPLOTA_30_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[HLADINA_100_PIN] = PORT_PCR_MUX(1);


	// Set pin direction output/input
	PTC->PDDR |= (1 << BUBEN_VLEVO_PIN); // output
	PTD->PDDR |= (1 << VYPUSTENI_PIN);
	PTE->PDDR |= (1 << NAPUSTENI_PIN);
	PTE->PDDR |= (1 << TOPENI_PIN);
	PTE->PDDR |= (1 << OTACKY_PIN);
	PTE->PDDR |= (1 << BUBEN_VPRAVO_PIN);

	PTC->PDDR &= ~(1 << TEPLOTA_90_PIN); //input
	PTC->PDDR &= ~(1 << HLADINA_50_PIN);
	PTC->PDDR &= ~(1 << TEPLOTA_40_PIN);
	PTD->PDDR &= ~(1 << TEPLOTA_60_PIN);
	PTD->PDDR &= ~(1 << TEPLOTA_30_PIN);
	PTD->PDDR &= ~(1 << HLADINA_100_PIN);

	PORTA->PCR[HLADINA_100_PIN] |= (PORT_PCR_PS_MASK | PORT_PCR_PE_MASK);
	PORTA->PCR[HLADINA_50_PIN] |= (PORT_PCR_PS_MASK | PORT_PCR_PE_MASK);

}

void PRACKA_NastavNapousteni(bool zapni)
{
	// Zapne napousteni vody
	if(zapni)
	{
		PTE->PSOR |= (1 << NAPUSTENI_PIN);

	}
	// Vypne napousteni vody
	else {
		PTE->PCOR |= (1 << NAPUSTENI_PIN);
	}


}

void PRACKA_NastavTopeni(bool zapni)
{
	//zapne topeni
	if (zapni) {
		PTE->PSOR |= (1 << TOPENI_PIN);
	}
	// vypne topeni
	else {
		PTE->PCOR |= (1 << TOPENI_PIN);
	}

}

void PRACKA_NastavBuben(bool smer, bool boost)	{
	// zapne otaceni doleva
	if (smer) {
		PTE->PCOR |= (1 << BUBEN_VPRAVO_PIN);
		PTC->PSOR |= (1 << BUBEN_VLEVO_PIN);
	}
	// zapne otaceni doprava
	else {
		PTC->PCOR |= (1 << BUBEN_VLEVO_PIN);
		PTE->PSOR |= (1 << BUBEN_VPRAVO_PIN);
	}
	// zvysi otacky
	if(boost){
		PTE->PSOR |= (1 << OTACKY_PIN);
	}
	// snizi otacky
	else
	{
		PTE->PCOR |= (1 << OTACKY_PIN);
	}

}

void PRACKA_VYPNOUTbuben() {
	PTC->PCOR |= (1 << BUBEN_VLEVO_PIN);
	PTE->PCOR |= (1 << BUBEN_VPRAVO_PIN);
}

void PRACKA_NastavCerpadlo(bool zapni) {
	//zapne vypousteni
	if (zapni) {
		PTD->PSOR |= (1 << VYPUSTENI_PIN);
	}
	// vypne vypousteni
	else {
		PTD->PCOR |= (1 << VYPUSTENI_PIN);
	}
}

int PRACKA_CtiTeplotu() {

	if((PTC->PDIR & (1 << TEPLOTA_90_PIN)) != 0) {
		return 90;
	}
	else if((PTD->PDIR & (1 << TEPLOTA_60_PIN)) != 0) {
		return 60;
	}
	else if((PTC->PDIR & (1 << TEPLOTA_40_PIN)) != 0) {
		return 40;
	}
	else if((PTD->PDIR & (1 << TEPLOTA_30_PIN)) != 0) {
		return 30;
	}
	else {
		return 0;
	}
}


int PRACKA_CtiHladinu() {

	if((PTD->PDIR & (1 << HLADINA_100_PIN)) != 0) {
		return 100;
	}
	else if((PTC->PDIR & (1 << HLADINA_50_PIN)) != 0) {
		return 50;
	}
	else {
		return 0;
	}
}

int PRACKA_temperatureSetUpDisplay(teplota, boost)
{
	ADC0->SC1[0] = ADC_SC1_ADCH(11);
	while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 );

	// Ulozime vysledek prevodu
	uint16_t vysledek = ADC0->R[0];
	teplota = ((int)vysledek+1) / 204.8;

	if (teplota < 204) {
		teplota = 0;
	}

	if (vysledek > 204 && vysledek < 410) {
		teplota = 30;
	}

	if (vysledek > 410 && vysledek < 615) {
		teplota = 40;
	}

	if (vysledek > 615 && vysledek < 820) {
		teplota = 60;
	}

	if (vysledek > 820) {
		teplota = 90;
	}


	char str[30];
	sprintf(str, "Teplota: %d Celsia", teplota);
	LCD_clear();
	LCD_set_cursor(1,1);
	LCD_puts(str);

	if(boost){
		LCD_set_cursor(2,1);
		LCD_puts("Boost: ON");
	}
	else{
		LCD_set_cursor(2,1);
		LCD_puts("Boost: OFF");
	}




	return teplota;
}

void PRACKA_DisplayOutput(teplota, boost)
{
	LCD_clear();
		char str[30];
		sprintf(str, "Teplota: %d Celsia", teplota);
		LCD_set_cursor(1,1);
		LCD_puts(str);

		if(boost){
			LCD_set_cursor(2,1);
			LCD_puts("Boost: ON");
		}
		else{
			LCD_set_cursor(2,1);
			LCD_puts("Boost: OFF");
		}
}

bool PRACKA_Bezi(ONOtoBEZI)
{
	if(switch1_read() == SWITCH_PRESSED && (ONOtoBEZI == false))
	{
		return true;
	}
	if((switch1_read() == SWITCH_PRESSED) && (ONOtoBEZI == true))
	{
		return false;
	}
	else
	{
		return ONOtoBEZI;
	}

}

