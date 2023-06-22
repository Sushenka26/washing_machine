/*
 * drv_pracka.h
 *
 *  Created on: 2. 11. 2022
 *      Author: student
 */

#ifndef SOURCES_DRV_PRACKA_H_
#define SOURCES_DRV_PRACKA_H_

#include <stdbool.h>
#include "drv_lcd.h"

void PRACKA_init();

void PRACKA_NastavNapousteni(bool zapni);
void PRACKA_NastavTopeni(bool zapni);
void PRACKA_NastavBuben(bool smer, bool boost);
void PRACKA_NastavCerpadlo(bool zapni);
void PRACKA_VYPNOUTbuben();
int PRACKA_CtiTeplotu();
int PRACKA_CtiHladinu();

bool PRACKA_Bezi(ONOtoBEZI);

int PRACKA_temperatureSetUpDisplay(teplota, boost);

void PRACKA_DisplayOutput(teplota, boost);


#endif /* SOURCES_DRV_PRACKA_H_ */
