/*
 * sterownik_ATU_100.h
 *
 *  Created on: 8 sty 2021
 *      Author: witek
 */
#include "Arduino.h"

#ifndef STEROWNIK_ATU_100_H_
#define STEROWNIK_ATU_100_H_

#define TUNE_BUTTON_PIN	11
#define FWD_PIN			A6
#define REF_PIN			A7
#define SW_PIN			A1	// przełączanie kondensatorów przód tył IN/OUT

void tune();
void get_swr(void);
void atu_reset(void);
void get_pwr();
int get_reverse(void);
int get_forward(void);
int correction(int input);
void show_pwr(int Power, int SWR);
void set_ind(char Ind);
void set_cap(char Cap);
void set_sw(char SW);
void lcd_ind();
void sub_tune();
void coarse_tune();
void sharp_ind();
void coarse_cap();
void sharp_cap();
void sharp_ind();

#endif /* STEROWNIK_ATU_100_H_ */
