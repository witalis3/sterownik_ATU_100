/*
 * sterownik_ATU_100.h
 *
 *  Created on: 8 sty 2021
 *      Author: witek
 *      na bazie ATU-100 N7DDC
 */
#include "Arduino.h"

#ifndef STEROWNIK_ATU_100_H_
#define STEROWNIK_ATU_100_H_

#define DEBUG

#define TUNE_BUTTON_PIN		7
#define AUTO_BUTTON_PIN		5
#define BYPASS_BUTTON_PIN	6
#define MANUAL_BUTTON_PIN	A0	// BUTTON6
#define FWD_PIN				A7
#define REF_PIN				A6
#define SW_PIN				A1	// przełączanie kondensatorów przód tył IN/OUT
#define TX_REQUEST_PIN		11	// żądanie nadawania dla TRX
#define GREEN_LED_PIN		9
#define RED_LED_PIN			10
#define BAND0_PIN			8
#define BAND1_PIN			2
#define BAND2_PIN			3
#define BAND3_PIN			4

void tune();
void get_swr(void);
void atu_reset(void);
void get_pwr();
int get_reverse(void);
int get_forward(void);
int correction(int input);
void show_pwr(int Power, int SWR);
void set_ind(byte Ind);
void set_cap(byte Cap);
void set_sw(byte SW);
void lcd_ind();
void sub_tune();
void coarse_tune();
void sharp_ind();
void coarse_cap();
void sharp_cap();
void sharp_ind();
void lcd_prep();
void led_wr_str(byte row, byte col, char * lan, byte len);
unsigned int get_indu_nH(byte indu);
unsigned int get_capa_pF(byte capa);
void lcd_pwr();
void lcd_swr(int swr);
void dysp_on();
void dysp_off();
void button_delay();
bool button_pressed();
void led_init();
void read_i2c_inputs();
void load_settings();
void button_proc(void);
void show_reset();
void btn_push();
void button_proc_test(void);
void Test_init(void);
void cells_init(void);
uint8_t Bcd2Dec(uint8_t n);
void tune_zapis();

#endif /* STEROWNIK_ATU_100_H_ */
