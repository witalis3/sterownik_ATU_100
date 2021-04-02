/*
 *  Created on: 8 sty 2021
 *      Author: witek
 * sterownik ATU na bazie ATU-100 wg N7DDC na procesor atmega328
 * SP3JDZ
 *
 * ToDo
 * - opcja 7x7 lub 7x8 do wyboru
 * - pojawia się dwa razy reset po resecie ;-)
 * - inne błędy...
 *
 */
#include "Arduino.h"
#include "sterownik_ATU_100.h"
#include <EEPROM.h>


#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD I2C address

#include <Adafruit_MCP23008.h>
Adafruit_MCP23008 mcp_c;	// expander dla kondensatorów; sub adres 0x0
Adafruit_MCP23008 mcp_l;	// expander dla cewek; sub adres 0x1

#include "Bounce2.h"
Bounce2::Button tune_button = Bounce2::Button();
Bounce2::Button auto_button = Bounce2::Button();
Bounce2::Button bypass_button = Bounce2::Button();
Bounce2::Button manual_button = Bounce2::Button();

byte rready = 0, p_cnt = 0, lcd_prep_short = 0, Auto;
byte type = 1, Soft_tune = 0;	// 1602
char work_char, work_str[7], work_str_2[7];
byte Test = 0, Restart = 0, Loss_mode = 0, Fid_loss;
int Power = 0, Power_old = 10000;
int SWR_old = 10000;
int P_max, SWR, PWR, swr_a, work_int;
int SWR_fixed_old = 0;
int Cap1, Cap2, Cap3, Cap4, Cap5, Cap6, Cap7, Cap8;
//int Cap1 = 10, Cap2 = 22, Cap3 = 47, Cap4 = 100, Cap5 = 220, Cap6 = 470, Cap7 = 1000, Cap8 = 1820;
int Ind1, Ind2, Ind3, Ind4, Ind5, Ind6, Ind7, Ind8;
byte Dysp_delay = 0;
int dysp_cnt = 0;
float dysp_cnt_mult = 2.3;
int Auto_delta;
byte L = 1, but = 0;
byte ind = 0, cap = 0, SW = 0, step_cap = 0, step_ind = 0, L_linear = 0, C_linear = 0, L_q = 7, C_q = 8, Overload = 0,
		D_correction = 1, K_Mult, P_High = 1, L_invert = 0, Loss_ind = 0;
byte L_mult = 4, C_mult = 8;		// ustawione na 7X8
// Rel_Del opóźnienie przekaźnika
int Rel_Del = 30, min_for_start, max_for_start, max_swr = 0;
byte mem_offset = 0;
byte offset;
byte bypas = 0, cap_mem = 0, ind_mem = 0, SW_mem = 0, Auto_mem = 0;

union swaper
{
	byte bajt;
	struct
	{
		unsigned char b0 :1;
		unsigned char b1 :1;
		unsigned char b2 :1;
		unsigned char b3 :1;
		unsigned char b4 :1;
		unsigned char b5 :1;
		unsigned char b6 :1;
		unsigned char b7 :1;
	} bit;
};

void setup()
{
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("setup poczatek");
#endif
	cells_init();
	pinMode(SW_PIN, OUTPUT);
	digitalWrite(SW_PIN, SW);
	pinMode(TX_REQUEST_PIN1, OUTPUT);
	digitalWrite(TX_REQUEST_PIN1, LOW);		// TX request; stan aktywny wysoki
	pinMode(TX_REQUEST_PIN2, OUTPUT);
	digitalWrite(TX_REQUEST_PIN2, LOW);		// TX request; stan aktywny wysoki + opóźnienie
	pinMode(MANUAL_LED_PIN, OUTPUT);
	digitalWrite(MANUAL_LED_PIN, LOW);		// 	sygnalizacja trybu ręcznego; stan aktywny wysoki
	// LEDy jako wyświetlacz:
	pinMode(GREEN_LED_PIN, OUTPUT);
	digitalWrite(GREEN_LED_PIN, HIGH);
	pinMode(RED_LED_PIN, OUTPUT);
	digitalWrite(RED_LED_PIN, HIGH);
	// band data:
	pinMode(BAND0_PIN, INPUT_PULLUP);
	pinMode(BAND1_PIN, INPUT_PULLUP);
	pinMode(BAND2_PIN, INPUT_PULLUP);
	pinMode(BAND3_PIN, INPUT_PULLUP);
	// przyciski:
	tune_button.attach(TUNE_BUTTON_PIN, INPUT_PULLUP);
	tune_button.setPressedState(LOW);
	tune_button.interval(50);
	auto_button.attach(AUTO_BUTTON_PIN, INPUT_PULLUP);
	auto_button.setPressedState(LOW);
	auto_button.interval(50);
	bypass_button.attach(BYPASS_BUTTON_PIN, INPUT_PULLUP);
	bypass_button.setPressedState(LOW);
	bypass_button.interval(50);
	manual_button.attach(MANUAL_BUTTON_PIN, INPUT_PULLUP);
	manual_button.setPressedState(LOW);
	manual_button.interval(50);

	dysp_cnt = Dysp_delay * dysp_cnt_mult;
    //
    delay(300);
    if ((digitalRead(BYPASS_BUTTON_PIN) == 0) && (digitalRead(AUTO_BUTTON_PIN) == 0))
    { // Test mode
        Test = 1;
        Auto = 0;
    }
    if (L_q == 5)
        L_mult = 1;
    else if (L_q == 6)
        L_mult = 2;
    else if (L_q == 7)
        L_mult = 4;
    if (C_q == 5)
        C_mult = 1;
    else if (C_q == 6)
        C_mult = 2;
    else if (C_q == 7)
        C_mult = 4;
    else if (C_q == 8)		// 8 kondensatorów
    	C_mult = 8;
    //
    delay(300);
    led_init();

    // expandery do obsługi przekaźników
    mcp_l.begin(1);
	mcp_l.writeGPIO(0x0);		// wszystkie przekaźniki wyłączone
	for (int var = 0; var < 8; ++var)
	{
		mcp_l.pinMode(var, OUTPUT);
	}
	//mcp_l.write8(MCP23008_IODIR, 0);	// wszystkie piny jako wyjścia
	mcp_c.begin(0);
	mcp_c.writeGPIO(0x0);		// wszystkie przekaźniki wyłączone
	for (int var = 0; var < 8; ++var)
	{
		mcp_c.pinMode(var, OUTPUT);
	}
	//mcp_c.write8(MCP23008_IODIR, 0);	// wszystkie piny jako wyjścia

    if (Test == 0)
    {
        read_i2c_inputs();
        load_settings();
        if (Restart == 1)
            lcd_prep_short = 1;
        lcd_prep();
    }
    else
    {
        Test_init();
    }
    lcd_ind();
}

void loop()
{
    lcd_pwr();
    manual_button.update();
    if (manual_button.pressed())
    {
    	if (Test == 0)
    	{
    		digitalWrite(MANUAL_LED_PIN, HIGH);
            digitalWrite(TX_REQUEST_PIN1, HIGH);
    		Test = 1;
    		lcd.setCursor(8, 0);
    		lcd.print("l");
    		lcd_prep_short = 1;
    		lcd_prep();
    	}
    	else if (Test == 1)
    	{
    		digitalWrite(MANUAL_LED_PIN, LOW);
            digitalWrite(TX_REQUEST_PIN1, LOW);
    		Test = 0;
    		lcd.setCursor(8, 0);
    		lcd.print(" ");
    		tune_zapis();
    	}
    }
    if (Test == 0)
    {
        button_proc();	// główna procedura
    }
    else
    {
        button_proc_test();
    }
	if (dysp_cnt != 0)
        dysp_cnt--;
    else if ((Test == 0) && (Dysp_delay != 0))
        dysp_off();
    // memo_code
    offset = mem_offset;
    read_i2c_inputs();
    if (offset != mem_offset)
    {
#ifdef DEBUG
		Serial.print("mem_offset: ");
		Serial.println(mem_offset, HEX);
#endif
        load_settings();
        lcd_ind();
    }
    // end memo_code
}

void tune()
{
    p_cnt = 0;
    P_max = 0;
    //
    rready = 0;
#ifdef DEBUG
    Serial.print("tune:");
#endif
    get_swr();
    if (SWR < 110)
        return;
    atu_reset();
    if (Loss_ind == 0)
        lcd_ind();                // wyświetlenie wartości L i C
    delay(50);
    get_swr();
    swr_a = SWR;
    if (SWR < 110)
        return;
    if ((max_swr > 110) && (SWR > max_swr))
    	// max_swr - zawartość komórki 9 (domyślnie 0)
        return;
    //

    sub_tune();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    if (SWR < 120)
        return;
    if ((C_q == 5) & (L_q == 5))
        return;

    if (L_q > 5)
    {
        step_ind = L_mult;
        L_mult = 1;
        sharp_ind();
        set_multis();	// ustawienie mult do pierwotnej wartości
    }
    if (SWR < 120)
        return;

    if (C_q > 5)
    {
        step_cap = C_mult; // = C_mult
        C_mult = 1;
        sharp_cap();
        set_multis();	// ustawienie mult do pierwotnej wartości
    }
    // ToDo po co to? -> powrót do pierwotnych wartości?
    return;
}
void get_swr()
{
    get_pwr();
    if (p_cnt != 100)	// raz na 100 pomiarów pokazuje maksymalną wartość mocy
    {
        p_cnt += 1;
        if (PWR > P_max)
            P_max = PWR;
    }
    else
    {
        p_cnt = 0;
        show_pwr(P_max, SWR);
        P_max = 0;
    }
    while ((PWR < min_for_start) || ((PWR > max_for_start) && (max_for_start > 0)))
    { // waiting for good power

        get_pwr();
        if (p_cnt != 100)
        {
            p_cnt += 1;
            if (PWR > P_max)
                P_max = PWR;
        }
        else
        {
            p_cnt = 0;
            show_pwr(P_max, SWR);
            P_max = 0;
        }
        tune_button.update();
        if (tune_button.read() == 1)
            rready = 1;
        if ((rready == 1) && tune_button.pressed())
        { //  press button  Tune
            show_reset();
            SWR = 0;	// wskaźnik przerwania oczekiwania na właściwą moc - reset
            return;
        }
    } //  good power
    return;
}
void atu_reset()
{
    ind = 0;
    cap = 0;
    set_ind(ind);
    set_cap(cap);
#ifdef DEBUG
    Serial.print("atu_reset:SW:");
    Serial.print(SW);
    Serial.print(':');
#endif
    delay(Rel_Del);
}
void get_pwr()
{
    long Forward, Reverse;
    float p;
    //
    Forward = get_forward();
    Reverse = get_reverse();
#ifdef DEBUGi
    if (Forward > 0)
    {
    Serial.print("Forward: ");
    Serial.println(Forward);
    }
    if (Reverse > 0)
    {
    Serial.print("Reverse: ");
    Serial.println(Reverse);
    }
#endif
    if (D_correction == 1)
        p = correction(Forward * 3);
    else
        p = Forward * 3;
#ifdef DEBUGi
    if (p > 0)
    {
    Serial.print("p: ");
    Serial.println(p);
    }
#endif

    if (Reverse >= Forward)
        Forward = 999;
    else
    {
        Forward = ((Forward + Reverse) * 100) / (Forward - Reverse);
        if (Forward > 999)
            Forward = 999;
    }
    //
    p = p * K_Mult / 1000.0; // mV to Volts on Input
    p = p / 1.414;
    if (P_High == 1)
        p = p * p / 50; // 0 - 1500 ( 1500 Watts)
    else
        p = p * p / 5; // 0 - 1510 (151.0 Watts)
    p = p + 0.5; // rounding to 0.1 W
    //
    PWR = p;
#ifdef DEBUGi
    if (PWR > 0)
    {
    Serial.print("PWR: ");
    Serial.println(PWR);
    }
#endif
    if (PWR < 10)
        SWR = 1;
    else if (Forward < 100)
        SWR = 999;
    else
        SWR = Forward;
#ifdef DEBUG
    if (PWR > 50)
    {
        Serial.print("SWR: ");
        Serial.println(SWR);
    }
#endif
    return;
}
int get_forward()
{
    int forward;
    forward = analogRead(FWD_PIN);
    if (forward > 1000)
        Overload = 1;
    else
        Overload = 0;
    return forward * 4.883; // zwraca napięcie w mV
}
int get_reverse()
{
    return analogRead(REF_PIN) * 4.883; // zwraca napięcie w mV
}
int correction(int input)
{
    if (input <= 80)
        return 0;
    if (input <= 171)
        input += 244;
    else if (input <= 328)
        input += 254;
    else if (input <= 582)
        input += 280;
    else if (input <= 820)
        input += 297;
    else if (input <= 1100)
        input += 310;
    else if (input <= 2181)
        input += 430;
    else if (input <= 3322)
        input += 484;
    else if (input <= 4623)
        input += 530;
    else if (input <= 5862)
        input += 648;
    else if (input <= 7146)
        input += 743;
    else if (input <= 8502)
        input += 800;
    else if (input <= 10500)
        input += 840;
    else
        input += 860;
    //
    return input;
}
void show_pwr(int Power, int SWR)
{
    float a, b;
    int p_ant;
    float eff;
    if ((Test == 0) && (Loss_ind == 1) && (SWR >= 100))
    {
        if (Loss_mode == 0)
        { // prepare
            if (type == 4 || type == 5)
            { // 128*64 OLED
                if (P_High == 1)
                    led_wr_str(4, 16, "ANT=  0W", 8);
                else
                    led_wr_str(4, 16, "ANT=0.0W", 8);
                led_wr_str(6, 16, "EFF=  0%", 8);
            }
            else if (type == 2 || type == 3)
            { // 128*32 OLED
                if (P_High == 1)
                    led_wr_str(0, 9, "ANT=  0W", 8);
                else
                    led_wr_str(0, 9, "ANT=0.0W", 8);
                led_wr_str(1, 9, "EFF=  0%", 8);
            }
            else if (type == 1)
            { // 1602 LCD
                if (P_High == 1)
                    led_wr_str(0, 9, "AN=  0W", 7);
                else
                    led_wr_str(0, 9, "AN=0.0W", 7);
                led_wr_str(1, 9, "EFF= 0%", 7);
            }
        }
        Loss_mode = 1;
    }
    else
    {
        if (Loss_mode == 1)
            lcd_ind();
        Loss_mode = 0;
    }

    if (Power != Power_old)
    {
        Power_old = Power;
        //
        if (P_High == 0)
        {
            if (Power >= 100)
            { // > 10 W
                Power += 5; // rounding to 1 W
        		itoa(Power, work_str, 10);
                //IntToStr(Power, work_str);
                work_str_2[0] = work_str[0];
                work_str_2[1] = work_str[1];
                work_str_2[2] = work_str[2];
                work_str_2[3] = 'W';
            }
            else
            {
        		itoa(Power, work_str, 10);
                //IntToStr(Power, work_str);
                if (work_str[4] != ' ')
                    work_str_2[0] = work_str[4];
                else
                    work_str_2[0] = '0';
                work_str_2[1] = '.';
                if (work_str[5] != ' ')
                    work_str_2[2] = work_str[5];
                else
                    work_str_2[2] = '0';
                work_str_2[3] = 'W';
            }
        }
        else
        { // High Power
        	if (Power >= 1000)
        	{
        		sprintf(work_str,"%4u", Power);
        	}
        	else
        	{
        		if (Power >=100)
        		{
        			sprintf(work_str,"%3uW", Power);
        		}
        		else if (Power >= 10)
        		{
        			sprintf(work_str," %2uW", Power);
        		}
        		else
        		{
        			sprintf(work_str,"  %1uW", Power);
        		}
        	}
#ifdef DEBUGi
        		Serial.print("Power_str: _");
        		Serial.print(work_str);
        		Serial.println('_');
#endif
        }

        if ((type == 4) || (type == 5))
            led_wr_str(0, 16 + 4 * 12, work_str_2, 4); // 128*64 OLED
        else if (type != 0)
        {
        	// 1602  & 128*32
            lcd.setCursor(4, 0);
            lcd.print(work_str);
        }
        //  Loss indication
        if (Loss_mode == 1)
        {
            if (ind == 0 && cap == 0)
                swr_a = SWR;
            a = 1.0 / ((swr_a / 100.0 + 100.0 / swr_a) * Fid_loss / 10.0 * 0.115 + 1.0); // Fider loss
            b = 4.0 / (2.0 + SWR / 100.0 + 100.0 / SWR); // SWR loss
            if (a >= 1.0)
                a = 1.0;
            if (b >= 1.0)
                b = 1.0;
            p_ant = Power * a * b;
            eff = a * b * 100;
            if (eff >= 100)
                eff = 99;
            //
            if (P_High == 0)
            {
                if (p_ant >= 100)
                { // > 10 W
                    p_ant += 5; // rounding to 1 W
            		itoa(p_ant, work_str, 10);
                    //IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[2];
                    work_str_2[1] = work_str[3];
                    work_str_2[2] = work_str[4];
                    work_str_2[3] = 'W';
                }
                else
                {
            		itoa(p_ant, work_str, 10);
                    //IntToStr(p_ant, work_str);
                    if (work_str[4] != ' ')
                        work_str_2[0] = work_str[4];
                    else
                        work_str_2[0] = '0';
                    work_str_2[1] = '.';
                    if (work_str[5] != ' ')
                        work_str_2[2] = work_str[5];
                    else
                        work_str_2[2] = '0';
                    work_str_2[3] = 'W';
                }
            }
            else
            { // High Power
                if (p_ant < 999) { // 0 - 1500 Watts
            		itoa(p_ant, work_str, 10);
                    //IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[3];
                    work_str_2[1] = work_str[4];
                    work_str_2[2] = work_str[5];
                    work_str_2[3] = 'W';
                }
                else
                {
            		itoa(p_ant, work_str, 10);
                    //IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[2];
                    work_str_2[1] = work_str[3];
                    work_str_2[2] = work_str[4];
                    work_str_2[3] = work_str[5];
                }
            }
            if (type == 4 || type == 5)
                led_wr_str(4, 16 + 4 * 12, work_str_2, 4); // 128*64 OLED
            else if (type == 2 || type == 3)
                led_wr_str(0, 13, work_str_2, 4); // 128*32
            else if (type == 1)
                led_wr_str(0, 12, work_str_2, 4); // 1602
            //
    		itoa(eff, work_str, 10);
            //IntToStr(eff, work_str);
            work_str_2[0] = work_str[4];
            work_str_2[1] = work_str[5];
            if (type == 4 || type == 5)
                led_wr_str(6, 16 + 5 * 12, work_str_2, 2);
            else if (type == 2 || type == 3)
                led_wr_str(1, 14, work_str_2, 2);
            else if (type == 1)
                led_wr_str(1, 13, work_str_2, 2);
        }
    }
    return;
}
void set_ind(byte Ind)
{
#ifdef DEBUG
	Serial.print("Ind: ");
	Serial.print(Ind, HEX);
	Serial.print(':');
	Serial.println(get_indu_nH(Ind));
#endif
    if (L_invert == 0)
    {
    	mcp_l.writeGPIO(Ind);	// ToDo sprawdzić poprawność wysyłania całego bajtu
    }
    else
    {
    	mcp_l.writeGPIO(~Ind);	// ToDo sprawdzić poprawność negacji całego bajtu
    }
    delay(Rel_Del);
}
void set_cap(byte Cap)
{
#ifdef DEBUG
	Serial.print("Cap: ");
	Serial.print(Cap, HEX);
	Serial.print(':');
	Serial.println(get_capa_pF(Cap));
#endif
	mcp_c.writeGPIO(Cap);	// ToDo sprawdzić poprawność wysyłania całego bajtu
    delay(Rel_Del);
}
/*
 * wyświetlenie wartości L i C
 */
void lcd_ind()
{
	byte wiersz;
	// wyświetlanie indukcyjności:
    int indu, indu_sub;
	work_int = get_indu_nH(ind);
	if (work_int > 9999)
	{ // more then 9999 nH
	    indu = work_int / 1000;
	    indu_sub = (work_int % 1000)/100;
	    sprintf(work_str,"L=%2u.%01uu", indu, indu_sub);
	}
	else
	{
	    indu = work_int / 1000;
	    indu_sub = (work_int % 1000)/10;
	    sprintf(work_str,"L=%1u.%02uu", indu, indu_sub);
	}
	if ((type == 4) || (type == 5))
	{ // 128*64 OLED
		if (SW == 1)
			wiersz = 4;
		else
			wiersz = 6;
		led_wr_str(wiersz, 16, "L=", 2);
		led_wr_str(wiersz, 16 + 6 * 12, "uH", 2);
		led_wr_str(wiersz, 16 + 2 * 12, work_str_2, 4);
	}
	else if (type == 2 || type == 3)
	{ // 128*32 OLED
		if (SW == 1)
			wiersz = 0;
		else
			wiersz = 1;
		led_wr_str(wiersz, 9, "L=", 2);
		led_wr_str(wiersz, 15, "uH", 2);
		led_wr_str(wiersz, 11, work_str_2, 4);
	}
	else if (type == 1)
	{ //  1602 LCD
		if (SW == 1)
			wiersz = 0;
		else
			wiersz = 1;
		lcd.setCursor(9, wiersz);
		lcd.print(work_str);
	}
	// wyświetlanie pojemności:
	work_int = get_capa_pF(cap);
	if (work_int >= 1000)
	{
		sprintf(work_str,"C=%4up", work_int);
	}
	else if (work_int >= 100)
	{
		sprintf(work_str,"C= %3up", work_int);
	}
	else if (work_int >= 10)
	{
		sprintf(work_str,"C=  %2up", work_int);
	}
	else
	{
		sprintf(work_str,"C=   %1up", work_int);
	}
	if (type == 4 || type == 5)
	{ // 128*64 OLED
		if (SW == 1)
			wiersz = 6;
		else
			wiersz = 4;
		led_wr_str(wiersz, 16, "C=", 2);
		led_wr_str(wiersz, 16 + 6 * 12, "pF", 2);
		led_wr_str(wiersz, 16 + 2 * 12, work_str_2, 4);
	}
	else if (type == 2 || type == 3)
	{ // 128*32 OLED
		if (SW == 1)
			wiersz = 1;
		else
			wiersz = 0;
		led_wr_str(wiersz, 9, "C=", 2);
		led_wr_str(wiersz, 15, "pF", 2);
		led_wr_str(wiersz, 11, work_str_2, 4);
	}
	else if (type == 1)
	{ // 1602 LCD
		if (SW == 1)
			wiersz = 1;
		else
			wiersz = 0;
		lcd.setCursor(9, wiersz);
		lcd.print(work_str);
	}
	return;
}
void sub_tune()
{
    int swr_mem, ind_mem, cap_mem;
    //
    swr_mem = SWR;
#ifdef DEBUG
    Serial.print("sub_tune:swr_mem: ");
    Serial.println(swr_mem);
#endif
    coarse_tune();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    sharp_ind();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    sharp_cap();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    //
    /*
    if ((SWR < 200) && (SWR < swr_mem) && ((swr_mem - SWR) > 100))
        return;
        zależy mi na maksymalnie małym SWR a nie na szybkości
        */
    swr_mem = SWR;
    ind_mem = ind;
    cap_mem = cap;
    // przełączenie kondensatorów na drugą stronę i ponowne szukanie
    if (SW == 1)
        SW = 0;
    else
        SW = 1;
    atu_reset();
    set_sw(SW);
    delay(50);
    get_swr();
    if (SWR < 120)
        return;
    //
    coarse_tune();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    sharp_ind();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    sharp_cap();
    if (SWR == 0)
    {
        atu_reset();
        return;
    }
    get_swr();
    if (SWR < 120)
        return;
    //
    if (SWR > swr_mem)
    {
        if (SW == 1)
            SW = 0;
        else
            SW = 1;
        set_sw(SW);
        ind = ind_mem;
        cap = cap_mem;
        set_ind(ind);
        set_cap(cap);
        SWR = swr_mem;
    }
    return;
}
void coarse_tune()
{
    byte step = 3;
    byte count;
    byte mem_cap, mem_step_cap;
    int min_swr;

    mem_cap = 0;
    step_ind = step;
    mem_step_cap = 3;
    min_swr = SWR + SWR / 20;
#ifdef DEBUG
    Serial.print("coarse_tune[L]:min_swr: ");
    Serial.println(min_swr);
#endif
    for (count = 0; count <= 31;)
    {
#ifdef DEBUG
        Serial.print("coarse_tune[L]:count: ");
        Serial.println(count);
#endif
        set_ind(count * L_mult);
        coarse_cap();
        get_swr();
        if (SWR == 0)
            return;
        if (SWR < min_swr)
        {
            min_swr = SWR + SWR / 20;
#ifdef DEBUG
    Serial.print("coarse_tune[L]:min_swr: ");
    Serial.println(min_swr);
#endif
            ind = count * L_mult;
            mem_cap = cap;
            step_ind = step;
            mem_step_cap = step_cap;
            if (SWR < 120)
                break;
            count += step;
            if ((L_linear == 0) && (count == 9))
                count = 8;
            else if ((L_linear == 0) && (count == 17))
            {
                count = 16;
                step = 4;
            }
        }
        else
            break;
    }
    cap = mem_cap;
#ifdef DEBUG
    	Serial.print("coarse_tune[L]:set_ind:");
#endif
    set_ind(ind);
#ifdef DEBUG
    	Serial.print("coarse_tune[L]:set_cap:");
#endif
    set_cap(cap);
    step_cap = mem_step_cap;
    delay(10);
    return;
}
void sharp_ind()
{
    byte range, count, max_range, min_range;
    int min_SWR;
    range = step_ind * L_mult;
    //
    max_range = ind + range;
    if (max_range > 32 * L_mult - 1)
        max_range = 32 * L_mult - 1;
    if (ind > range)
        min_range = ind - range;
    else
        min_range = 0;
    ind = min_range;
    set_ind(ind);
    get_swr();
    if (SWR == 0)
        return;
    min_SWR = SWR;
#ifdef DEBUG
    Serial.print("sharp_ind:min_swr: ");
    Serial.println(min_SWR);
#endif
    for (count = min_range + L_mult; count <= max_range; count += L_mult)
    {
#ifdef DEBUG
    Serial.print("sharp_ind:count: ");
    Serial.println(count, HEX);
#endif
        set_ind(count);
        get_swr();
        if (SWR == 0)
            return;
        if (SWR >= min_SWR)
        {
            delay(10);
            get_swr();
        }
        if (SWR >= min_SWR)
        {
            delay(10);
            get_swr();
        }
        if (SWR < min_SWR)
        {
            min_SWR = SWR;
#ifdef DEBUG
    Serial.print("sharp_ind:min_swr: ");
    Serial.println(min_SWR);
#endif
            ind = count;
            if (SWR < 120)
                break;
        } else
            break;
    }
    set_ind(ind);
    return;
}
void sharp_cap()
{
    byte range, count, max_range, min_range;
    int min_SWR;
    range = step_cap * C_mult;
    //
    max_range = cap + range;
    if (max_range > 32 * C_mult - 1)
        max_range = 32 * C_mult - 1;
    if (cap > range)
        min_range = cap - range;
    else
        min_range = 0;
    cap = min_range;
    set_cap(cap);
    get_swr();
    if (SWR == 0)
        return;
    min_SWR = SWR;
#ifdef DEBUG
    Serial.print("sharp_cap:min_swr: ");
    Serial.println(min_SWR);
#endif
    for (count = min_range + C_mult; count <= max_range; count += C_mult) {
#ifdef DEBUG
    Serial.print("sharp_cap:count: ");
    Serial.println(count, HEX);
#endif
        set_cap(count);
        get_swr();
        if (SWR == 0)
            return;
        if (SWR >= min_SWR) {
            delay(10);
            get_swr();
        }
        if (SWR >= min_SWR) {
            delay(10);
            get_swr();
        }
        if (SWR < min_SWR) {
#ifdef DEBUG
    Serial.print("sharp_cap:min_swr: ");
    Serial.println(min_SWR);
#endif
            min_SWR = SWR;
            cap = count;
            if (SWR < 120)
                break;
        } else
            break;
    }
    set_cap(cap);
    return;
}
void set_sw(byte SW)
{ // 0 - IN,  1 - OUT
#ifdef DEBUG
	Serial.print("SW: ");
	Serial.println(SW, HEX);
#endif
    digitalWrite(SW_PIN, SW);
    delay(Rel_Del);
}
void coarse_cap()
{
    byte step = 3;
    byte count;
    int min_swr;
    int coarse_swr_min;		// witek
    byte coarse_cap = 0;	// witek

    cap = 0;
    set_cap(cap);
    step_cap = step;
    get_swr();
    if (SWR == 0)
        return;
    coarse_swr_min = SWR;
    min_swr = SWR + SWR / 20;
#ifdef DEBUG
    	Serial.print("coarse_cap:min_swr: ");
    	Serial.println(min_swr);
#endif
    for (count = step; count <= 31;)
    {
#ifdef DEBUG
    	Serial.print("coarse_cap:count: ");
    	Serial.println(count);
#endif
        set_cap(count * C_mult);
        get_swr();
        if (SWR == 0)
            return;
        if (SWR < coarse_swr_min)
        {
        	coarse_swr_min = SWR;
        	coarse_cap = count * C_mult;
        }
        if (SWR < min_swr)
        {
            min_swr = SWR + SWR / 20;
#ifdef DEBUG
    	Serial.print("coarse_cap:min_swr: ");
    	Serial.println(min_swr);
#endif
            cap = count * C_mult;
            step_cap = step;
            if (SWR < 120)
                break;
            count += step;
            if ((C_linear == 0) && (count == 9))
                count = 8;
            else if ((C_linear == 0) && (count == 17))
            {
                count = 16;
                step = 4;
            }
        }
        else
            break;
    }
#ifdef DEBUG
    	Serial.print("coarse_cap:set_cap:");
#endif
    //set_cap(cap);
    	set_cap(coarse_cap);
    return;
}
void lcd_prep()
{
	if (lcd_prep_short == 0)
	{
		led_wr_str(0, 4, "ATU-100", 7);
		led_wr_str(1, 3, "EXT board", 9);
		delay(700);
		delay(500);
		led_wr_str(0, 4, "by N7DDC", 8);
		led_wr_str(1, 3, "FW ver 3.1a", 11);
		delay(600);
		delay(500);
		led_wr_str(0, 4, "        ", 8);
		led_wr_str(1, 3, "           ", 11);
	}
	delay(150);
	if (P_High == 1)
		led_wr_str(0, 0, "PWR=  0W", 8);
	else
		led_wr_str(0, 0, "PWR=0.0W", 8);
	led_wr_str(1, 0, "SWR=0.00", 8);
	if (Auto)
		led_wr_str(0, 8, ".", 1);
	lcd_ind();
	return;
}
void led_wr_str(uint8_t row, uint8_t col, char *lan, uint8_t len)
{
	lcd.setCursor(col, row);
	lcd.print(lan);
}
unsigned int get_indu_nH(byte indu)
{
	unsigned int indukcyjnosc;
	swaper indu_bit;
	indu_bit.bajt = indu;
	indukcyjnosc =
			indu_bit.bit.b7*Ind8 +
			indu_bit.bit.b6*Ind7 +
			indu_bit.bit.b5*Ind6 +
			indu_bit.bit.b4*Ind5 +
			indu_bit.bit.b3*Ind4 +
			indu_bit.bit.b2*Ind3 +
			indu_bit.bit.b1*Ind2 +
			indu_bit.bit.b0*Ind1;
	return indukcyjnosc;
}
unsigned int get_capa_pF(byte capa)
{
	unsigned int pojemnosc;
	swaper capa_bit;
	capa_bit.bajt = capa;
	pojemnosc =
			capa_bit.bit.b7*Cap8 +
			capa_bit.bit.b6*Cap7 +
			capa_bit.bit.b5*Cap6 +
			capa_bit.bit.b4*Cap5 +
			capa_bit.bit.b3*Cap4 +
			capa_bit.bit.b2*Cap3 +
			capa_bit.bit.b1*Cap2 +
			capa_bit.bit.b0*Cap1;
	return pojemnosc;
}
void lcd_pwr()
{
	int p = 0;
	byte peak_cnt;
	int delta;
	byte cnt;
	int SWR_fixed = 1;
	delta = Auto_delta - 100;
	PWR = 0;
	// peak detector
	cnt = 120;
	for (peak_cnt = 0; peak_cnt < cnt; peak_cnt++)
	{
		if (button_pressed())
				return;		// Fast return if button pressed
		get_pwr();
		if (PWR > p)
		{
			p = PWR;
			SWR_fixed = SWR;
		}
		delay(3);
	}
	Power = p;
	lcd_swr(SWR_fixed);
	if (SWR_fixed >= 100)
	{
		dysp_on(); // dysplay ON
		dysp_cnt = Dysp_delay * dysp_cnt_mult;
	}
	// ToDo sprawdzić logikę ;-)
	if (Auto && SWR_fixed >= Auto_delta
			&& ((SWR_fixed > SWR_fixed_old && (SWR_fixed - SWR_fixed_old) > delta)
					|| (SWR_fixed < SWR_fixed_old
							&& (SWR_fixed_old - SWR_fixed) > delta)
					|| SWR_fixed_old == 999))
		Soft_tune = 1;
	//
	if (button_pressed())
			return;		// Fast return if button pressed
	show_pwr(Power, SWR_fixed);
	//
	if (button_pressed())
			return;		// Fast return if button pressed
	if (Overload == 1)
	{
		if (type == 4 || type == 5)
		{ // 128*64 OLED
			led_wr_str(2, 16, "        ", 8);
			delay(100);
			led_wr_str(2, 16, "OVERLOAD", 8);
			delay(500);
			led_wr_str(2, 16, "        ", 8);
			delay(300);
			led_wr_str(2, 16, "OVERLOAD", 8);
			delay(500);
			led_wr_str(2, 16, "        ", 8);
			delay(300);
			led_wr_str(2, 16, "OVERLOAD", 8);
			delay(500);
			led_wr_str(2, 16, "        ", 8);
			delay(100);
			led_wr_str(2, 16, "SWR=    ", 8);
		}
		else if (type != 0)
		{ // 1602  & 128*32// 1602
			led_wr_str(1, 0, "        ", 8);
			delay(100);
			led_wr_str(1, 0, "OVERLOAD", 8);
			delay(500);
			led_wr_str(1, 0, "        ", 8);
			delay(300);
			led_wr_str(1, 0, "OVERLOAD", 8);
			delay(500);
			led_wr_str(1, 0, "        ", 8);
			delay(300);
			led_wr_str(1, 0, "OVERLOAD", 8);
			delay(500);
			led_wr_str(1, 0, "        ", 8);
			delay(100);
			led_wr_str(1, 0, "SWR=    ", 8);
		}
		SWR_old = 10000;
		lcd_swr(SWR_fixed);
	}
	return;
}
void lcd_swr(int swr)
{
    if (swr != SWR_old)
    {
        SWR_old = swr;
        if (swr == 1)
        { // Low power
            if ((type == 4) || (type == 5))
                led_wr_str(2, 16 + 4 * 12, "0.00", 4); // 128*64 OLED
            else if (type != 0)
                led_wr_str(1, 4, "0.00", 4); // 1602  & 128*32 OLED
             // real-time 2-colors led work
            	digitalWrite(GREEN_LED_PIN, HIGH);
            	digitalWrite(RED_LED_PIN, HIGH);
            SWR_old = 0;
        }
        else
        {
            SWR_old = swr;
    		itoa(swr, work_str, 10);
#ifdef DEBUG
    		//if (swr > 100)
    		if (true)
    		{
    			Serial.print("swr: ");
    			Serial.println(swr);
    			Serial.print("swr_str: _");
    			Serial.print(work_str);
    			Serial.println('_');
    		}
#endif
            work_str_2[0] = work_str[0];
            work_str_2[1] = '.';
            work_str_2[2] = work_str[1];
            work_str_2[3] = work_str[2];
			if ((type == 4) || (type == 5))
				led_wr_str(2, 16 + 4 * 12, work_str_2, 4); // 128*64 OLED
			else if (type != 0)
				led_wr_str(1, 4, work_str_2, 4); // 1602  & 128*32

			// real-time 2-colors led work
			if (swr <= 150)
			{
				digitalWrite(GREEN_LED_PIN, LOW);
				digitalWrite(RED_LED_PIN, HIGH);
			} // Green
			else if (swr <= 250)
			{
				digitalWrite(GREEN_LED_PIN, LOW);
				digitalWrite(RED_LED_PIN, LOW);
			} // Orange
			else
			{
				digitalWrite(GREEN_LED_PIN, HIGH);
				digitalWrite(RED_LED_PIN, LOW);
			} // Red
		}
    }
    return;
}
void dysp_on()
{
	lcd.backlight();
}
void dysp_off()
{
	lcd.noBacklight();
}
bool button_pressed()
{
	bool zwrot = false;
	if (digitalRead(TUNE_BUTTON_PIN) == LOW || digitalRead(AUTO_BUTTON_PIN) == LOW || digitalRead(BYPASS_BUTTON_PIN) == LOW)
	{
		tune_button.update();
		auto_button.update();
		bypass_button.update();
		if (tune_button.isPressed() || auto_button.isPressed() || bypass_button.isPressed())
			zwrot = true;
		else
			zwrot = false;
	}
	return zwrot;
}
void led_init()
{
	lcd.init();
	lcd.noCursor();
	lcd.backlight();
}
void read_i2c_inputs()
{
    swaper band;
    band.bit.b0 = digitalRead(BAND0_PIN);
    band.bit.b1 = digitalRead(BAND1_PIN);
    band.bit.b2 = digitalRead(BAND2_PIN);
    band.bit.b3 = digitalRead(BAND3_PIN);
    mem_offset = (~band.bajt) & 0x0F;
}
void load_settings()
{
    cap = EEPROM.read(255 - mem_offset * 5);
    ind = EEPROM.read(254 - mem_offset * 5);
    SW = EEPROM.read(253 - mem_offset * 5);
    if (SW > 1)
    	SW = 1;
    swr_a = EEPROM.read(252 - mem_offset * 5) * 256;
    swr_a += EEPROM.read(251 - mem_offset * 5);
    set_ind(ind);
    set_cap(cap);
    set_sw(SW);
}
void Test_init(void)
{ // Test mode
    if ((type == 4) || (type == 5)) // 128*64 OLED
        led_wr_str(0, 10, "TEST MODE", 9);
    else if (type != 0) // 1602 LCD  or 128*32 OLED
        led_wr_str(0, 3, "TEST MODE", 9);
    delay(2000);
    if ((type == 4) || (type == 5)) // 128*64 OLED
        led_wr_str(0, 10, "         ", 9);
    else if (type != 0) // 1602 LCD  or 128*32 OLED
        led_wr_str(0, 3, "         ", 9);
    atu_reset();
    SW = 1; // C to OUT
    set_sw(SW);
#ifdef DEBUG
    Serial.println("Test init: zapis do EEPROM");
#endif
    EEPROM.write(255 - mem_offset * 5, cap);
    EEPROM.write(254 - mem_offset * 5, ind);
    EEPROM.write(253 - mem_offset * 5, SW);
    if ((type == 4) || (type == 5)) // 128*64 OLED
        led_wr_str(0, 16 + 12 * 8, "l", 1);
    else if (type != 0) // 1602 LCD or 128*32 OLED
        led_wr_str(0, 8, "l", 1);
    //
    lcd_prep_short = 1;
    lcd_prep();
    return;
}
void button_proc(void)
{
	tune_button.update();
#ifdef DEBUG
	if (tune_button.isPressed())
	{
		Serial.print("tune.pressed: ");
		Serial.println(tune_button.isPressed());
	}
	if (tune_button.isPressed())
	{
		Serial.print("tune.isPressed: ");
		Serial.println(tune_button.isPressed());
	}
#endif
    if (tune_button.isPressed() || Soft_tune)
    {
        dysp_on();
        dysp_cnt = Dysp_delay * dysp_cnt_mult;
        delay(250);
        if (Soft_tune == 0 && digitalRead(TUNE_BUTTON_PIN) == 1)
        { // short press TUNE button
            show_reset();
            bypas = 0;
        }
        else
        { // long press TUNE button
            digitalWrite(TX_REQUEST_PIN1, HIGH);
            delay(250); //
#ifdef SP2HYO
            delay(1000);	// dodatkowe opóźnienie dla TX_REQUEST_PIN2
#endif
            digitalWrite(TX_REQUEST_PIN2, HIGH);

            btn_push();		// tutaj rozpoczęcie procedury strojenia

            bypas = 0;
            tune_button.update();
            while (tune_button.isPressed())
            {
                lcd_pwr();
                tune_button.update();
            }
            Soft_tune = 0;
        }
    }
    //
    bypass_button.update();
    if (bypass_button.isPressed())
    { // BYPASS button
        dysp_on();
        dysp_cnt = Dysp_delay * dysp_cnt_mult;
        if (bypas == 0) {
            bypas = 1;
            cap_mem = cap;
            ind_mem = ind;
            SW_mem = SW;
            cap = 0;
            ind = 0;
            SW = 1;
            set_ind(ind);
            set_cap(cap);
            set_sw(SW);
            if (Loss_mode == 0)
                lcd_ind();
            Auto_mem = Auto;
            Auto = 0;
        }
        else
        {
            bypas = 0;
            cap = cap_mem;
            ind = ind_mem;
            SW = SW_mem;
            set_cap(cap);
            set_ind(ind);
            set_sw(SW);
            if (Loss_mode == 0)
                lcd_ind();
            Auto = Auto_mem;
        }
        if (type == 4 || type == 5)
        { // 128*64 OLED
            if (Auto && !bypas)
                led_wr_str(0, 16 + 8 * 12, ".", 1);
            else if (!Auto && bypas)
                led_wr_str(0, 16 + 8 * 12, "_", 1);
            else
                led_wr_str(0, 16 + 8 * 12, " ", 1);
        }
        else if (type != 0)
        { //  1602 LCD  or 128*32 OLED
            if (Auto && !bypas)
                led_wr_str(0, 8, ".", 1);
            else if (!Auto && bypas)
                led_wr_str(0, 8, "_", 1);
            else
                led_wr_str(0, 8, " ", 1);
        }
        bypass_button.update();
        while (bypass_button.isPressed())
        {
            lcd_pwr();
            bypass_button.update();
        }
    }
    //
    auto_button.update();
    if (auto_button.isPressed() & bypas == 0)
    { // Auto button
        dysp_on();
        dysp_cnt = Dysp_delay * dysp_cnt_mult;
        if (Auto == 0)
            Auto = 1;
        else
            Auto = 0;
#ifdef DEBUG
    Serial.println("Auto_button:: zapis do EEPROM");
#endif
        EEPROM.write(2, Auto);
        if (type == 4 || type == 5)
        { // 128*64 OLED
            if (Auto && !bypas)
                led_wr_str(0, 16 + 8 * 12, ".", 1);
            else if (!Auto && bypas)
                led_wr_str(0, 16 + 8 * 12, "_", 1);
            else
                led_wr_str(0, 16 + 8 * 12, " ", 1);
        }
        else if (type != 0)
        { //  1602 LCD  or 128*32 OLED
            if (Auto && !bypas)
                led_wr_str(0, 8, ".", 1);
            else if (!Auto && bypas)
                led_wr_str(0, 8, "_", 1);
            else
                led_wr_str(0, 8, " ", 1);
        }
        auto_button.update();
        while (auto_button.isPressed())
        {
            lcd_pwr();
            auto_button.update();
        }
    }
    return;
}
void show_reset()
{
    atu_reset();
    SW = 1;
    set_sw(SW);
#ifdef DEBUG
    Serial.println("show_reset: zapis do EEPROM");
#endif
    EEPROM.write(255 - mem_offset * 5, 0);
    EEPROM.write(254 - mem_offset * 5, 0);
    EEPROM.write(253 - mem_offset * 5, 1);
    EEPROM.write(252 - mem_offset * 5, 0);
    EEPROM.write(251 - mem_offset * 5, 0);
    lcd_ind();
    Loss_mode = 0;
    digitalWrite(TX_REQUEST_PIN1, LOW);
    digitalWrite(TX_REQUEST_PIN2, LOW);
    SWR = 0;
    PWR = 0;
    SWR_fixed_old = 0;
    if ((type == 4) || (type == 5))
    { // 128*64 OLED
        led_wr_str(2, 16, "RESET   ", 8);
        delay(600);
        led_wr_str(2, 16, "SWR=0.00", 8);
    } else if (type != 0)
    { // 1602 LCD & 128*32 OLED
        led_wr_str(1, 0, "RESET   ", 8);
        delay(600);
        led_wr_str(1, 0, "SWR=0.00", 8);
    }
    else
    {
    	digitalWrite(GREEN_LED_PIN, HIGH);
    	digitalWrite(RED_LED_PIN, HIGH);
    }
    SWR_old = 10000;
    Power_old = 10000;
    lcd_pwr();
    return;
}
/*
 * wywołanie strojenia
 * i zapisanie wyniku w EEPROM
 */
void btn_push()
{
    if ((type == 4) || (type == 5))
    { // 128*64 OLED
        led_wr_str(2, 16 + 12 * 4, "TUNE", 4);
    }
    else if (type != 0)
    { // 1602 LCD & 128*32 OLED
        led_wr_str(1, 4, "TUNE", 4);
    }
	digitalWrite(GREEN_LED_PIN, HIGH);
	digitalWrite(RED_LED_PIN, HIGH);

    tune();		// strojenie

	// real-time 2-colors led work
	if (SWR <= 150)
	{
		digitalWrite(GREEN_LED_PIN, LOW);
		digitalWrite(RED_LED_PIN, HIGH);
	} // Green
	else if (SWR <= 250)
	{
		digitalWrite(GREEN_LED_PIN, LOW);
		digitalWrite(RED_LED_PIN, LOW);
	} // Orange
	else
	{
		digitalWrite(GREEN_LED_PIN, HIGH);
		digitalWrite(RED_LED_PIN, LOW);
	} // Red
    if ((Loss_mode == 0) || (Loss_ind == 0))
        lcd_ind();
    tune_zapis();
    SWR_old = 10000;
    Power_old = 10000;
    lcd_pwr();
    SWR_fixed_old = SWR;
    digitalWrite(TX_REQUEST_PIN1, LOW);
    digitalWrite(TX_REQUEST_PIN2, LOW);
    return;
}
void button_proc_test(void)
{
	tune_button.update();
    if (tune_button.isPressed())
    { // Tune btn
        delay(250);
        if (digitalRead(TUNE_BUTTON_PIN) == 1)
        { // short press button
            if (SW == 0)
                SW = 1;
            else
                SW = 0;
            set_sw(SW);
            lcd_ind();
        }
        else
        { // long press button
            if (L == 1)
                L = 0;
            else
                L = 1;
            if (L == 1)
            {
                if (type == 4 || type == 5) // 128*64 OLED
                    led_wr_str(0, 16 + 12 * 8, "l", 1);
                else if (type != 0) // 1602 LCD & 128*32 OLED
                    led_wr_str(0, 8, "l", 1);
            }
            else
            {
                if (type == 4 || type == 5) // 128*64 OLED
                    led_wr_str(0, 16 + 12 * 8, "c", 1);
                else if (type != 0) // 1602 LCD & 128*32 OLED
                    led_wr_str(0, 8, "c", 1);
            }
        }
        tune_button.update();
        while (tune_button.isPressed())
        {
            lcd_pwr();
            tune_button.update();
        }
    } // END Tune btn
    //
    bypass_button.update();
    if (bypass_button.isPressed())
    { // BYP button
        while (digitalRead(BYPASS_BUTTON_PIN) == 0)
        {
            if (L && ind < (32 * L_mult - 1))
            {
                ind++;
                set_ind(ind);
            }
            else if (!L && cap < (32 * C_mult - 1))	// było L_mult
            {
                cap++;
                set_cap(cap);
            }
            lcd_ind();
            lcd_pwr();
            delay(30);
        }
    } // end of BYP button
    //
    auto_button.update();
    if (auto_button.isPressed() & (bypas == 0))
    { // Auto button
        while (digitalRead(AUTO_BUTTON_PIN) == 0)
        {
            if (L && (ind > 0))
            {
                ind--;
                set_ind(ind);
            }
            else if (!L && (cap > 0))
            {
                cap--;
                set_cap(cap);
            }
            lcd_ind();
            lcd_pwr();
            delay(30);
        }
    }
    return;
}
void cells_init(void)
{
	// ToDo domyślny zapis w EEPROM
	// ToDo odczyt ustawień z EEPROM
    // Cells init
    type = EEPROM.read(1); // type of display
    type = 1;	// 1602
    Auto = 0;
    if (EEPROM.read(2) == 1)
        Auto = 1;
    Rel_Del = Bcd2Dec(EEPROM.read(3)); // Relay's Delay
    Rel_Del = 30;
    Auto_delta = Bcd2Dec(EEPROM.read(4)) * 10; // Auto_delta
    Auto_delta = 130;
    min_for_start = Bcd2Dec(EEPROM.read(5)) * 10; // P_min_for_start
#ifdef SP2HYO
    min_for_start = 10;
#else
    min_for_start = 30;
#endif
    max_for_start = Bcd2Dec(EEPROM.read(6)) * 10; // P_max_for_start
    max_for_start = 0;
    // 7  - shift down
    // 8 - shift left
    max_swr = Bcd2Dec(EEPROM.read(9)) * 10; // Max SWR
    max_swr = 0;
    L_q = EEPROM.read(10);
    L_q = 7;
    L_linear = EEPROM.read(11);
    L_linear = 0;
    C_q = EEPROM.read(12);
    C_q = 8;
    C_linear = EEPROM.read(13);
    C_linear = 0;
    D_correction = EEPROM.read(14);
    D_correction = 1;
    L_invert = EEPROM.read(15);
    L_invert = 0;
    //
    Ind1 = Bcd2Dec(EEPROM.read(16)) * 100 + Bcd2Dec(EEPROM.read(17)); // Ind1
    Ind2 = Bcd2Dec(EEPROM.read(18)) * 100 + Bcd2Dec(EEPROM.read(19)); // Ind2
    Ind3 = Bcd2Dec(EEPROM.read(20)) * 100 + Bcd2Dec(EEPROM.read(21)); // Ind3
    Ind4 = Bcd2Dec(EEPROM.read(22)) * 100 + Bcd2Dec(EEPROM.read(23)); // Ind4
    Ind5 = Bcd2Dec(EEPROM.read(24)) * 100 + Bcd2Dec(EEPROM.read(25)); // Ind5
    Ind6 = Bcd2Dec(EEPROM.read(26)) * 100 + Bcd2Dec(EEPROM.read(27)); // Ind6
    Ind7 = Bcd2Dec(EEPROM.read(28)) * 100 + Bcd2Dec(EEPROM.read(29)); // Ind7
    //
    Ind1 = 50, Ind2 = 100, Ind3 = 220, Ind4 = 450, Ind5 = 1000, Ind6 = 2200, Ind7 = 4400;

    Cap1 = Bcd2Dec(EEPROM.read(32)) * 100 + Bcd2Dec(EEPROM.read(33)); // Cap1
    Cap2 = Bcd2Dec(EEPROM.read(34)) * 100 + Bcd2Dec(EEPROM.read(35)); // Cap2
    Cap3 = Bcd2Dec(EEPROM.read(36)) * 100 + Bcd2Dec(EEPROM.read(37)); // Cap3
    Cap4 = Bcd2Dec(EEPROM.read(38)) * 100 + Bcd2Dec(EEPROM.read(39)); // Cap4
    Cap5 = Bcd2Dec(EEPROM.read(40)) * 100 + Bcd2Dec(EEPROM.read(41)); // Cap5
    Cap6 = Bcd2Dec(EEPROM.read(42)) * 100 + Bcd2Dec(EEPROM.read(43)); // Cap6
    Cap7 = Bcd2Dec(EEPROM.read(44)) * 100 + Bcd2Dec(EEPROM.read(45)); // Cap7
    Cap8 = Bcd2Dec(EEPROM.read(46)) * 100 + Bcd2Dec(EEPROM.read(47)); // Cap8

    Cap1 = 10, Cap2 = 22, Cap3 = 47, Cap4 = 100, Cap5 = 220, Cap6 = 470, Cap7 = 1000, Cap8 = 1820;
    //
    P_High = EEPROM.read(0x30); // High power
    P_High = 1;
    K_Mult = Bcd2Dec(EEPROM.read(0x31)); // Tandem Natch rate

#ifdef SP2HYO
    K_Mult = 33;
#else
    K_Mult = 24;
#endif
    Dysp_delay = Bcd2Dec(EEPROM.read(0x32)); // Dysplay ON delay
    Dysp_delay = 20;
    Loss_ind = EEPROM.read(0x33);
    Loss_ind = 0;
    Fid_loss = Bcd2Dec(EEPROM.read(0x34));
    Fid_loss = 0;
    return;
}
uint8_t Bcd2Dec(uint8_t n)
{
  return ((n / 16 * 10) + (n % 16));
}
void tune_zapis()
{
#ifdef DEBUG
    Serial.println("btn_push: zapis do EEPROM");
#endif
    EEPROM.write(255 - mem_offset * 5, cap);
    EEPROM.write(254 - mem_offset * 5, ind);
    EEPROM.write(253 - mem_offset * 5, SW);
    EEPROM.write(252 - mem_offset * 5, swr_a / 256);
    EEPROM.write(251 - mem_offset * 5, swr_a % 256);
}
void set_multis()
{
    if (L_q == 5)
        L_mult = 1;
    else if (L_q == 6)
        L_mult = 2;
    else if (L_q == 7)
        L_mult = 4;
    if (C_q == 5)
        C_mult = 1;
    else if (C_q == 6)
        C_mult = 2;
    else if (C_q == 7)
        C_mult = 4;
    else if (C_q == 8)		// 8 kondensatorów
    	C_mult = 8;
}
