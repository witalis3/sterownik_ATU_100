#include "Arduino.h"
#include "sterownik_ATU_100.h"

#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27);  // Set the LCD I2C address

#include <Adafruit_MCP23008.h>
Adafruit_MCP23008 mcp_c;	// expander dla kondensatorów; sub adres 0x0
Adafruit_MCP23008 mcp_l;	// expander dla cewek; sub adres 0x1

#include "Bounce2.h"
Bounce tune_b = Bounce();

char rready = 0, p_cnt = 0;
int P_max, SWR, PWR, swr_a;
static char ind = 0, cap = 0, SW = 0, step_cap = 0, step_ind = 0, L_linear = 0, C_linear = 0, L_q = 7, C_q = 7, Overload = 0,
		D_correction = 1, K_Mult = 24, P_High = 1, L_invert = 0, L_mult = 1, C_mult = 1, Loss_ind = 0;

static int Rel_Del = 15, min_for_start, max_for_start, max_swr = 0;


void setup()
{
	pinMode(SW_PIN, OUTPUT);
	digitalWrite(SW_PIN, SW);
	lcd.begin(16,2);               // initialize the lcd
	lcd.print("ATU-100 start");
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("setup poczatek");
#endif
	mcp_l.begin(1);
	mcp_l.writeGPIO(0x0);		// wszystkie przekaźniki wyłączone
	mcp_l.write8(MCP23008_IODIR, 0);	// wszystkie piny jako wyjścia
	tune_b.attach(TUNE_BUTTON_PIN, INPUT_PULLUP);
	delay(500);
	lcd.noBacklight();
	delay(200);
	lcd.backlight();
}

void loop()
{
	tune_b.update();
	if (tune_b.read() == LOW)
	{
		tune();
	}
}

void tune()
{
    p_cnt = 0;
    P_max = 0;
    //
    rready = 0;
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
    if ((max_swr > 110) & (SWR > max_swr))
    	// max_swr - zawartość komórki 9 (domyślnie 0)
        return;
    //

    sub_tune();

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
    while ((PWR < min_for_start) | ((PWR > max_for_start) & (max_for_start > 0)))
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
        //
        /*	ToDo obsługa klawisza Tune
        if (Button( & PORTB, 0, 5, 1))
            rready = 1;
        if (rready == 1 & Button( & PORTB, 0, 5, 0))
        { //  press button  Tune
            show_reset();
            SWR = 0;	// wskaźnik przerwania oczekiwania na właściwą moc - reset
            return;
        }
        */
    } //  good power
    return;
}
void atu_reset()
{
    ind = 0;
    cap = 0;
    set_ind(ind);
    set_cap(cap);
    delay(Rel_Del);
}
void get_pwr()
{
    long Forward, Reverse;
    float p;
    //
    Forward = get_forward();
    Reverse = get_reverse();
    if (D_correction == 1)
        p = correction(Forward * 3);
    else
        p = Forward * 3;
    //
    if (Reverse >= Forward)
        Forward = 999;
    else {
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
    if (PWR < 10)
        SWR = 1;
    else if (Forward < 100)
        SWR = 999;
    else
        SWR = Forward;
    return;
}
int get_forward()
{
    int Forward;
    Forward = analogRead(FWD_PIN);
    if (Forward > 1000)
        Overload = 1;
    else
        Overload = 0;
    return Forward * 4.883; // zwraca napięcie w mV
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
    lcd.setCursor(0, 0);
    lcd.print(Power);
    lcd.setCursor(0, 1);
    lcd.print(SWR);
    //
    /*	ToDo wyświetlanie mocy
    if (Test == 0 & Loss_ind == 1 & SWR >= 100)
    {
        if (Loss_mode == 0) { // prepare
            if (type == 4 | type == 5) { // 128*64 OLED
                if (P_High == 1)
                    led_wr_str(4, 16, "ANT=  0W", 8);
                else
                    led_wr_str(4, 16, "ANT=0.0W", 8);
                led_wr_str(6, 16, "EFF=  0%", 8);
            } else if (type == 2 | type == 3) { // 128*32 OLED
                if (P_High == 1)
                    led_wr_str(0, 9, "ANT=  0W", 8);
                else
                    led_wr_str(0, 9, "ANT=0.0W", 8);
                led_wr_str(1, 9, "EFF=  0%", 8);
            } else if (type == 1) { // 1602 LCD
                if (P_High == 1)
                    led_wr_str(0, 9, "AN=  0W", 7);
                else
                    led_wr_str(0, 9, "AN=0.0W", 7);
                led_wr_str(1, 9, "EFF= 0%", 7);
            }
        }
        Loss_mode = 1;
    } else {
        if (Loss_mode == 1)
            lcd_ind();
        Loss_mode = 0;
    }

    if (Power != Power_old) {
        Power_old = Power;
        //
        if (P_High == 0) {
            if (Power >= 100) { // > 10 W
                Power += 5; // rounding to 1 W
                IntToStr(Power, work_str);
                work_str_2[0] = work_str[2];
                work_str_2[1] = work_str[3];
                work_str_2[2] = work_str[4];
                work_str_2[3] = 'W';
            } else {
                IntToStr(Power, work_str);
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
        } else { // High Power
            if (Power < 999) { // 0 - 1500 Watts
                IntToStr(Power, work_str);
                work_str_2[0] = work_str[3];
                work_str_2[1] = work_str[4];
                work_str_2[2] = work_str[5];
                work_str_2[3] = 'W';
            } else {
                IntToStr(Power, work_str);
                work_str_2[0] = work_str[2];
                work_str_2[1] = work_str[3];
                work_str_2[2] = work_str[4];
                work_str_2[3] = work_str[5];
            }
        }
        if (type == 4 | type == 5)
            led_wr_str(0, 16 + 4 * 12, work_str_2, 4); // 128*64 OLED
        else if (type != 0)
            led_wr_str(0, 4, work_str_2, 4); // 1602  & 128*32
        //

        //  Loss indication
        if (Loss_mode == 1) {
            if (ind == 0 & cap == 0)
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
            if (P_High == 0) {
                if (p_ant >= 100) { // > 10 W
                    p_ant += 5; // rounding to 1 W
                    IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[2];
                    work_str_2[1] = work_str[3];
                    work_str_2[2] = work_str[4];
                    work_str_2[3] = 'W';
                } else {
                    IntToStr(p_ant, work_str);
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
            } else { // High Power
                if (p_ant < 999) { // 0 - 1500 Watts
                    IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[3];
                    work_str_2[1] = work_str[4];
                    work_str_2[2] = work_str[5];
                    work_str_2[3] = 'W';
                } else {
                    IntToStr(p_ant, work_str);
                    work_str_2[0] = work_str[2];
                    work_str_2[1] = work_str[3];
                    work_str_2[2] = work_str[4];
                    work_str_2[3] = work_str[5];
                }
            }
            if (type == 4 | type == 5)
                led_wr_str(4, 16 + 4 * 12, work_str_2, 4); // 128*64 OLED
            else if (type == 2 | type == 3)
                led_wr_str(0, 13, work_str_2, 4); // 128*32
            else if (type == 1)
                led_wr_str(0, 12, work_str_2, 4); // 1602
            //
            IntToStr(eff, work_str);
            work_str_2[0] = work_str[4];
            work_str_2[1] = work_str[5];
            if (type == 4 | type == 5)
                led_wr_str(6, 16 + 5 * 12, work_str_2, 2);
            else if (type == 2 | type == 3)
                led_wr_str(1, 14, work_str_2, 2);
            else if (type == 1)
                led_wr_str(1, 13, work_str_2, 2);
        }
    }

    return;
    */
}
void set_ind(char Ind)
{
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
void set_cap(char Cap)
{
	mcp_c.writeGPIO(Cap);	// ToDo sprawdzić poprawność wysyłania całego bajtu
}
/*
 * wyświetlenie wartości L i C
 */
void lcd_ind()
{

}
void sub_tune()
{
    int swr_mem, ind_mem, cap_mem;
    //
    swr_mem = SWR;
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
    if ((SWR < 200) & (SWR < swr_mem) & ((swr_mem - SWR) > 100))
        return;
    swr_mem = SWR;
    ind_mem = ind;
    cap_mem = cap;
    //
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
    char step = 3;
    char count;
    char mem_cap, mem_step_cap;
    int min_swr;

    mem_cap = 0;
    step_ind = step;
    mem_step_cap = 3;
    min_swr = SWR + SWR / 20;
    for (count = 0; count <= 31;)
    {
        set_ind(count * L_mult);
        coarse_cap();
        get_swr();
        if (SWR == 0)
            return;
        if (SWR < min_swr)
        {
            min_swr = SWR + SWR / 20;
            ind = count * L_mult;
            mem_cap = cap;
            step_ind = step;
            mem_step_cap = step_cap;
            if (SWR < 120)
                break;
            count += step;
            if ((L_linear == 0) & (count == 9))
                count = 8;
            else if ((L_linear == 0) & (count == 17))
            {
                count = 16;
                step = 4;
            }
        }
        else
            break;
    }
    cap = mem_cap;
    set_ind(ind);
    set_cap(cap);
    step_cap = mem_step_cap;
    delay(10);
    return;
}
void sharp_ind()
{
    char range, count, max_range, min_range;
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
    for (count = min_range + L_mult; count <= max_range; count += L_mult) {
        set_ind(count);
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
            min_SWR = SWR;
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
    char range, count, max_range, min_range;
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
    for (count = min_range + C_mult; count <= max_range; count += C_mult) {
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
void set_sw(char SW)
{ // 0 - IN,  1 - OUT
    digitalWrite(SW_PIN, SW);
    delay(Rel_Del);
}
void coarse_cap()
{
    char step = 3;
    char count;
    int min_swr;

    cap = 0;
    set_cap(cap);
    step_cap = step;
    get_swr();
    if (SWR == 0)
        return;
    min_swr = SWR + SWR / 20;
    for (count = step; count <= 31;) {
        set_cap(count * C_mult);
        get_swr();
        if (SWR == 0)
            return;
        if (SWR < min_swr) {
            min_swr = SWR + SWR / 20;
            cap = count * C_mult;
            step_cap = step;
            if (SWR < 120)
                break;
            count += step;
            if ((C_linear == 0) & (count == 9))
                count = 8;
            else if ((C_linear == 0) & (count == 17)) {
                count = 16;
                step = 4;
            }
        } else
            break;
    }
    set_cap(cap);
    return;
}
