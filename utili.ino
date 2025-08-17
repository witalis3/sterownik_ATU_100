/**
 * Utili software parts
 */

#include "defines.h"
extern Adafruit_MCP23008 mcp_c;	// expander dla kondensatorów; sub adres 0x0
extern Adafruit_MCP23008 mcp_l;	// expander dla cewek; sub adres 0x1

void UTILI_SetRelays(void)
{
	mcp_l.writeGPIO(global.ind_relays);
	mcp_c.writeGPIO(global.cap_relays);	// ToDo sprawdzić poprawność wysyłania całego bajtu
    digitalWrite(SW_PIN, global.cap_sw);

	//UTILI_Delay_ms(RELAY_DELAY_MS);
}

void UTILI_Set_LC_Relays(uint16_t relays)
{
	if (relays & 0x8000)
	{
		global.cap_sw = 1;
	}
	else
	{
		global.cap_sw = 0;
	}

	global.cap_relays = (uint8_t) (relays >> 8) & 0x7f;
	global.ind_relays = (uint8_t) (relays & 0x7f);
	UTILI_SetRelays();
}

uint16_t UTILI_Get_LC_Relays(void)
{
	uint16_t relays = (((uint16_t) global.cap_relays) << 8) + global.ind_relays;

	if (global.cap_sw)
	{
		relays |= 0x8000;
	}
	return (relays);
}

//uint8_t UTILI_Bcd2Dec(uint8_t bcdnum)
//{
//  return ((bcdnum / 16 * 10) + (bcdnum % 16));
//};
//
//uint8_t UTILI_Dec2Bcd(uint8_t decnum)
//{
//  return ((decnum / 10 * 16) + (decnum % 10));
//};

void UTILI_Int2Str(int16_t number, char *output, uint8_t size)
{

	char *p;
	uint8_t minus = 0;

	if (size <= 1)
		return;

	uint8_t pos = size - 1;

	p = output + pos;

	*p = 0; //string terminator

	if (number < 0)
	{
		minus = 1;
		number = number * -1;
	}

	do
	{
		pos--;
		p--;
		if (number > 0)
		{
			*p = '0' + (number % 10);
			number /= 10;
		}
		else
		{
			if (pos == (size - 2))
				*p = '0'; //if zero -> '0'
			else
			{
				if (minus)
				{
					*p = '-';   //if minus -> '-'
					minus = 0;
				}
				else
				{
					*p = ' ';  //fill with space
				}
			}
		}

	} while (pos > 0);

}
;

/**
 * Calculate the voltage on a 50ohm resistor
 * U = sqrt( P x  50Ohm) ->  centiVolt = sqrt( P x 50Ohm) x 100 
 * centiVolt = sqrt( (deziWatt/10) x 50Ohm) x 100 
 * centiVolt = sqrt(deziWatt x 50000)
 * @param deziWatt
 * @return deziVolt
 */
int16_t UTILI_deciWatt_to_centiVolt(int16_t deciWatt)
{
	int32_t number;
	int32_t sqrt;
	int32_t temp;

	//sqrt function without math.h
	number = deciWatt * (int32_t) 50000;
	sqrt = number / 2;
	temp = 0;

	while (sqrt != temp)
	{
		temp = sqrt;
		sqrt = ((number / temp) + temp) / 2;
	}

	return ((int16_t) sqrt);
}

/**
 * Calculate the power fron the voltage on 50ohms
 * P = U� / 50 ohm
 * deziWatt = ((centiVolt/100) x (centiVolt/100) / 50ohm) * 10
 * deziWatt = ((centiVolt x centiVolt) / (10000 x 50)) * 10
 * deziWatt = (centiVolt x centiVolt) / 50000
 * Round +25000/50000 = +0,5
 * @param deciVolt
 * @return 
 */
int16_t UTILI_centiVolt_to_deciWatt(int16_t centiVolt)
{
	int16_t deciWatt = (int16_t) ((((int32_t) centiVolt * centiVolt) + 25000)
			/ 50000);
	return (deciWatt);
}

void UTILI_CalPWR(void)
{

	int16_t centiVolt = (int16_t) (((int32_t) global.cal_gain * global.adc_f_mV)
			/ CAL_GAIN_MULTIPLIER) + global.cal_offset;
	int16_t deciWatt = UTILI_centiVolt_to_deciWatt(centiVolt);

	//Cheat: Set 0.1W to 0.0W
	if (deciWatt <= 1)
	{
		deciWatt = 0;
	}

	global.PWR = deciWatt;
}

void UTILI_CalSWR(void)
{
	int16_t swr = 0;

	//global.Reflection = (uint8_t) (((int32_t)global.adc_r_mV * 100) / global.adc_f_mV);

	if (global.adc_f_mV >= SWR_FORWORD_THRESHOLD)
	{

		if (global.adc_f_mV > global.adc_r_mV)
		{
			swr = (int16_t) (((int32_t) (global.adc_f_mV + global.adc_r_mV) * 100) / (global.adc_f_mV - global.adc_r_mV));
			if (swr > MAX_SWR)
			{
				swr = MAX_SWR;
			}
		}
		else
		{
			swr = MAX_SWR; //Max SWR 9.99
		}
	}
	global.SWR = swr;
}

int16_t UTILI_GetCapValue(void)
{
	int16_t value = 0;
	for (uint8_t i = 0; i < 7; i++)
	{
		if (global.cap_relays & (1 << i))
		{
			//value = value + global.cap_values[i];
			value = value + CAP_VALUES[i];
		}
	}
	return (value);
}

int16_t UTILI_GetIndValue(void)
{
	int16_t value = 0;
	for (uint8_t i = 0; i < 7; i++)
	{
		if (global.ind_relays & (1 << i))
		{
			//value = value + global.ind_values[i];
			value = value + IND_VALUES[i];
		}
	}
	return (value);
}

void UTILI_GetCapValueStr(char *str, uint8_t str_size)
{
	if (str_size < 7)
		return;
	int16_t value = UTILI_GetCapValue();
	UTILI_Int2Str(value, str, 5); //value 4 char
	str[4] = 'p';
	str[5] = 'F';
	str[6] = 0; // termination
}

void UTILI_GetIndValueStr(char *str, uint8_t str_size)
{
	if (str_size < 7)
		return;
	int16_t value = UTILI_GetIndValue();
	UTILI_Int2Str(value, str, 5); //value 4 char
	str[4] = 'n';
	str[5] = 'H';
	str[6] = 0; // termination
}
