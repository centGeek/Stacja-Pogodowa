/*****************************************************************************
 *   Peripherals such as temp sensor, light sensor, accelerometer,
 *   and trim potentiometer are monitored and values are written to
 *   the OLED display.
 *
 *   Copyright(C) 2010, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************/



#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "oled.h"
#include "temp.h"
#include "light.h"

int32_t temperature = 0;
int32_t pressure = 0;

static uint32_t msTicks = 0;
static uint8_t buf[10];

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    // the buffer must not be null and at least have a length of 2 to handle one
    // digit and null-terminator
    if (pBuf == NULL || len < 2)
    {
        return;
    }

    // a valid base cannot be less than 2 or larger than 36
    // a base value of 2 means binary representation. A value of 1 would mean only zeros
    // a base larger than 36 can only be used if a larger alphabet were used.
    if (base < 2 || base > 36)
    {
        return;
    }

    // negative value
    if (value < 0)
    {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    // calculate the required length of the buffer
    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        // the len parameter is invalid.
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);

    return;

}

void SysTick_Handler(void) {
    msTicks++;
}

static uint32_t getTicks(void)
{
    return msTicks;
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

int32_t calculatePressure()
{
	uint8_t bufor[3];

	uint8_t adres = 0x77;

	int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
	uint16_t ac4, ac5, ac6;

	int32_t b6, x3, b3, p;
	uint32_t b4, b7;

	bufor[0] = 0xAA;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac1 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xAC;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac2 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xAE;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac3 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xB0;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac4 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xB2;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac5 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xB4;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	ac6 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xB6;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	b1 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xB8;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	b2 = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xBA;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	mb = (bufor[0] << 8) | bufor[1];
	mb++;

	bufor[0] = 0xBC;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	mc = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xBE;
	I2CWrite(adres, bufor, 1);
	I2CRead(adres, bufor, 2);
	md = (bufor[0] << 8) | bufor[1];

	bufor[0] = 0xD0;
	I2CWrite(adres, bufor, 1);
	if (I2CRead(adres, bufor, 1) == 0 && bufor[0] == (uint8_t)0x55)
	{
		int32_t ut;

		bufor[0] = 0xF4;
		bufor[1] = 0x2E;
		I2CWrite(adres, bufor, 2);
		Timer0_Wait(5);
		bufor[0] = 0xF6;
		I2CWrite(adres, bufor, 1);
		I2CRead(adres, bufor, 2);

		ut = (bufor[0] << 8) | bufor[1];

		int16_t oss = 2;
		int32_t up;

		bufor[0] = 0xF4;
		bufor[1] = 0xB4; /*0x34 + (oss << 6)*/
		I2CWrite(adres, bufor, 2);
		Timer0_Wait(14);
		bufor[0] = 0xF6;
		I2CWrite(adres, bufor, 1);
		I2CRead(adres, bufor, 3);

		up = ((bufor[0] << 16) | (bufor[1] << 8) | bufor[2]) >> (8 - oss);

		int32_t x1, x2, b5;

		x1 = ((ut - ac6) * ac5) >> 15;
		x2 = (mc << 11) / (x1 + md);
		b5 = x1 + x2;

		b6 = b5 - 4000;
		x1 = ((int32_t)b2 * (b6 * b6 >> 12)) >> 11;
		x2 = (int32_t)ac2 * b6 >> 11;
		x3 = x1 + x2;
		b3 = ((((int32_t)ac1 * 4 + x3) << oss) + 2) >> 2;
		x1 = (int32_t)ac3 * b6 >> 13;
		x2 = ((int32_t)b1 * (b6 * b6 >> 12)) >> 16;
		x3 = ((x1 + x2) + 2) >> 2;
		b4 = (int32_t)ac4 * (uint32_t)(x3 + 32768) >> 15;
		b7 = ((uint32_t)up - b3) * (int32_t)(50000 >> oss);
		if (b7 < 0x80000000)
		{
			p = (b7 * 2) / b4;
		}
		else
		{
			p = (b7 / b4) * 2;
		}
		x1 = (p >> 8) * (p >> 8);
		x1 = (x1 * 3038) >> 16;
		x2 = (-7357 * p) >> 16;
		p = p + ((x1 + x2 + (int32_t)3791) >> 4);

		return p;
	}
}

void printData()
{

	oled_putString(1,1,  (uint8_t*)"Temp: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(1,12,  (uint8_t*)"Press: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	//Temperatura

	int32_t t_real = 0;
	int32_t t_remainder = 0;
	t_real = temperature / 10;
	t_remainder = temperature % 10;
	intToString((uint32_t)t_real, buf, 10, 10);

	oled_putString(48, 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	oled_putString(48 + 12, 1, (uint8_t *)".", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	intToString((uint32_t)t_remainder, buf, 10, 10);
	oled_putString(48 + 16, 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	oled_putPixel(48 + 25, 1, OLED_COLOR_BLACK);
	oled_putPixel(48 + 25, 2, OLED_COLOR_BLACK);
	oled_putPixel(48 + 26, 1, OLED_COLOR_BLACK);
	oled_putPixel(48 + 26, 2, OLED_COLOR_BLACK);

	oled_putString(48 + 28, 1, (uint8_t *)"C", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	//Cisnienie

	intToString((uint32_t)((pressure / 100) + 25), buf, 10, 10);
	oled_putString(48, 12, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(48 + 24, 12, (uint8_t *)"hPa", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

}

int main (void)
{

    int32_t t = 0;

    init_i2c();
    init_ssp();

    oled_init();

    temp_init (&getTicks);


	if (SysTick_Config(SystemCoreClock / 1000)) {
		    while (1);  // Capture error
	}

    oled_clearScreen(OLED_COLOR_WHITE);


    while(1) {

        temperature = temp_read();
        pressure = calculatePressure();

        printData();

        /* delay */
        Timer0_Wait(200);
    }

}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
