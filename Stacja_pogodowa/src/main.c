#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_rtc.h"

#include "oled.h"
#include "temp.h"
#include "light.h"
#include "joystick.h"
#include "led7seg.h"

/* Zmienne na dane odczytane z czujnikow */
static int32_t temperature = 0;
static int32_t pressure = 0;
static int16_t humidity = 0;

/* Typ wyliczeniowy do wybrania konkretnej cyfry w dacie i godzinie */
enum joystickMovement
{
    LEFT,
    CENTER,
    RIGHT
};

/* Typ wyliczeniowy do wyboru miedzy data, a czasem */
enum dateOrTime
{
	DATE,
	TIME
};


/* Zmienne pomocnicze/sterujace */
static uint32_t msTicks = 0;
static uint8_t buf[10];
static enum dateOrTime dateTime = DATE;
static enum joystickMovement jMov = LEFT;
static RTC_TIME_Type rtc;

/*!
 *  @brief    Procedura zamienia wartosc calkowita na lancuch znakow
 *  @param 	  value
 *              Wartosc zmieniana na lancuch znakow
 *  @param 	  pBuf
 *              Bufor
 *  @param 	  len
 *              Dlugosc bufora
 *  @param 	  base
 *              System liczbowy (np. 2 - binarny, 10 - dziesietny)
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    if (pBuf == NULL || len < 2)
    {
        return;
    }

    if (base < 2 || base > 36)
    {
        return;
    }

    if (value < 0)
    {
        tmpValue = -tmpValue;
        value = -value;
        pBuf[pos] = '-';
        pos++;
    }

    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value = value / base;
    } while(value > 0);

    return;
}

/*!
 *  @brief    Procedura do obslugi tickow systemowych
 *  @param 	  Brak
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void SysTick_Handler(void) {
    msTicks++;
}

/*!
 *  @brief    Getter zmiennej msTicks
 *  @param 	  Brak
 *  @returns  msTicks
 *  @side_effects:
 *            Brak
 */
static uint32_t getTicks(void)
{
    return msTicks;
}

/*!
 *  @brief    Procedura sluzaca do inicjalizacji kontrolera SSP
 *  @param 	  Brak
 *  @returns  Nic
 *  @side_effects:
 *            Konfiguracja pinow i zmiany w rejestrze SSP
 */
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

	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	SSP_Cmd(LPC_SSP1, ENABLE);

}

/*!
 *  @brief    Procedura sluzaca do inicjalizacji interfejsu I2C2
 *  @param 	  Brak
 *  @returns  Nic
 *  @side_effects:
 *            Konfiguracja pinow i zmiany w rejestrze I2C
 */
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	I2C_Init(LPC_I2C2, 100000);

	I2C_Cmd(LPC_I2C2, ENABLE);
}

/*!
 *  @brief    Funkcja obliczajaca aktualne cisnienie atmosferyczne
 *  @param 	  Brak
 *  @returns  32-bitowa liczba calkowita oznaczajaca cisnienie
 *            W przypadku problemow z komunikacja z czujnikiem zwraca 0.
 *  @side_effects:
 *            Brak
 */
int32_t calculatePressure(void)
{
	uint8_t communicationBuffer[3];

	uint8_t deviceAddress = 0x77;

    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;

    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;

    int32_t b6;
    int32_t x3;
    int32_t b3;
    int32_t p = 0;
    uint32_t b4;
    uint32_t b7;

	communicationBuffer[0] = 0xAA;

	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac1 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xAC;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac2 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xAE;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac3 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xB0;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac4 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xB2;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac5 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xB4;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	ac6 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xB6;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	b1 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xB8;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	b2 = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xBA;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	mb = (communicationBuffer[0] << 8) | communicationBuffer[1];
	mb++;

	communicationBuffer[0] = 0xBC;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	mc = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xBE;
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2);
	md = (communicationBuffer[0] << 8) | communicationBuffer[1];

	communicationBuffer[0] = 0xD0;

	I2CWrite(deviceAddress, communicationBuffer, 1);

    if ((I2CRead(deviceAddress, communicationBuffer, 1) == 0) && (communicationBuffer[0] == (uint8_t)0x55))
	{
		int32_t ut;

		communicationBuffer[0] = 0xF4;
		communicationBuffer[1] = 0x2E;
		I2CWrite(deviceAddress, communicationBuffer, 2);
		Timer0_Wait(5);

		communicationBuffer[0] = 0xF6;
		I2CWrite(deviceAddress, communicationBuffer, 1);
		I2CRead(deviceAddress, communicationBuffer, 2);

		ut = (communicationBuffer[0] << 8) | communicationBuffer[1];

		int16_t oss = 2;
		int32_t up;

		communicationBuffer[0] = 0xF4;
		communicationBuffer[1] = 0xB4;

		I2CWrite(deviceAddress, communicationBuffer, 2);
		Timer0_Wait(14);
		communicationBuffer[0] = 0xF6;
		I2CWrite(deviceAddress, communicationBuffer, 1);
		I2CRead(deviceAddress, communicationBuffer, 3);

		up = ((communicationBuffer[0] << 16) | (communicationBuffer[1] << 8) | communicationBuffer[2]) >> (8 - oss);

        int32_t x1;
        int32_t x2;
        int32_t b5;

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
        if (b7 < 0x80000000U)
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
	}
	else
	{
        led7seg_setChar('3', FALSE);
	}
    return p;
}

/*!
 *  @brief    Funkcja obliczajaca wilgotnosc powietrza
 *  @param    Brak
 *  @returns  Liczba calkowita 16-bitowa oznaczajaca obliczona wilgotnosc powietrza.
 *            W przypadku problemow z komunikacja z czujnikiem zwraca 0.
 *  @side_effects:
 *            Brak
 */
int16_t calculateHumidity(void)
{
	uint8_t deviceAddress = 0x40;
	uint16_t srh = 0;
	int16_t rh = 0;
    int16_t result = 0;

	uint8_t communicationBuffer[3];

	communicationBuffer[0] = 0xF5;

	if (I2CWrite(deviceAddress, communicationBuffer, 1) == 0)
	{
		Timer0_Wait(13);
		I2CRead(deviceAddress, communicationBuffer, 2);
		srh = ((communicationBuffer[0] << 8) | communicationBuffer[1]) & 0xFFFC;
        rh = -6 + 125 * (long)srh / 65536;

        result = rh;

		if (rh < 0)
        {
            result = 0;
        }
        if (rh > 100)
        {
            result = 100;
        }

	}
	else
	{
		led7seg_setChar('3', FALSE);
	}
    return result;
}

/*!
 *  @brief    Procedura wyswietlajaca dane z czujnikow
 *  @param    Brak
 *  @returns  Nic
 *  @side_effects:
 *            Zmiana zmiennej globalnej buf
 */
void printData(void)
{
    oled_putString(1,1, (uint8_t*)"Temp: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,12, (uint8_t*)"Press: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,23, (uint8_t*)"Humidity: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	int32_t integerPart = 0;
	int32_t fractionPart = 0;
	integerPart = temperature / 10;
	fractionPart = temperature % 10;

	intToString((uint32_t)integerPart, buf, 10, 10);
	oled_putString(48, 1, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	oled_putString(60, 1, (uint8_t *)".", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	intToString((uint32_t)fractionPart, buf, 10, 10);
	oled_putString(64, 1, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	oled_putPixel(73, 1, OLED_COLOR_WHITE);
	oled_putPixel(73, 2, OLED_COLOR_WHITE);
	oled_putPixel(74, 1, OLED_COLOR_WHITE);
	oled_putPixel(74, 2, OLED_COLOR_WHITE);
	oled_putString(76, 1, (uint8_t *)"C", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	intToString((uint32_t)((pressure / 100) + 25), buf, 10, 10);
	oled_putString(48, 12, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(72, 12, (uint8_t *)"hPa", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	intToString(humidity, buf, 10, 10);
	oled_putString(58, 23, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(80, 23, (uint8_t *)"%", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	if (humidity < 100)
	{
		oled_putString(70, 23, (uint8_t *)" ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
}

/*!
 *  @brief    Setter czasu
 *  @param 	  h
 * 			  Godzina
 *  @param 	  m
 * 			  Minuta
 *  @param 	  s
 * 			  Sekunda
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void setTime(uint32_t h, uint32_t m, uint32_t s)
{
	rtc.HOUR = h;
	rtc.MIN = m;
	rtc.SEC = s;

	RTC_SetFullTime(LPC_RTC, &rtc);
}

/*!
 *  @brief    Setter daty
 *  @param 	  d
 * 			  Dzien miesiaca
 *  @param 	  m
 * 			  Miesiac
 *  @param 	  y
 * 			  Rok
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void setDate(uint32_t d, uint32_t m, uint32_t y)
{
	rtc.DOM = d;
	rtc.MONTH = m;
	rtc.YEAR = y;

	RTC_SetFullTime(LPC_RTC, &rtc);
}

/*!
 *  @brief    Procedura ustawia linie podkreslajaca na konkretnej pozycji na ekranie
 *  @param 	  Brak
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void underline(void)
{
	oled_line(8 + jMov * 20, 12 + dateTime * 16, 20 + jMov * 20, 12 + dateTime * 16, OLED_COLOR_WHITE);
	oled_line(8 + jMov * 20, 12 + 1 + dateTime * 16, 20 + jMov * 20, 12 + 1 + dateTime * 16, OLED_COLOR_WHITE);
}

/*!
 *  @brief    Procedura usuwa linie z konkretnej pozycji na ekranie
 *  @param 	  Brak
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void refreshLine(void)
{
    oled_line(0, 12, 96, 12, OLED_COLOR_BLACK);
    oled_line(0, 12 + 1, 96, 12 + 1, OLED_COLOR_BLACK);

    oled_line(0, 12 + 16, 96, 12 + 16, OLED_COLOR_BLACK);
    oled_line(0, 12 + 1 + 16, 96, 12 + 1 + 16, OLED_COLOR_BLACK);
}

/*!
 *  @brief    Procedura zmieniajaca pozycje linii na ekranie zaleznie od pozycji joysticka
 *  @param 	  joyMove
 * 			  Pozycja joysticka
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void moveLine(int joyMove)
{
    if (joyMove == JOYSTICK_LEFT)
    {
        jMov--;
    }
    if (joyMove == JOYSTICK_RIGHT)
    {
        jMov++;
    }
    if (joyMove == JOYSTICK_CENTER)
    {
        if (dateTime == DATE)
        {
            dateTime = TIME;
        }
        else if (dateTime == TIME)
        {
            dateTime = DATE;
        }
    }

    if (jMov < LEFT)
    {
        jMov = 0;
    }
    if (jMov > RIGHT)
    {
        jMov = 2;
    }
}

/*!
 *  @brief    Funkcja zmieniajaca wartosc daty i godziny w chwili przekroczenia wartosci minimalnej lub maksymalnej
 *              (np. gdy sekudny osiagna wartosc 60, funkcja ustawi wartosc sekund na 0)
 *  @param 	  value
 * 			  Aktulana wartosc daty/godziny
 *  @param 	  minValue
 * 			  Minimalna wartosc zmiennej
 *  @param 	  maxValue
 * 			  Maksymalna wartosc zmiennej
 *  @returns  Liczba calkowita 32-bitowa bedaca poprawiona (lub nie) wartoscia konkretnej cyfry z daty/godziny
 *  @side_effects:
 *            Brak
 */
uint32_t handleTimeAndDateLimit(uint32_t value, uint32_t minValue, uint32_t maxValue)
{
    if (value < minValue)
    {
        return maxValue;
    }
    if (value > maxValue)
    {
        return minValue;
    }
    return value;
}

/*!
 *  @brief    Procedura sluzaca do zmiany konkretnych wartosci w dacie/godzinie
 *  @param 	  joyMove
 * 			  Pozycja joysticka
 *  @returns  Nic
 *  @side_effects:
 *            Brak
 */
void modifyDateOrTime(int joyMove)
{
    uint32_t grid[2][3];

    grid[0][0] = rtc.DOM;
    grid[0][1] = rtc.MONTH;
    grid[0][2] = rtc.YEAR;

    grid[1][0] = rtc.HOUR;
    grid[1][1] = rtc.MIN;
    grid[1][2] = rtc.SEC;

    if (joyMove == JOYSTICK_UP)
    {
        grid[dateTime][jMov]++;
    }
    if (joyMove == JOYSTICK_DOWN)
    {
        grid[dateTime][jMov]--;
    }

    grid[0][0] = handleTimeAndDateLimit(grid[0][0], 1, 31);
    grid[0][1] = handleTimeAndDateLimit(grid[0][1], 1, 12);
    grid[0][2] = handleTimeAndDateLimit(grid[0][2], 1000, 3000);

    grid[1][0] = handleTimeAndDateLimit(grid[1][0], 0, 23);
    grid[1][1] = handleTimeAndDateLimit(grid[1][1], 0, 59);
    grid[1][2] = handleTimeAndDateLimit(grid[1][2], 0, 59);

    switch (dateTime)
    {
        case DATE:
        {
            setDate(grid[0][0], grid[0][1], grid[0][2]);
            break;
        }
        case TIME:
        {
            setTime(grid[1][0], grid[1][1], grid[1][2]);
            break;
        }
        default:
        {
            break;
        }
    }
}

/*!
 *  @brief    Procedura wyswietlajaca date i godzine
 *  @param    Brak
 *  @returns  Nic
 *  @side_effects:
 *            Zmiana zmiennej globalnej buf
 */
static void printTime(void)
{
	RTC_GetFullTime(LPC_RTC, &rtc);

    if ((rtc.DOM >= 0) && (rtc.DOM <= 9))
	{
		intToString((uint32_t)0, buf, 10, 10);
		oled_putString((8), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		intToString((uint32_t)rtc.DOM, buf, 10, 10);
		oled_putString((14), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	else
	{
		intToString((uint32_t)rtc.DOM, buf, 10, 10);
		oled_putString((8), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	oled_putString((20), 2, (uint8_t *)".", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    if ((rtc.MONTH >= 0) && (rtc.MONTH <= 9))
	{
		intToString((uint32_t)0, buf, 10, 10);
		oled_putString((26), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		intToString((uint32_t)rtc.MONTH, buf, 10, 10);
		oled_putString((32), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	else
	{
		intToString((uint32_t)rtc.MONTH, buf, 10, 10);
		oled_putString((26), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	oled_putString((38), 2, (uint8_t *)".", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	intToString((uint32_t)rtc.YEAR, buf, 10, 10);
	oled_putString((44), 2, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    if ((rtc.HOUR >= 0) && (rtc.HOUR <= 9))
	{
		intToString((uint32_t)0, buf, 10, 10);
		oled_putString((8), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		intToString((uint32_t)rtc.HOUR, buf, 10, 10);
		oled_putString((14), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	else
	{
		intToString((uint32_t)rtc.HOUR, buf, 10, 10);
		oled_putString((8), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	oled_putString((20), 18, (uint8_t *)":", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    if ((rtc.MIN >= 0) && (rtc.MIN <= 9))
	{
		intToString((uint32_t)0, buf, 10, 10);
		oled_putString((26), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		intToString((uint32_t)rtc.MIN, buf, 10, 10);
		oled_putString((32), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	else
	{
		intToString((uint32_t)rtc.MIN, buf, 10, 10);
		oled_putString((26), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	oled_putString((38), 18, (uint8_t *)":", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    if ((rtc.SEC >= 0) && (rtc.SEC <= 9))
	{
		intToString((uint32_t)0, buf, 10, 10);
		oled_putString((44), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		intToString((uint32_t)rtc.SEC, buf, 10, 10);
		oled_putString((50), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	else
	{
		intToString((uint32_t)rtc.SEC, buf, 10, 10);
		oled_putString((44), 18, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	uint8_t joy = 0;
	underline();
	joy = joystick_read();
	if ((joy & JOYSTICK_CENTER) != 0)
	{
		refreshLine();
		moveLine(JOYSTICK_CENTER);
	}

	if ((joy & JOYSTICK_LEFT) != 0)
	{
		refreshLine();
		moveLine(JOYSTICK_LEFT);
	}
	if ((joy & JOYSTICK_RIGHT) != 0)
	{
		refreshLine();
		moveLine(JOYSTICK_RIGHT);
	}
	if ((joy & JOYSTICK_UP) != 0)
	{
		modifyDateOrTime(JOYSTICK_UP);
	}
	if ((joy & JOYSTICK_DOWN) != 0)
	{
		modifyDateOrTime(JOYSTICK_DOWN);
	}
}

int main (void)
{
    int16_t currentSite = 1;

    init_i2c();
    init_ssp();
    oled_init();
    joystick_init();
    RTC_Init(LPC_RTC);
    setTime(12, 0, 0);
	setDate(13, 6, 2024);
	RTC_Cmd(LPC_RTC, ENABLE);

	led7seg_init();

    temp_init (&getTicks);

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1) { }
	}

    oled_clearScreen(OLED_COLOR_BLACK);

    uint8_t leftButton = ((GPIO_ReadValue(0) >> 4) & 0x01);

    while(1)
    {

    	led7seg_setChar('0', FALSE);

        temperature = temp_read();
        pressure = calculatePressure();
        humidity = calculateHumidity();

        leftButton = ((GPIO_ReadValue(0) >> 4) & 0x01);

        if(leftButton == 0)
        {
        	switch(currentSite)
        	{
        		case 1:
				{
					currentSite = 2;
					oled_clearScreen(OLED_COLOR_BLACK);
					break;
				}
        		case 2:
				{
        			currentSite = 1;
        			oled_clearScreen(OLED_COLOR_BLACK);
        			break;
				}
                default:
                {
                    break;
                }
        	}
        }

        if(currentSite == 1)
        {
            printData();
        }

        if(currentSite == 2)
        {
    		printTime();
        }

        Timer0_Wait(200);
    }

}

void check_failed(void)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
    while(1) { }
}
