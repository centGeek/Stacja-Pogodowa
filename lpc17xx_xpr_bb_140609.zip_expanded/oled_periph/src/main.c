#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "oled.h"
#include "temp.h"
#include "light.h"

//---------------------------------------

//  Zmienne na dane odczytane z czujnikow

int32_t temperature = 0;
int32_t pressure = 0;

//---------------------------------------

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
	uint8_t communicationBuffer[3];

	uint8_t deviceAddress = 0x77; //   Dokumentacja BMP180, strona 20

	// Wartosci potrzebne do poprawnego odczytu cisnienia (przy okazji temperatury)
	// Wystepuja w pierwszym kroku algorytmu opisanego w dokumentacji czujnika
	int16_t ac1, ac2, ac3, b1, b2, mb, mc, md; //   Dokumentacja BMP180
	uint16_t ac4, ac5, ac6;                    //   Strona 15

	int32_t b6, x3, b3, p;
	uint32_t b4, b7;

	communicationBuffer[0] = 0xAA; // Adres rejestru z pierwsza czescia zmiennej kalibracyjnej (8 bitow MSB = Most Significant Bit)
								   // Dokumentacja strona 13
	                               // Czytamy z E2PROM czyli nieulotna pamiec tylko do odczytu
	I2CWrite(deviceAddress, communicationBuffer, 1);
	I2CRead(deviceAddress, communicationBuffer, 2); // Ostateczenie do bufora wczytalismy 8 bitow z rejestru 0xAA oraz 8 bitow z 0xAB (LSB = Least Significant Bit)
	ac1 = (communicationBuffer[0] << 8) | communicationBuffer[1]; // Po wczytaniu, "sklejamy" dwa poczatkowo oddzielne bajty
	 															  // w cala wartosc potrzebna do kalibracji


	// Analogicznie dla reszty
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

	communicationBuffer[0] = 0xD0; // Rejestr, dzieki ktoremu mozna sprawdzic czy komunikacja dziala
								   // Zgodnie z dokumentacja (strona 18), powienien przechowywac wartosc 0x55
	I2CWrite(deviceAddress, communicationBuffer, 1);
	// Jezeli z komunikacja wszystko w porzadku, przechodzimy do nastepnego kroku algorytmu z dokumentacji
	if (I2CRead(deviceAddress, communicationBuffer, 1) == 0 && communicationBuffer[0] == (uint8_t)0x55)
	{
		int32_t ut; // long

		communicationBuffer[0] = 0xF4; // Do tego rejestru
		communicationBuffer[1] = 0x2E; // Wpisujemy to
		I2CWrite(deviceAddress, communicationBuffer, 2);
		Timer0_Wait(5); // Dokumentacja zaleca odczekac 4.5ms.
		                // Z racji, ze ta prodecura przyjmuje tylko liczby calkowite - mamy zaokraglenie
		communicationBuffer[0] = 0xF6;
		I2CWrite(deviceAddress, communicationBuffer, 1);
		I2CRead(deviceAddress, communicationBuffer, 2);

		ut = (communicationBuffer[0] << 8) | communicationBuffer[1];

		int16_t oss = 2; // Oversampling w trybie "high resolution". Dokumentacja strona 12
		int32_t up;

		communicationBuffer[0] = 0xF4; // Do tego rejestru
		communicationBuffer[1] = 0xB4; // Wpisujemy to, czyli to co powstanie po podstawieniu do
		                               // wzoru z dokumentacji z algorytmu ( 0x34 + (oss << 6) )
		I2CWrite(deviceAddress, communicationBuffer, 2);
		Timer0_Wait(14);
		communicationBuffer[0] = 0xF6;
		I2CWrite(deviceAddress, communicationBuffer, 1);
		I2CRead(deviceAddress, communicationBuffer, 3);

		up = ((communicationBuffer[0] << 16) | (communicationBuffer[1] << 8) | communicationBuffer[2]) >> (8 - oss); // Wzor z algorytmu

		int32_t x1, x2, b5;


		// Tutaj co prawda zmierzamy do policzenia temperatury, lecz w naszym przypadku
		// potrzeba jedynie wspolczynnika B5, poniewaz wystepuje on w czesci obliczeniowej
		// dla wartosci cisnienia. Pomiar temepratury wyswietlany na ekranie wykonuje dla
		// nas zupelnie inny czujnik.
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

	// Nazwy wyswietlanych danych

	oled_putString(1,1,  (uint8_t*)"Temp: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(1,12,  (uint8_t*)"Press: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	// Temperatura

	int32_t integerPart = 0;
	int32_t fractionPart = 0;
	integerPart = temperature / 10;
	fractionPart = temperature % 10;

	// Z racji, ze przekazujemy procedurze odpowiedzialnej za wyswietlanie liczbe calkowita,
	// musimy "imitowac" ulamek poprzez wyswietlenie oddzielnie czesci calkowitej i ulamkowej.
	intToString((uint32_t)integerPart, buf, 10, 10);
	oled_putString(48, 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	oled_putString(60, 1, (uint8_t *)".", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	intToString((uint32_t)fractionPart, buf, 10, 10);
	oled_putString(64, 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	// Symbol stopni oraz jednostka
	oled_putPixel(73, 1, OLED_COLOR_BLACK);
	oled_putPixel(73, 2, OLED_COLOR_BLACK);
	oled_putPixel(74, 1, OLED_COLOR_BLACK);
	oled_putPixel(74, 2, OLED_COLOR_BLACK);
	oled_putString(76, 1, (uint8_t *)"C", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	//Cisnienie

	intToString((uint32_t)((pressure / 100) + 25), buf, 10, 10);
	oled_putString(48, 12, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(72, 12, (uint8_t *)"hPa", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

}

int main (void)
{

    init_i2c();
    init_ssp();
    oled_init();

	// Przekazujemy wskaznik na funkcje, ktora odmierza czas od uruchomienia programu.
	// Czas ten moze przekroczyc zakres zmiennej przez co przy tym samym uruchomieniu
	// moze w pewnym momencie zaczac liczyc od zera.
    temp_init (&getTicks);

	if (SysTick_Config(SystemCoreClock / 1000)) {
		    while (1);  // Capture error
	}

	// Wypelnienie calego wyswietlacza bialym kolorem
    oled_clearScreen(OLED_COLOR_WHITE);

    while(1) {

		// Czytamy potrzebne dane
        temperature = temp_read();
        pressure = calculatePressure();

		// Wyswietlamy zebrane dane
        printData();

		// Opoznienie w odswiezaniu
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
