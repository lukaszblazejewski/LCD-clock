#include "stm32f4xx.h"

#define LCD_MODE     		0x06		// I/D=1 - inkrementacja, S=0 - brak przesunięcia zawartości DDRAM
#define LCD_CLEAR    		0x01		// czyści wyświetlacz i ustawia kursor na początku (adres=0)
#define LCD_ON		 		0x0c		// D=1 - włączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
#define LCD_OFF		 		0x08		// D=0 - wyłączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
#define LCD_8bit_data 		0x38		// DL=1 - 8 linii danych, N=1 - dwu liniowy, F=0 - matryca 5x7 punktów
#define LCD_DISP_SHIFT		0x18		// Przesuwanie ekranu o jedno pole
#define _BV(bit) (1<<bit)
//przyporządkowanie sygnałom wyświetlacza LCD odpowiednich portów mikrokontrolera
#define	LCD_DATA_OUT	GPIOE->ODR
#define	LCD_CONTROL		GPIOD->ODR

// ustawienie sygnałów sterujacych dla rozpoczęcia zapisu instrukcji
#define LCD_WR_INST	GPIOD->BSRR=0b00000001100000000000000000000000
// ustawienie sygnałów sterujacych dla rozpoczęcia zapisu danej
#define LCD_WR_DATA	GPIOD->BSRR=0b00000001000000000000000010000000
// ustawienie sygnałów sterujacych dla rozpoczęcia odczytu flagi zajętości
#define LCD_RD_FLAG	GPIOD->BSRR=0b00000000100000000000000100000000
// ustawienie sygnałów sterujacych dla rozpoczęcia odczytu danej
#define LCD_RD_DATA	GPIOD->BSRR=0b00000000000000000000000110000000
// ustawienie sygnału sterujacego EN
#define LCD_EN_SET	GPIOD->BSRR=0b00000000000000000000001000000000
// wyzerowanie sygnału sterujacego EN
#define LCD_EN_CLEAR	GPIOD->BSRR = 0b00000010000000000000000000000000
// Ustawienie sygnalu sterujacego dla przesuwania napisu w lewo


void check_busy_flag (void)
{
	uint16_t i=0;
	char flag=0x80;
		  LCD_RD_FLAG;
		  GPIOE->MODER = 0x00000000;
			 for(i=0; i<15; i++)
				 asm("nop");		// odczekanie około 40 us
		  while (flag != 0){
			LCD_EN_SET;				// ustawienie linii EN
			 for(i = 0; i < 40; i++)
				 asm("nop");
			flag = ((GPIOD->IDR)&_BV(11));
			LCD_EN_CLEAR;			// zerowanie linii EN
			HAL_Delay(1);
		};
		  GPIOE->MODER = 0b00000000010101010101010100000000;
};

void send_LCDdata (unsigned char LCDdata) 		//wysłanie bajtu do LCD
{
	uint16_t i = 0;
	check_busy_flag();		// sprawdzenie flagi zajętości
	LCD_WR_DATA;			// ustawienie sygnałów sterujących dla rozpoczęcia zapisu danej
	 for( i= 0; i < 15; i++)
		 asm("nop");		// odczekanie około 40 us
	LCD_EN_SET;				// ustawienie linii EN
	LCD_DATA_OUT = (LCDdata << 4);	//przesłanie bajtu danych do LCD_DATA_OUT
	 for(i = 0; i < 40; i++)
		 asm("nop");
	LCD_EN_CLEAR;			// zerowanie linii EN
	HAL_Delay(1);
};

void send_LCDinstr (unsigned char LCDinstr)
{
	uint16_t i = 0;
	check_busy_flag();		// sprawdzenie flagi zajętości
	LCD_WR_INST;			// ustawienie sygnałów sterujących dla rozpoczęcia zapisu instrukcji
	 for(i = 0; i < 15; i++)
		 asm("nop");		// odczekanie około 40 us
	LCD_EN_SET;				// ustawienie linii EN
	LCD_DATA_OUT = (LCDinstr << 4);//przesłanie instrukcji do LCD_DATA_OUT
	 for(i = 0; i < 40; i++)
		 asm("nop");
	LCD_EN_CLEAR;			// zerowanie linii EN
	HAL_Delay(1);
}

void send_LCDinstr_without_check_flag (unsigned char LCDinstr)
{
	uint16_t i = 0;
	LCD_WR_INST;			// ustawienie sygnałów sterujących dla rozpoczęcia zapisu instrukcji
	 for(i = 0; i < 15; i++)
		 asm("nop");
	LCD_EN_SET;				// ustawienie linii EN
	LCD_DATA_OUT = (LCDinstr << 4);//przesłanie instrukcji do LCD_DATA_OUT
	 for(i = 0; i < 40; i++)
		 asm("nop");
	LCD_EN_CLEAR;			// zerowanie linii EN
	HAL_Delay(1);
};

void init_LCD(void)
{
	//inicjalizacja wyświetlacza LCD
	//maksymalna wartość opóźnienia generowanego przez funkcję _delay_ms wynosi 6.5535s z rozdzielczością 0.1 ms, a do czasu (2^16-1)*4/fclk[kHz]=16.38375 ms z rozdzielczością 2.5e-4 ms
		HAL_Delay(15); 	//odmierzenie 15ms po włączeniu zasilania
		send_LCDinstr_without_check_flag(LCD_8bit_data); //ustawia 8 bitową magistralę danych
		HAL_Delay(5); 	//odmierzenie 5 ms po pierwszym wywołaniu instrukcji LCD_8bit_data
	 	send_LCDinstr_without_check_flag(LCD_8bit_data); //ustawia 8 bitową magistralę danych
	 	HAL_Delay(1);
		send_LCDinstr_without_check_flag(LCD_8bit_data); //ustawia 8 bitową magistralę danych
	 	HAL_Delay(1);
		send_LCDinstr(LCD_8bit_data); //ustawia 8 bitową magistralę danych
		send_LCDinstr(LCD_OFF);	   // D=0 - wyłączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
		send_LCDinstr(LCD_CLEAR); //czyści wyświetlacz LCD i 'Entry mode set' przesuwa kursor +1
		send_LCDinstr(LCD_MODE);// I/D=1 - inkrementacja, S=0 - brak przesunięcia zawartości DDRAM
		send_LCDinstr(LCD_ON);	   // D=1 - włączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
	};

unsigned char address_DDRAM (unsigned char x, unsigned char y)
{
unsigned char xy;
switch(x)
	{
			case 1: 	xy = y-1; 		break;
			case 2:		xy = 0x40 + y - 1;	break;
			case 3:		xy = 0x14 + y - 1;	break;
			case 4:		xy = 0x54 + y - 1;	break;
			default:	xy = y - 1;
	};
return xy;
};
void set_cursor (unsigned char x, unsigned char y)
{
	send_LCDinstr(address_DDRAM(x, y) | 0x80);
}

void write_string_xy(unsigned char x, unsigned char y, char *str)	//pisz tekst na LCD wskazywany wskaźnikiem *text
{
	set_cursor(x, y);
	while (*str)
	{					 //sprawdzenie czy nie odczytano ostatniego znaku z wprowdzanego ciągu znaków
		send_LCDdata(*str++);  		 // umieść wskazany znak tekstu na LCD
	};
};
