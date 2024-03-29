#include "stm32f4xx.h"

#define LCD_MODE     		0x0006		// I/D=1 - inkrementacja, S=0 - brak przesunięcia zawartości DDRAM
#define LCD_CLEAR    		0x0001		// czyści wyświetlacz i ustawia kursor na początku (adres=0)
#define LCD_ON		 		0x000c		// D=1 - włączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
#define LCD_OFF		 		0x0008		// D=0 - wyłączenie wyświetlacza, C=0 - wyłączenie kursora, B=0 - wyłączenie migania kursora
#define LCD_8bit_data 		0x0038		// DL=1 - 8 linii danych, N=1 - dwu liniowy, F=0 - matryca 5x7 punktów
#define LCD_DISP_SHIFT		0x0018		// Przesuwanie ekranu o jedno pole
#define _BV(bit) (1<<bit)
//przyporządkowanie sygnałom wyświetlacza LCD odpowiednich portów mikrokontrolera
#define	LCD_DATA_OUT	GPIOE->ODR

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



void check_busy_flag (void);
void send_LCDdata (unsigned char LCDdata);
void send_LCDinstr (unsigned char LCDinstr) ;
void send_LCDinstr_without_check_flag (unsigned char LCDinstr);
void init_LCD(void);
unsigned char address_DDRAM (unsigned char x, unsigned char y);
void set_cursor (unsigned char x, unsigned char y);
void write_string_xy(unsigned char x, unsigned char y, char *str);
