
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	unsigned char data[10] = {0b00000010, 0b10011110, 0b00100100, 0b00001100,
						0b10011000, 0b01001000, 0b01000000, 0b00011110,
						0b00000000, 0b00001000};
	unsigned char index = 0;
	DDRC = 0xFF;
	PORTC = 0x00;
	DDRA = 0x03;
	PORTA = 0x00;
	
	
    /* Replace with your application code */
    while (1) 
    {
		PORTA = 0x01;
		PORTC = ~(data[index++]);
		index &= 0x0F;
		_delay_ms(100);
		PORTC = 0x00;
		PORTA = 0x02;
	
    }
}
