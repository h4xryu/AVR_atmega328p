#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	unsigned char value = 0x01;
	PORTC = 0xFF;
	DDRC = 0xFF;
	
	while(1){
		
		PORTC = 0xFF & ~(value);
		value <<= 1;
		value &= 0xff;
		_delay_ms(1000);
	
	}

}