#include <avr/io.h>
#include <util/delay.h>

int main(void){
	
	DDRD = 0xFF;
	PORTD = 0xFF;
	unsigned char index = 0x00;
	unsigned char val = 0x01;
	
	while(1){

		val <<= 1;
		PORTD = ~val;
		if(val == 0x00){
			PORTD = 0b11111110;
			val = 0x01;
		}
		_delay_ms(1000);
		
	}		
}