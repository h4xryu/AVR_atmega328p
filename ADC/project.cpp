#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void ADC_init(void);
unsigned int ADC_read(char Ch);
void Port_init();
void Timer1_init();
void Timer1_delay();

volatile unsigned int Value_ADC0;
volatile unsigned int Value_ADC5;
volatile unsigned char Flag_ADC = 0;
volatile unsigned char ADC_ch = 0;
volatile unsigned int ADC_result = 0;

int main(void){

	sei();
 	Port_init();
	ADC_init();
	Timer1_init();
	
	
	while(1)
	{	
		
	
	  	if(Flag_ADC){
			ADC_ch = (ADC_ch + 1) & 0x05;
			Flag_ADC = 0;
		}
		Value_ADC5 = ADC_read(ADC_ch);
		//PORTB = (unsigned char) (Value_ADC0);
		//PORTD = (unsigned char) (Value_ADC0 >> 6);
		PORTB = (unsigned int) (Value_ADC5);	
       	PORTD = (unsigned int) (Value_ADC5 >> 4);

	  	_delay_ms(100);
	
		PORTB = 0x00;
		PORTD = 0x00;
	
		

    }
    return 0;
	main();
}

void ADC_init(void){
    ADMUX = (0 << REFS0) | (1 << REFS1) | (0 << ADLAR);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

ISR(TIMER1_COMPA_vect){
	if(!Flag_ADC){
		ADMUX += ADC_ch;
		ADCSRA |= (1 << ADSC);
		ADCSRA |= (1 << ADIE);
	}
}

ISR(ADC_vect){
	ADC_result = ADC;
	ADCSRA &= ~(1 << ADIE);
	Flag_ADC = 1;
}

unsigned int ADC_read(char Ch){
   	ADMUX |= (0x05 & 0x0F);
   	ADCSRA |= (1<<ADSC);
   	while(!(ADCSRA & 0x10)){} // repeat until interrupt bit set 1
   	return ADC;
}

void Timer1_init(void){
	TCCR1B = 0x0B;
	OCR1AH = 0x61;
	OCR1AL = 0xA8;
}

void Timer1_delay(void){
	while(!(TIFR1 & 0x02));
	TIFR1 |= (0x02);
}

void Port_init(){
	//PORTD = 0x20;
	DDRD = 0x20;
	//PORTB = 0x01;
	DDRB = 0x01;
}