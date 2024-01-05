#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
#include "I2C_MPU9250.h"
#include "Queue.h"
#define DEGREEperUS 1.33
//=============================================================

//=============================================================
void Timer2_init(void);
void Timer1_init(void);
void USART_init(void);
void PORT_init(void);
void Tx_char(unsigned char data);
//=============================================================
int main(void){
    sei();
    Timer1_init();
    Timer2_init();
	USART_init();
    PORT_init();


}
//=============================================================
ISR(INT0_vect){
    TCCR1B = 0;
    TCNT1 = 0;
    

}

ISR(TIMER1_COMPA_vect){

   
}

ISR(TIMER2_COMPA_vect){
    
}
//=============================================================
void Timer1_init(){ //400KHz
    TCCR1B =  (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);
	TCNT1 = 0;
    OCR1A = 0x0002;
    TIMSK1 |=  (1 << OCIE1A);
}

void Timer2_init(){
    TCCR2B =  (1 << WGM22) | (0 << CS22) | (1 << CS21) | (1 << CS20);
	TCNT2 = 0;
    OCR2A = 0xFF;
    OCR2B = 
    TIMSK2 |=  (1 << OCIE2A);
}

void USART_init(){
    UCSR0A = 0x20;
    UCSR0B = 0x18;
    UCSR0C = 0x06;
    UBRR0 = 8;
}
void PORT_init(){
    DDRD = 0x00;
    //PORTD = 0xFE;
}
//=============================================================
void Tx_char(unsigned char data){
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}