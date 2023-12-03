#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define DEGREEperUS 1.33
//=============================================================
volatile unsigned char Flag_start = 0;
volatile unsigned char Flag_end = 0;
volatile unsigned char avg_cnt = 0;
volatile unsigned char read_cnt = 0;
volatile unsigned long sum = 0;
volatile unsigned long angle_time;
volatile unsigned int overflow_cnt = 0;
volatile unsigned char angle_value = 0;
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
    angle_value = 0;


    EICRA = 0b00000011; //INT0 상승 에지 인터럽트 설정
    
    EIMSK |= 1<<INT0;

}
//=============================================================
ISR(INT0_vect){
    TCCR1B = 0;
    TCNT1 = 0;
    if(!Flag_start){

        EIMSK &= ~(1<<INT0);
        return;
    }

    TCCR1B =  (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);
    TIMSK1 |=  (1 << OCIE1A);
    if(read_cnt != (avg_cnt+1)){
        
        read_cnt ++;
        return;
    }
    TIMSK1 &= ~(1 << OCIE1A);
    
    
    angle_time = ((overflow_cnt)/avg_cnt);
    Flag_end=1;

}

ISR(TIMER1_COMPA_vect){

    overflow_cnt++;
}

ISR(TIMER2_COMPA_vect){
    if(!Flag_start){
            cli();
            sum = 0;
            overflow_cnt = 0;
            avg_cnt = 1;
            Flag_start = 1;       
            sei();
            EIMSK |= 1<<INT0;

        }
        if(Flag_end){
            //cli();
            EIMSK &= ~(1<<INT0);
            Flag_start = 0; 
            Flag_end = 0;
            angle_value = angle_time * DEGREEperUS;
            Tx_char('0'+angle_value);
            read_cnt = 0;
            _delay_us(10);
            //sei();
        }
}
//=============================================================
void Timer1_init(){
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