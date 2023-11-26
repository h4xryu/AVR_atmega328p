#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
//====================================================
#define Btn1 2
#define RED 0
#define BLUE 1
#define GREEN 2
#define chk_NewKey(new_key) new_key & 0b00001000 || new_key & 0b00000100 || new_key & 0b00000010 || new_key & 0b00000001
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//====================================================
volatile unsigned char job = 0;
volatile unsigned char key_flag = 0;
volatile unsigned char new_key = 0;
volatile unsigned char key_data = 0;
    
volatile unsigned char pressing_flag = 0;
volatile unsigned char keyNum = 0;
volatile unsigned char current_key = 0;
volatile unsigned char old_key = 0;
volatile unsigned char flag_time = 0;
//====================================================
void Timer0_init();
void Timer2_init();
void Port_init();
unsigned char key_scan();
unsigned char key_analysis(unsigned char , unsigned char);
void LED_ON();
void LED_OFF();
void USART_init();
void Tx_char(unsigned char data);
void setup_watchdog();
//====================================================
int main(void){
    setup_watchdog();
    Port_init();
    Timer2_init();
    USART_init();
    sei();
    
    // EICRA = 0x03;
    // EIMSK |= (1 << INT0);
	while(1){
        if(flag_time){
            flag_time = 0;
        }
        if(key_flag){
            key_flag = 0;
            if(key_data == -1){
                return ;
            }
            if(key_data == 11){
                //*
            }
            if(key_data == 12){
                //#
            }
            Tx_char('0'+key_data);
            Tx_char(' ');
            LED_ON();
        }
        else{
            LED_OFF();
        }
    }
}
//====================================================
void setup_watchdog() {
  // WDTCSR: Watchdog Timer Control Register
  // WDCE (Watchdog Change Enable)
  // WDTO_1S: Watchdog Timeout 
  WDTCSR |= (1<<WDE) | (1<<WDCE) | (1<<WDIE) | (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); 
}
//===================================================
void Timer0_init(){
    TCCR0B = (1 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);
    OCR0A = 0x07c4; //0.0625 * 25000;
    TIMSK0 = (1 << TOIE0) | (1 << OCIE0A);
}

void Timer2_init(){
    TCCR2A = 0;
    TCCR2B = (1 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);
    OCR2A = 0x09c4; //0.0625 * 25000;
    TIMSK2 = (1 << TOIE2) | (1 << OCIE2A);
}

void Port_init(){

    //set B0~D4 as input,  D5, D6, D7 pins are set high to find key vals 
    DDRD = 0b01110000;
    PORTD = 0b00000000;
    DDRB = 0b00000000;

}
//====================================================
void LED_ON(){
    DDRC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);
    PORTC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);
}

void LED_OFF(){
    //DDRC &= ~(1 << RED) & ~(1 << GREEN) & ~(1 << BLUE);
    PORTC &= ~(1 << RED) & ~(1 << GREEN) & ~(1 << BLUE);
}
//====================================================
ISR(TIMER0_COMPA_vect){
    
    flag_time = 1;
    key_data = key_scan();

    //PORTC ^= 0xff;
}

ISR(TIMER2_OVF_vect){
    
    flag_time = 1;
    key_data = key_scan();

    //PORTC ^= 0xff;
}



ISR(INT0_vect){
    
}

//====================================================
void USART_init(){
    UCSR0A = 0x20;
    UCSR0B = 0x18;
    UCSR0C = 0x06;
    UBRR0 = 9;
}

void Tx_char(unsigned char data){
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}
//====================================================
unsigned char key_scan(){

    unsigned char Raw;
    unsigned char bit_shift = 0b00010000;

    if(key_flag) return -1;

    PORTD &= ~(0x70);
    new_key = (PINB) & 0x0f;



    /* procedure to prevent signals chattering */
    if(new_key != old_key){  
        old_key = new_key;
        return -1;
    }


    if(pressing_flag){ //checking if btn released
        //pin 1 ,3, 5 => D4, D5, D6 / pin 2,7,6,4 => D0, D1, D2, D3
        if(new_key == 0x0f){ //compare with current key and the received new key
                             //because if one of btns is pressed, it can get signals 
            pressing_flag = 0; 

            return -1;
        }
        else return -1;
    }

    else { //checking if key pressed has a diffrent value with the previous key 
        if (new_key == 0x0f){

            return -1;
        }
        else {
            

            for(Raw = 0; Raw < 3; Raw++){
                PORTD |= 0x70; 
                PORTD &= ~(bit_shift); //to check specific raw, Ex) raw 1 -> D5, D6 must be HIGH state
                //in addtion if pin of PORTD is on, PINB won't be low state

                new_key = (PINB) & 0x0f;

                //key = ~new_key;
                //key &= 0x0f;
        
                if(new_key == 0x0f){ // + when raw is LOW
                   //Tx_char('a');
                    bit_shift <<= 1;
                    continue;
                }
                else {
                    //Tx_char('b');
                    pressing_flag = 1;
                    key_flag = 1;
                    return key_analysis(new_key, Raw);
                }
            }
        
        }
    }

    /* end checking */
}


unsigned char key_analysis(unsigned char key, unsigned char Raw){


  switch(Raw){
    case 0:
        switch(key){

            case 0b11100000:
                key_flag = 1;
                return 1;            
            case 0b11010000:
                key_flag = 1;
                return 4;
            case 0b10110000:
                key_flag = 1;
                return 7;
            case 0b01110000:
                return 10; 
            default:
                return -1;
        }

    case 1:
        switch(key){
            case 0b11100000:
                return 2;
            case 0b11010000:
                return 5;
            case 0b10110000:
                return 8;
            case 0b01110000:
                return 0;
            default:
                return -1;
        }

    case 2:
        switch(key){
            case 0b11100000:
                return 3;
            case 0b11010000:
                return 6;
            case 0b10110000:
                return 9;
            case 0b01110000:
                return 11;
            default:
                return -1;
        }
    default:
        return -1;
  }
}
//====================================================