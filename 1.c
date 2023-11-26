#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned char new_key = 0;
volatile unsigned char old_key = 0;
volatile unsigned char key_flag = 0;
volatile unsigned char pressing_flag = 0;

//=============================================
void USART0_init();
void Timer_init();
void PORT_init();
unsigned char key_scan(void);
unsigned char key_analysis(unsigned char key, unsigned char Raw);
//=============================================
int main(void)
{
  USART0_init();
  Timer_init();
  PORT_init();
  while(1);
}

//=============================================
void USART0_init(){
  UCSR0A = 0x20;
  UCSR0B = 0X18;
  UBRR0 = 103;
}
void Timer_init(){
  TCCR0B = (1 << WGM02) | (0 << CS02) | | (1 << CS01) | (1 << CS00);
  OCR0A = 0x09C4;
  TIMSK0 = (1 << TOIE0) | (1 << OCIE0A);
}
void PORT_init(){
  DDRD = 0b11110000;
  DDRB = 0b00001111;
  PORTB = 0b00001111;
  
}
//=============================================
ISR(TIMER0_COMPA_vect){
  key_scan();
  if(key_flag){
    DDRC = 0x00000111;
    PORTC = 0x00000111;
  }
}
//=============================================
unsigned char key_scan(void){
  unsigned char Raw;
  unsigned char bit_shift = 0b000010000;
  
  if(key_flag) return -1;
  PORTD &= 0xf0;
  new_key = PINB & 0x0f;
  
  if(new_key != old_key){
    old_key = new_key;
    return -1;
  }
  if(pressing_flag){
    if(new_key == 0x0f){
      pressing_flag = 0;
      return -1;
    }
  }else return -1;
  else{
    if(new_key == 0x0f) return -1;
    else{
      for(Raw = 0; Raw < 3; Raw++){
        PORTD |= 0x70; //masking
        PORTD &= (~bit_shift); //to check specific raw
        new_key = PINB & 0x0f;
        
        if(new == 0x0f){
          bit_shift <<= 1;
          continue;
        }
        else {
          pressing_flag = 1;
          return key_analysis(new_key, Raw);
          
        }
      }
    }
  }
}

unsigned char key_analysis(unsigned char key, unsigned char Raw){
  key_flag = 1;
  switch(Raw){
    case 0:
      switch(key){
        case 0x01:
          return 1;
        case 0x02:
          return 4;
        case 0x04:
          return 7;
        case 0x08:
          return '*';  
      }
    case 1:
      switch(key){
        case 0x01:
          return 2;
        case 0x02:
          return 5;
        case 0x04:
          return 8;
        case 0x08:
          return 0;

      }
    case 2:
      switch(key){
        case 0x01:
          return 3;
        case 0x02:
          return 6;
        case 0x04:
          return 9;
        case 0x08:
          return '#';
      }
    default:
      break;
  }
}