#include <avr/io.h>
#include <stdio.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define TRIG_HIGH() PORTD |= 0x08
#define TRIG_LOW() PORTD &= ~(0x08)
#define ECHO_IN (PIND &= 0x04)
#define T_RESOLUTION 5 // delay

unsigned char Distance_counter = 0;
unsigned int High_counter = 0;
float Distance_CM = 0.0;

volatile unsigned char Flag_distance_start = 0;
volatile unsigned char Flag_distance_end = 0;
volatile unsigned char Flag_edge = 0;
volatile unsigned int High_time = 0;


float getEcho(void){
  Distance_counter = 0;
  Distance_CM = 0.0;
  TRIG_HIGH();
  _delay_us(250);
  TRIG_LOW();
  while(!(PIND & 0x04));
  TCNT1 = 0;
  TCCR1B = 2; //timer start
  while((PIND & 0x04));
  TCCR1B = 0;
  High_counter = TCNT1;
  return ((float)High_counter/2)*T_RESOLUTION*0.03465;

}

void setup() {
  DDRD = 0b00001000;
  sei();
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop(){
  if(!Flag_distance_start){
    TRIG_HIGH();
    _delay_us(10);
    TRIG_LOW();
    EICRA = 0b00000011; //INT0 rising edge
    Flag_edge = 1;
    Flag_distance_start = 1;
    EIMSK = (1<<INT0);
  }
  if(Flag_distance_end){
    Flag_distance_end = 0;
    Flag_distance_start = 0;
    Distance_CM = (float)(High_time*0.5)*0.03465/2;
    Serial.print("Distance : ");
    Serial.println(Distance_CM);
    _delay_ms(100);
  }
  
}

ISR(INT0_vect){
  if(!Flag_distance_start) {
    EIMSK &= ~(1 << INT0);
    return;
  }
  if(Flag_edge){ //rising edge
    High_time = 0;
    TCNT1 = 0;
    TCCR1B = 2; 
    EICRA = 0b00000010;
    Flag_edge = 0;
  }
  else {
    TCCR1B = 0;
    High_time = TCNT0;
    Flag_distance_end = 1;
    EIMSK &= ~(1<<INT0);
  }
}