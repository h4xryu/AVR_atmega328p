#include <avr/io.h>
#include <avr/interrupt.h>

#define CH_HUMIDITY 0
#define CH_TEMPERATURE 1 
#define FALLING_EDGE 0
#define RISING_EDGE 1

volatile unsigned char Flag_dht11_error = 0;
volatile unsigned char Flag_temperature = 0;
volatile unsigned char Flag_humidity = 0;
volatile unsigned char Flag_data_start = 0;
volatile unsigned char High_time = 0;
volatile unsigned char Low_time = 0;
volatile unsigned char Sig_channel = 0;
volatile unsigned char bit_counter = 0;
volatile unsigned char dht11_job;
volatile unsigned char Time_sum = 0;
volatile unsigned char Time_error_wait = 0;
volatile unsigned char Humidity = 0;
volatile unsigned int Temperature = 0;
volatile unsigned int I_buffer1 = 0;
volatile unsigned int I_buffer2 = 0;
volatile unsigned char Flag_edge = 0;

void interrupt_edge_set(unsigned char ch, unsigned char Flag_edge){
  unsigned char edge;
  TIMSK0 &= ~(1 << TOIE0);
  if(Flag_edge == FALLING_EDGE){
    edge = 0x02; //ISC01, ISC00 -> 1 0 -> Falling edge
  }
  else{
    edge = 0x03; //ISC01, ISC00 -> 0 1 -> Rising edge
  }

  if (ch == CH_TEMPERATURE) edge <<= 2; //INT1 edge 0x0000xx00
  EICRA = edge;
  TCNT0 = 0;
  TIFR0;
  TIMSK0 |= (1 << TOIE0);

}

void ext_interrupt_set(unsigned char ch){
  dht11_job = 0;
  Flag_data_start = 0;
  if(ch == CH_TEMPERATURE){
    interrupt_edge_set(CH_TEMPERATURE, FALLING_EDGE);
    Sig_channel = CH_TEMPERATURE;
    EIMSK |= (1 << INT1) & ~(1 << INT1);
  } else if (ch == CH_HUMIDITY){
    interrupt_edge_set(CH_HUMIDITY, RISING_EDGE);
    Sig_channel = CH_HUMIDITY;
    EIMSK |= (1 << INT0) & ~(1 << INT0);
  }
  
}

void init_dht11(void){
  ext_interrupt_set(CH_TEMPERATURE);
  TIMSK0 |= 1 << TOIE0;
  OCR0A = 0;
  TIFR0;
}

void dht11_error(void){
  EIMSK = 0;
  Time_error_wait = 100;
  Flag_dht11_error = 1;
  Flag_data_start = 0;
}

unsigned char EvenParityCheck(unsigned int temp_value){
  unsigned char i, parity = 0;
  for(i = 0; i < 9; i++){
    if(temp_value & (1 << i)) parity++;
    if(parity % 2) return false;
  }
  return true;
}

/*ISR(TIMER0_OVF_vect){
  if(Time_error_wait) Time_error_wait --;
  if(Flag_data_start) dht11_error();
}*/

ISR(INT0_vect){
  unsigned char bit_ratio = 0;
  unsigned int I_value = 0, sum = 0;

  if(Flag_edge == RISING_EDGE){
    Low_time = TCNT0;
    interrupt_edge_set(CH_HUMIDITY, FALLING_EDGE);
    if(bit_counter != 8) return;
  }
  else {
    High_time = TCNT0;
    interrupt_edge_set(CH_HUMIDITY, RISING_EDGE);
    if(bit_counter != 8) Time_sum = (High_time + Low_time) << 4;

    bit_ratio = Time_sum/Low_time;

    switch(dht11_job) 
    {
      /*---------------------------------------------------------------------------------*/
      /*                                      습도 부분                                    */
      /*---------------------------------------------------------------------------------*/
      case 0:
        if((bit_ratio>16) && (bit_ratio < 26)) bit_ratio = 1;
        if((bit_ratio > 27) && (bit_ratio < 48)) {Flag_data_start = 1; bit_counter= 0; Humidity |= 0x0001;}
        else dht11_error();
        return;
    
      case 1:
        if((bit_ratio > 16)&&(bit_ratio <=26)) Humidity |= 0x0001;
        else if((bit_ratio > 26)&&(bit_ratio <= 48)) Humidity &= 0xFFFE;
        else dht11_error();
        Humidity <<= 1; bit_counter ++;
        if(bit_counter != 9) return;
        if(!EvenParityCheck(Humidity)) {
          init_dht11();
          return;
        }
        Flag_humidity = 1; 
        Flag_data_start = 0;
        ext_interrupt_set(CH_TEMPERATURE);
        break;

      /*---------------------------------------------------------------------------------*/
      /*                                      온도 부분                                    */
      /*---------------------------------------------------------------------------------*/
      case 3:
        if((bit_ratio > 27) && (bit_ratio < 48)) {Flag_data_start = 1; bit_counter= 0; I_buffer1 = 0; I_buffer2 = 0; dht11_job++;}
        else dht11_error();
        return;
    
      case 4:
        if((bit_ratio > 16)&&(bit_ratio <=26)) I_buffer1 |= 0x0001;
        else if((bit_ratio > 26)&&(bit_ratio <= 48)) I_buffer1 &= 0xFFFE;
        else dht11_error();
        I_buffer1 <<= 1; bit_counter ++;
        if(bit_counter == 9) dht11_job++;
        break;
      case 5:
        if((bit_ratio > 16) && (bit_ratio <= 26)) I_buffer2 |= 0x0001;
        else if ((bit_ratio > 26)&&(bit_ratio <= 48)) I_buffer2 &= 0xFFFE;
        else dht11_error();
        I_buffer2 <<= 1; bit_counter ++;

        if(bit_counter != 18) return;
        if(!EvenParityCheck((I_buffer1 >> 1) << 8) || !EvenParityCheck(I_buffer2)) {init_dht11(); return;}
        Flag_temperature = 1; Flag_data_start = 0;
        ext_interrupt_set(CH_HUMIDITY);
        break;
      
      default:
        dht11_error();
        break;
    }
  }
}
void readCHIPCAP(void){
  float Temp_float;
  float Humidity_float;

  Serial.print("Temperature: ");
  if(Flag_temperature){
    Flag_temperature = 0;
    Temp_float = ((float)Temperature / 1023 * 200) - 50;
    Serial.println(Temp_float);
  }
  Serial.print("Humidity: ");
  if(Flag_humidity){
    Flag_humidity = 0;
    Humidity_float = ((float)Humidity / 255*100);
    Serial.print(Humidity_float);
  }
  if((Flag_dht11_error) && !Time_error_wait){
    Flag_dht11_error = 0;
    init_dht11();
  }
}

void setup(){
  sei();
  init_dht11();
  Serial.begin(9600);
}

void loop(){
  readCHIPCAP();
}