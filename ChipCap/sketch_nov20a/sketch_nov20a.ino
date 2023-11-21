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
volatile unsigned char Time_error_wait = 5;
volatile unsigned char Humidity = 0;
volatile unsigned int Temperature = 0;
volatile unsigned int I_buffer1 = 0;
volatile unsigned int I_buffer2 = 0;
volatile unsigned char Flag_edge_int = 0;

void interrupt_edge_set(unsigned char ch, unsigned char Flag_edge){
  unsigned char edge;
  TIMSK0 &= ~(1 << TOIE0);
  if(Flag_edge == FALLING_EDGE){
    edge = 0x02; //ISC01, ISC00 -> 1 0 -> Falling edge
    Flag_edge_int = FALLING_EDGE;
  }
  else{
    edge = 0x03; //ISC01, ISC00 -> 0 1 -> Rising edge
    Flag_edge_int = RISING_EDGE;
    
  }
  if (ch == CH_TEMPERATURE) edge <<= 2; //INT1 edge 0x0000xx00
  EICRA = edge;
  TCNT2 = 0;
  TIFR2;
  TIMSK2 |= (1 << TOIE2);
}

void ext_interrupt_set(unsigned char ch){
  
  Flag_data_start = 0;
  if(ch == CH_TEMPERATURE){
    dht11_job = 3;
    interrupt_edge_set(CH_TEMPERATURE, FALLING_EDGE);
    Sig_channel = CH_TEMPERATURE;
    EIMSK |= (1 << INT1) & ~(1 << INT0);
  } else if (ch == CH_HUMIDITY){
    dht11_job = 0;
    interrupt_edge_set(CH_HUMIDITY, FALLING_EDGE);
    Sig_channel = CH_HUMIDITY;
    while(((PIND & 0x04)>>2));
    EIMSK |= (1 << INT0) & ~(1 << INT1);
  }
  
}

void sensor_init(){
  DDRD &= ~(0x04);
  PIND &= ~(0x04);
  _delay_ms(18);
  while(!(PIND & 0x04)>>2);
}

void init_dht11(void){

  unsigned char ch;

  //while(!(((PIND & 0x08) >> 3) || ((PIND & 0x04) >> 2))); //3번이나 4번핀 값 들어올 때 까지 대기
  //if((PIND & 0x08) >> 3) ch = CH_TEMPERATURE;
  //if((PIND & 0x04) >> 2) ch = CH_HUMIDITY;


  ext_interrupt_set(CH_HUMIDITY); //플래그 설정해서 외부 인터럽트를 그때마다 바꿔서 쓴다.
  TIMSK2 |= 1 << TOIE2;
  OCR2A = 0;
  TIFR2;
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


ISR(INT0_vect){
  Serial.println("인터럽트 발생");
  unsigned char bit_ratio = 0;
  unsigned int I_value = 0, sum = 0;

  if(Flag_edge_int == RISING_EDGE){
    Low_time = TCNT2; //업카운터 레지스터, FF에서 00될 때 인터럽트 됨
    interrupt_edge_set(CH_HUMIDITY, FALLING_EDGE);
    if(bit_counter == 8) return;
  }
  else {
    High_time = TCNT2;
    interrupt_edge_set(CH_HUMIDITY, RISING_EDGE);
    if(bit_counter == 8) Time_sum = (High_time + Low_time) << 4;

    bit_ratio = Time_sum/Low_time;
  }
  switch(dht11_job) 
    {
      /*---------------------------------------------------------------------------------*/
      /*                                      습도 부분                                    */
      /*---------------------------------------------------------------------------------*/
      case 0:
        Serial.println("case 0 접근");
        if((bit_ratio>16) && (bit_ratio < 26)) bit_ratio = 1;
        if((bit_ratio > 27) && (bit_ratio < 48)) {Flag_data_start = 1; bit_counter= 0; Humidity |= 0x0001;}
        else dht11_error();
        return;
    
      case 1:
        Serial.println("case 1 접근");
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
        ext_interrupt_set(CH_HUMIDITY);
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

ISR(TIMER2_OVF_vect){
  
  if(Time_error_wait) Time_error_wait --;
  if(Flag_data_start) dht11_error();
  
}


void readCHIPCAP(void){
  float Temp_float;
  float Humidity_float;
  sei();
  //Serial.print("Temperature: ");
  if(Flag_temperature){
    Flag_temperature = 0;
    Temp_float = ((float)Temperature / 1023 * 200) - 50;
    Serial.println(Temp_float);
  }
  //Serial.print("Humidity: ");
  if(Flag_humidity){
    Flag_humidity = 0;
    Humidity_float = ((float)Humidity / 255*100);
    Serial.print(Humidity_float);
  }
  if((Flag_dht11_error) && !Time_error_wait){ //다중 인터럽트 구현법 모르므로 일단 대기 -> 타임아웃 못씀
    Flag_dht11_error = 0;
    Time_error_wait = 5;
    init_dht11();
    
  }
  cli();
}

void setup(){
  //DDRD |= (0x04);
  //PORTD |= (0x04);
  Serial.begin(9600);
  init_dht11();
  
}

void loop(){
   
  // while(((PIND & 0x04)>>2)) {
    
  // }; 

  readCHIPCAP();
}