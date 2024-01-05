#define PI 3.14159265358979







//타입 정의
typedef uint16_t element;
typedef unsigned char bits;

//ADC 관련
void adc_init(void);
element adc_read(element adc_pin);
void send_data();
void seperate_data();

//시리얼 통신 관련
void USART_init();
void TX0_char(element Data);
int start; // 초 셀 때
element avg; // 값의 평균 구하기
element sum;

void setup() {
  adc_init();
  USART_init();
  sei();
}

void loop() {
  float value = adc_read(1);
  // 0~500의 값에서 0~255로 스케일링
  unsigned char scaledValue = (unsigned int)(((float)value / 511.0f) * 255.0f);
  TX0_char(scaledValue);
}
void USART_init(void){
  UCSR0A = (1 << UDRE0);
  UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = 0x06;
  UBRR0 = 7;
}
void TX0_char(element Data){
  
  while(!(UCSR0A & (1<<UDRE0))){};
  UDR0 = Data; //UDR0 I/O 레지스터에 값 보냄
}
void adc_init(void){
  ADMUX = (1 << REFS0); //set voltage reference
  ADMUX &= ~(1 << ADLAR); //right-side adjust
  ADCSRA = (1 << ADEN) | (1 << ADPS2); //prescale 64
}
element adc_read(element adc_pin){
  ADMUX |= (adc_pin & 0x0F);
  ADCSRA |= (1 << ADSC);
  while(!(ADCSRA & (1<< ADSC))); //변환 완료시까지 대기
  return ADC;
}







