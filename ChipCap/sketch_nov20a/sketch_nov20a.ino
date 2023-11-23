#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CH_HUMIDITY 0
#define CH_TEMPERATURE 1 
#define FALLING_EDGE 0
#define RISING_EDGE 1
#define AT24CxxPORT PORTC
#define AT24CxxDDR DDRC
#define AT24CxxPIN PINC
#define AT24Cxx_SCL 0b00010000
#define AT24Cxx_SDA 0b00100000
#define AT24CxxPORT PORTD
#define AT24CxxDDR DDRD
#define AT24CxxPIN PIND
#define AT24Cxx_SDA_HIGH() (AT24CxxPORT |= AT24Cxx_SDA)
#define AT24Cxx_SDA_LOW() (AT24CxxPORT &= ~AT24Cxx_SDA)
#define AT24Cxx_SCL_HIGH() (AT24CxxPORT |= AT24Cxx_SCL)
#define AT24Cxx_SCL_LOW() (AT24CxxPORT &= ~(AT24Cxx_SCL))
#define AT24Cxx_SDA_IN() (AT24CxxDDR &= ~AT24Cxx_SDA)
#define AT24Cxx_SDA_OUT() (AT24CxxDDR |= AT24Cxx_SDA)
#define wait_for_completion while(!(TWCR & (1 << TWINT)));
#define E_BIT 0x04 //location of E BIT
#define B_BIT 0x08 //location of bright
#define I2CLCD_ADDR 0x27
#define SLAVE2_ADDR 0xC0
#define AT24Cxx_DELAY 5
#define ACK 0x00 // ACK LOW
#define NACK 0x01 // NACK HIGH
#define EEP_WRITE_WAIT 10 // write wait time

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
volatile unsigned long highDuty[40];
volatile unsigned char recData[5];
volatile unsigned char ASCII_Table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
volatile unsigned char i2c_status = TWSR & 0b11111100;
volatile unsigned char Wait_time = 0;
volatile unsigned char digit[3];
volatile unsigned char  sub_addr = 0x80;
volatile unsigned char  save_data = 0, read_data = 0;
volatile unsigned char  temp_1, temp_2;
volatile unsigned char  ADC_buff;
volatile unsigned char  first_timer = 0;
volatile unsigned char  flag = 0;
volatile unsigned char  E_flag = 0;
void AT24Cxx_i2c_init(void);
void AT24Cxx_i2c_start(void);
void AT24Cxx_ACK_send(unsigned char ack_data);
void AT24Cxx_i2c_stop(void);
void write_AT24Cxx_i2c_byte(unsigned char byte);
void write_AT24Cxx_i2cControl(unsigned char byte);
unsigned char read_AT24Cxx_i2c_byte(void);
unsigned char AT24Cxx_Read(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2);
void LCD_Write(unsigned char address, unsigned char data);
void send_EFalling_edge(unsigned char byte);
void write_AT24Cxx_i2c_LCDPrint(unsigned char line, unsigned char location, unsigned char data);
void write_AT24Cxx_i2c_LCDAddressing(unsigned char byte);
void I2C_LCDSendControl();
void I2C_LCDSendChar(unsigned char c_data);
void I2C_LCD_init(unsigned char data);
void Port_init();
void Timer_init();
void ADC_init();
void interrupt_edge_set(unsigned char ch, unsigned char Flag_edge);
void ext_interrupt_set(unsigned char ch);
void init_dht11(void);
void dht11_error(void);
unsigned char EvenParityCheck(unsigned int temp_value);
void readCHIPCAP(void);
void I2CPRINTLCD(void);

void setup(){
  init_dht11();
}

void loop(){
  readCHIPCAP();
  I2CPRINTLCD();
}

void I2CPRINTLCD(void){
  AT24Cxx_i2c_start();
	write_AT24Cxx_i2c_LCDAddressing((I2CLCD_ADDR << 1)); //to write
  LCD_Write(I2CLCD_ADDR, 0x20);
  write_AT24Cxx_i2c_LCDAddressing(0x20);
  LCD_Write(I2CLCD_ADDR, 0x20);
  write_AT24Cxx_i2c_LCDAddressing(0x20);
  LCD_Write(I2CLCD_ADDR, 0x00);
write_AT24Cxx_i2c_LCDAddressing(0x00);
  LCD_Write(I2CLCD_ADDR, 0x00);
  write_AT24Cxx_i2c_LCDAddressing(0x00);
  LCD_Write(I2CLCD_ADDR, 0xE0);
  write_AT24Cxx_i2c_LCDAddressing(0xE0);
  LCD_Write(I2CLCD_ADDR, 0x00);
  write_AT24Cxx_i2c_LCDAddressing(0x00);
  LCD_Write(I2CLCD_ADDR, 0x60);
write_AT24Cxx_i2c_LCDAddressing(0x60);


  //글자 영역
  LCD_Write(I2CLCD_ADDR, 0x31);
  write_AT24Cxx_i2c_LCDAddressing(0x31);
  LCD_Write(I2CLCD_ADDR, 0x01);
write_AT24Cxx_i2c_LCDAddressing(0x01);

LCD_Write(I2CLCD_ADDR, 0x31);
  write_AT24Cxx_i2c_LCDAddressing(0x31);
  LCD_Write(I2CLCD_ADDR, 0x11);
write_AT24Cxx_i2c_LCDAddressing(0x11);

LCD_Write(I2CLCD_ADDR, 0x31);
  write_AT24Cxx_i2c_LCDAddressing(0x31);
  LCD_Write(I2CLCD_ADDR, 0x21);
write_AT24Cxx_i2c_LCDAddressing(0x21);

LCD_Write(I2CLCD_ADDR, 0x31);
  write_AT24Cxx_i2c_LCDAddressing(0x31);
  LCD_Write(I2CLCD_ADDR, 0x31);
write_AT24Cxx_i2c_LCDAddressing(0x31);
AT24Cxx_i2c_stop();
}

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
    
  }
  
}

void init_dht11(void){

  unsigned char ch;
  EIMSK = 0;
  DDRD |= 0x08;
  PORTD |= 0x08;
  _delay_ms(1000);
  //ext_interrupt_set(CH_HUMIDITY); //플래그 설정해서 외부 인터럽트를 그때마다 바꿔서 쓴다.
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
  unsigned char bit_ratio = 0;
  unsigned int I_value = 0, sum = 0;

  if(Flag_edge_int == RISING_EDGE){
    Low_time = TCNT2; //업카운터 레지스터, FF에서 00될 때 인터럽트 됨
    interrupt_edge_set(CH_HUMIDITY, FALLING_EDGE);
    if(bit_counter != 8) return;
  }
  else {
    High_time = TCNT2;
    interrupt_edge_set(CH_HUMIDITY, RISING_EDGE);
    if(bit_counter != 8) Time_sum = (High_time + Low_time) << 4;

    bit_ratio = Time_sum/Low_time;
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
  PORTD &= ~(0x08);
  _delay_ms(20);
  PORTD |= 0x08;
  _delay_ms(40);
  DDRD &= ~(0X08);
  EIMSK |= (1 << INT0) & ~(1 << INT1);
  while(((PIND & 0x08)>>2) == 0);

  // Data Analysis
  for(int i = 0 ; i < 5 ; i++){
    for(int j = 0 ; j < 8 ; j++){
      int temp = (8 * i) + j;
      if(highDuty[temp] > 50){
        recData[i] |= 1 << 7-j;                 
      }
    }
  }



  if((Flag_dht11_error) && !Time_error_wait){ 
    Flag_dht11_error = 0;
    Time_error_wait = 5;
    init_dht11();
    
  }
  cli();
}


void Port_init(void){
	PORTC = 0xFF; // PD1 = 1
	DDRC = 0xFF; 
	DDRB = 0x00;
}

void Timer_init(void){
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) |
	 (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
	TCCR1C = 0;
	OCR1A = 0x09C4;
	TIMSK0 = (1 << OCIE1A); 
}

void ADC_init(void){
	ADMUX = 0x60;
	ADCSRA = 0x87;
}

unsigned char ADC_read(unsigned char Ch){
	Ch &= 0x01;
	
	ADMUX |= (Ch & 0b00011111);
	ADCSRA |= 0x40;
}


void Hex2Dec(unsigned char data){
	digit[2] = data % 100;
	data = data - digit[2]*100;
	digit[1] = data%10;
	digit[0] = data - digit[1]*10;
}

unsigned char Digit2ASCII(unsigned char data){
	data &= 0x0F;
	if (data < 10) return (data + '0');
	return (data + 'A');
}

void USART0_init(void){
	UCSR0A = 0x20;
	UCSR0B = 0x18;
	UCSR0C = 0x06;
	UBRR0H = 0;
	UBRR0L = 103;
}

void AT24Cxx_i2c_init(void){
	
	AT24CxxPORT |= (AT24Cxx_SDA + AT24Cxx_SCL);
	AT24CxxDDR |= (AT24Cxx_SDA + AT24Cxx_SCL);
	_delay_us(AT24Cxx_DELAY);
	AT24Cxx_SCL_LOW();
	_delay_us(AT24Cxx_DELAY);
}

void AT24Cxx_i2c_start(void) { //start condition
	
	AT24Cxx_SDA_LOW(); //clear SDA
	_delay_us(1);
	AT24Cxx_SCL_LOW();
	_delay_us(1);
}


void AT24Cxx_ACK_send(unsigned char ack_data){
	if(ack_data) AT24Cxx_SDA_HIGH(); //NACK
	else AT24Cxx_SDA_LOW(); //ACK
	AT24Cxx_SDA_OUT(); //set SDA to Output
	AT24Cxx_SCL_HIGH(); //9th clk rising edge
	_delay_us(1);
	AT24Cxx_SCL_LOW();
	_delay_us(1);
}

void AT24Cxx_i2c_stop(void){
	//AT24Cxx_SDA_OUT(); //set SDA to output
	AT24Cxx_SDA_LOW(); //clear SDA
	
	_delay_us(1);
	
	AT24Cxx_SCL_HIGH(); // set SCL High
	_delay_us(1);
	AT24Cxx_SDA_HIGH(); // set SDA High
}

void write_AT24Cxx_i2c_LCDAddressing(unsigned char byte){//SDA의 바이트를 읽어노는 과정
	_delay_us(1);
 
	for(int i = 0; i <8; i++){
    
		if(byte & 0x80) AT24Cxx_SDA_HIGH(); //MSB
		else AT24Cxx_SDA_LOW(); //Clear
		
		AT24Cxx_SCL_HIGH(); //clock data
		byte = byte << 1; 
		_delay_us(1);
		AT24Cxx_SCL_LOW();
		_delay_us(1);
		 
	}
	AT24Cxx_SDA_LOW();
	_delay_us(1);
	
	AT24Cxx_SCL_HIGH();
  _delay_us(1);
	AT24Cxx_SCL_LOW();
}

void send_EFalling_edge(unsigned char byte){
  write_AT24Cxx_i2c_LCDAddressing(byte);
}

void write_AT24Cxx_i2c_byte(unsigned char byte){//SDA의 바이트를 읽어노는 과정 상승에지 하강에지 모두 구현
	_delay_us(1);
  if(!(byte & E_BIT)) byte |= E_BIT;
  if(!(byte & B_BIT)) byte |= B_BIT;
	write_AT24Cxx_i2c_LCDAddressing(byte);

}

void write_AT24Cxx_i2cControl(unsigned char byte){//control
  AT24Cxx_i2c_start();
  write_AT24Cxx_i2c_LCDAddressing((I2CLCD_ADDR << 1)); 
	write_AT24Cxx_i2c_byte(byte);
  AT24Cxx_i2c_stop();
  _delay_us(AT24Cxx_DELAY);
}


void LCD_Write(unsigned char address, unsigned char data){
  // D7 D6 D5 D4   X  E  RW RS
  // P7 P6 P5 P4   P3 P2 P1 P0
  _delay_us(1);
  write_AT24Cxx_i2c_byte(data); //function set

}