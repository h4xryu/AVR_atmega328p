#include <LiquidCrystal_I2C.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CLCD_RS 0b00100000
#define CLCD_RW 0b01000000
#define CLCD_EBIT 0b01000000
#define AT24CxxPORT PORTC
#define AT24CxxDDR DDRC
#define AT24CxxPIN PINC
#define AT24Cxx_SCL 0b00010000
#define AT24Cxx_SDA 0b00100000

#define E_BIT 0x04 //location of E BIT
#define B_BIT 0x08 //location of bright
#define I2CLCD_ADDR 0x27
#define SLAVE2_ADDR 0xC0
#define AT24Cxx_DELAY 5
#define ACK 0x00 // ACK LOW
#define NACK 0x01 // NACK HIGH
#define EEP_WRITE_WAIT 10 // write wait time

#define AT24Cxx_SDA_HIGH() (AT24CxxPORT |= AT24Cxx_SDA)
#define AT24Cxx_SDA_LOW() (AT24CxxPORT &= ~AT24Cxx_SDA)
#define AT24Cxx_SCL_HIGH() (AT24CxxPORT |= AT24Cxx_SCL)
#define AT24Cxx_SCL_LOW() (AT24CxxPORT &= ~(AT24Cxx_SCL))
#define AT24Cxx_SDA_IN() (AT24CxxDDR &= ~AT24Cxx_SDA)
#define AT24Cxx_SDA_OUT() (AT24CxxDDR |= AT24Cxx_SDA)

unsigned char ASCII_Table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t i2c_status = TWSR & 0b11111100;
#define wait_for_completion while(!(TWCR & (1 << TWINT)));

void USART0_init(void);
void TX0_char(unsigned char Data);
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

volatile unsigned char Wait_time = 0;
volatile unsigned char digit[3];
unsigned int sub_addr = 0x80;
unsigned char save_data = 0, read_data = 0;
unsigned char temp_1, temp_2;
unsigned char ADC_buff;
int first_timer = 0;
int flag = 0;
int E_flag = 0;
void setup(){
  
	Port_init();
	Timer_init();
	ADC_init();
	USART0_init();
  
  
}
 void loop(){

  // D7 D6 D5 D4   X  E  RW RS
  // P7 P6 P5 P4   P3 P2 P1 P0
	


  
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

void TX0_char(unsigned char Data){
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = Data;
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

void write_AT24Cxx_i2c_LCDPrint(unsigned char line, unsigned char location, unsigned char data){//SDA의 바이트를 읽어노는 과정
  location &= 0x0F; //limit num of char
  if(line == 1) location = 0x80 + location; // first line
  else if(line == 2) location = 0xC0 + location; //second line
  else return;
  
  //_delay_ms(30);
  write_AT24Cxx_i2c_byte(0x21);
  write_AT24Cxx_i2c_byte(0x00);
  write_AT24Cxx_i2c_byte(0x22);
  write_AT24Cxx_i2c_byte(0x00);
  write_AT24Cxx_i2c_byte(0x27);
  write_AT24Cxx_i2c_byte(0x00);
  write_AT24Cxx_i2c_byte(0x2F);
  write_AT24Cxx_i2c_byte(0x00);
  write_AT24Cxx_i2c_byte(0x8F);
  write_AT24Cxx_i2c_byte(0x00);
  write_AT24Cxx_i2c_byte(data);
}


void LCD_Write(unsigned char address, unsigned char data){
  
  // D7 D6 D5 D4   X  E  RW RS
  // P7 P6 P5 P4   P3 P2 P1 P0
  _delay_us(1);
  write_AT24Cxx_i2c_byte(data); //function set
  
	//AT24Cxx_i2c_stop();
}

void I2C_LCD_init(unsigned char address){
  
}


void I2C_LCDSendChar(unsigned char c_data);

void Hex2Dec(unsigned char data);
unsigned char Digit2ASCII(unsigned char data);

