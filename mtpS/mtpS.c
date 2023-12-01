#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
//====================================================
#define Btn1 2
#define RED 0
#define BLUE 1
#define GREEN 2
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//====================================================
#define AT24CxxPORT PORTB
#define AT24CxxDDR DDRB
#define AT24CxxPIN PINB
#define AT24Cxx_SCL 0b00100000
#define AT24Cxx_SDA 0b00010000
#define E_BIT 0x04 //location of E BIT
#define B_BIT 0x08 //location of bright
#define I2CLCD_ADDR 0x27
#define AT24Cxx_DELAY 1
#define ACK 0x00 // ACK LOW
#define NACK 0x01 // NACK HIGH
#define EEP_WRITE_WAIT 10 // write wait time
#define AT24Cxx_SDA_HIGH() (AT24CxxPORT |= AT24Cxx_SDA)
#define AT24Cxx_SDA_LOW() (AT24CxxPORT &= ~AT24Cxx_SDA)
#define AT24Cxx_SCL_HIGH() (AT24CxxPORT |= AT24Cxx_SCL)
#define AT24Cxx_SCL_LOW() (AT24CxxPORT &= ~(AT24Cxx_SCL))
#define AT24Cxx_SDA_IN() (AT24CxxDDR &= ~AT24Cxx_SDA)
#define AT24Cxx_SDA_OUT() (AT24CxxDDR |= AT24Cxx_SDA)
#define wait_for_completion while(!(TWCR & (1 << TWINT)));
//====================================================
volatile unsigned char job = 0;
volatile unsigned char lcd_flag = 0;
volatile unsigned char addr = 0;
volatile unsigned char key_flag = 0;
volatile unsigned char new_key = 0;
volatile unsigned char key_data = 0;
volatile unsigned char pressing_flag = 0;
volatile unsigned char keyNum = 0;
volatile unsigned char current_key = 0;
volatile unsigned char old_key;
volatile unsigned char flag_time = 0;
volatile unsigned char Wait_time = 0;
volatile unsigned char digit[3];
volatile unsigned char ASCII_Table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
//====================================================
void Timer0_init();
void Timer1_init();
void Port_init();
void USART_init();
void setup_watchdog();
//====================================================
unsigned char key_scan(void);
unsigned char key_analysis(unsigned char key, unsigned char Raw);
unsigned char read_AT24Cxx_i2c_byte(void);
unsigned char AT24Cxx_Read(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2);
void LCD_clear(void);
void LED_ON(void);
void LED_OFF(void);
void LCD_CharWrite(uint16_t byte);
void Tx_char(unsigned char data);
void AT24Cxx_i2c_init(void);
void AT24Cxx_i2c_start(void);
void AT24Cxx_ACK_send(unsigned char ack_data);
void AT24Cxx_i2c_stop(void);
void write_AT24Cxx_i2c_byte(unsigned char byte);
void write_AT24Cxx_i2cControl(unsigned char byte);
void LCD_Write(unsigned char byte);
void send_EFalling_edge(unsigned char byte);
void write_AT24Cxx_i2c_LCDPrint(unsigned char line, unsigned char location, unsigned char data);
void write_AT24Cxx_i2c_LCDAddressing(unsigned char byte);
void LCD_NEBIT(unsigned char byte);
void I2C_LCD_init(unsigned char data);
uint16_t num2lcdbyte(unsigned char num);
//====================================================
int main(void){
	
    //setup_watchdog();
	sei(); //must be first position
    Port_init();
    USART_init();
	//Timer0_init();
	Timer1_init();
    

	//EICRA = 0x03;
	//EIMSK = (1 << INT0);

	while(1){
		
    }
}
//====================================================



// void setup_watchdog() {
//   // WDTCSR: Watchdog Timer Control Register
//   // WDCE (Watchdog Change Enable)
//   // WDTO_1S: Watchdog Timeout 
//   WDTCSR |= (1<<WDE) | (1<<WDCE) | (1<<WDIE) | (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); 
// }  
//===================================================


void Timer1_init(){

    TCCR1B =  (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
	TCNT1 = 0;
    OCR1A = 0x09c4; 
    TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);
}

// void Timer0_init(){

//     TCCR0B =  (1 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);
// 	TCNT0 = 0;
//     OCR0A = 0x09c4; 
//     TIMSK0 |= (1 << TOIE0) | (1 << OCIE0A);
// }

void Port_init(){

    //set B0~D4 as input,  D5, D6, D7 pins are set high to find key vals 
    DDRD = 0b01110000;
    PORTD = 0b00000000;
    DDRB = 0b00110000;


}
void I2C_LCD_init(unsigned char address){
	cli();

	PORTC ^= 0xff;
	_delay_ms(18);
	AT24Cxx_i2c_start();
	write_AT24Cxx_i2c_LCDAddressing(I2CLCD_ADDR  << 1); //write mode
	LCD_Write(0x20);
	_delay_ms(5);
	write_AT24Cxx_i2c_LCDAddressing(0x20);
	_delay_ms(5);

	write_AT24Cxx_i2c_LCDAddressing(0x30);
	LCD_Write(0x00);
	_delay_ms(5);

	LCD_Write(0x00);
	write_AT24Cxx_i2c_LCDAddressing(0x00);
    _delay_ms(5);

	LCD_Write(0xE0);
	write_AT24Cxx_i2c_LCDAddressing(0xE0);
	_delay_ms(5);

	LCD_Write(0x00);
	write_AT24Cxx_i2c_LCDAddressing(0x00);
	_delay_ms(5);

	LCD_Write(0x60);
	write_AT24Cxx_i2c_LCDAddressing(0x60);
	_delay_ms(5);
	AT24Cxx_i2c_stop();
	PORTC ^= 0xff;
	sei();
	
	
	
}
//====================================================
void LED_ON(){
    DDRC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);
    PORTC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);
}

void LED_OFF(){
    DDRC &= ~(1 << RED) & ~(1 << GREEN) & ~(1 << BLUE); //refresh
    DDRC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);
    PORTC &= ~(1 << RED) & ~(1 << GREEN) & ~(1 << BLUE);
}
//====================================================
ISR(TIMER1_COMPA_vect){

   //**NOTE: Timer must perform only the role of timer otherwise, global variables will be reset **
    key_data = key_scan();


    
    if(key_flag){
		
        key_flag = 0;

        if(key_data == 0xff){
            return ;
		}

        if(key_data == 10){
            //key_data += 32; // 42 -> *
			I2C_LCD_init(0x27);
        }
        if(key_data == 11){
            //key_data += 24; // 35 -> #
			LCD_CharWrite(0x10);
			_delay_us(38);
			LCD_CharWrite(0x30);
			_delay_ms(5);

			I2C_LCD_init(0x27);
			_delay_ms(5);

			LCD_CharWrite(0x10);
			_delay_us(38);
			LCD_CharWrite(0x30);
			_delay_ms(10);

			I2C_LCD_init(0x27);
			_delay_ms(10);
			I2C_LCD_init(0x27);


			// _delay_us(38);
			// I2C_LCD_init(0x27);

		}                                                       
		if(key_data == 1){
			LCD_CharWrite(num2lcdbyte(1));
			//key_data += '0';
		}
		if(key_data == 2){
			LCD_CharWrite(num2lcdbyte(2));
			//key_data += '0';
		}
		if(key_data == 3){
			LCD_CharWrite(num2lcdbyte(3));
			//key_data += '0';
		}
		if(key_data == 4){
			LCD_CharWrite(num2lcdbyte(4));
			//key_data += '0';
		}
		if(key_data == 5){
			
			LCD_CharWrite(num2lcdbyte(5));
			//key_data += '0';
		}
		if(key_data == 6){
			
			LCD_CharWrite(num2lcdbyte(6));
			//key_data += '0';
		}
		if(key_data == 7){
			
			LCD_CharWrite(num2lcdbyte(7));
			//key_data += '0';
		}
		if(key_data == 8){
			
			LCD_CharWrite(num2lcdbyte(8));
			//key_data += '0';
		}
		if(key_data == 9){
			
			LCD_CharWrite(num2lcdbyte(9));
			//key_data += '0';
		}
		if(key_data == 0){
			
			LCD_CharWrite(num2lcdbyte(0));
			//key_data += '0';
		}
		LCD_CharWrite(0xF0);

		PORTC |= 0b00100000;
        //Tx_char(key_data);
        //Tx_char(' ');
        //LED_ON();
        }
        else{
            //LED_OFF();
			PORTC &= ~0b00100000;
  			PORTC &= ~0x01;
        }
	
}

ISR(TIMER0_COMPA_vect){ //Timer
   //**NOTE: Timer must perform only the role of timer otherwise, global variables will be reset **
	Wait_time =1;

}

// ISR(INT0_vect){
// cli();
// 	TIMSK0 &= ~(1 << TOIE0) & ~(1 << OCIE0A);
	
//    //AT24Cxx_i2c_start();
//    DDRC &= ~0x01;
//    DDRC |= 0x01;
//    PORTC |= 0x01;
//    sei();
//    TIMSK0 |= (1 << TOIE0) | (1 << OCIE0A); 
//    	//
//    LCD_CharWrite(num2lcdbyte(1));
// }
//====================================================
void USART_init(){
    UCSR0A = 0x20;
    UCSR0B = 0x18;
    UCSR0C = 0x06;
    UBRR0 = 8;
}



void Tx_char(unsigned char data){
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}
//====================================================
unsigned char key_scan(){


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
            for(int Raw = 0; Raw < 3; Raw++){
                PORTD |= 0x70; 
                PORTD &= ~(bit_shift); //to check specific raw, Ex) raw 1 -> D5, D6 must be HIGH state
									   //in addition if pin of PORTD is on, PINB won't be low state
			    
                
                DDRD &= ~(bit_shift); //refresh
				DDRD |= 0x70; 
				
                new_key = (PINB) & 0x0f;
                if(new_key == 0x0f){ // + when raw is LOW
                  
                    bit_shift <<= 1;
                    continue;
                }
                else {
					if(!lcd_flag) I2C_LCD_init(0x27);
					lcd_flag = 1;
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

	key_flag = 1;
	switch(Raw){
		case 0:
		switch(key){
			case 0b00001110:
			return 1;
			case 0b00001101:
			return 4;
			case 0b00001011:
			return 7;
			case 0b00000111:
			return 10;
			default:
			key_flag = 0;
			return -1;
		}

		case 1:
		switch(key){
			case 0b00001110:
			return 2;
			case 0b00001101:
			return 5;
			case 0b00001011:
			return 8;
			case 0b00000111:
			return 0;
			default:
			key_flag = 0;
			return -1;
		}

		case 2:
		switch(key){
			case 0b00001110:
			return 3;
			case 0b0001101:
			return 6;
			case 0b00001011:
			return 9;
			case 0b00000111:
			return 11;
			default:
			key_flag = 0;
			return -1;
		}
		default:
		key_flag = 0;
		return -1;
	}
}
//====================================================

void LCD_Write(unsigned char data){ //mainly command
	
	// D7 D6 D5 D4   X  E  RW RS
	// P7 P6 P5 P4   P3 P2 P1 P0
	_delay_us(1);
	//LCD_NEBIT(data);
	write_AT24Cxx_i2c_byte(data); 

}

void LCD_CharWrite(uint16_t byte){
	
	cli();

	_delay_us(1);
	AT24Cxx_i2c_start();
	write_AT24Cxx_i2c_LCDAddressing((I2CLCD_ADDR << 1)); 
	unsigned char upperbits = byte>>8 & 0xFF;
	unsigned char lowerbits = byte & 0xFF;
	LCD_NEBIT(upperbits);
	write_AT24Cxx_i2c_byte(upperbits); 
	LCD_NEBIT(lowerbits);
	write_AT24Cxx_i2c_byte(lowerbits); 
	AT24Cxx_i2c_stop();
	sei();
}

uint16_t num2lcdbyte(unsigned char num){
	// D7 D6 D5 D4   X  E  RW RS
	// P7 P6 P5 P4   P3 P2 P1 P0

	if(num == 0) return 0x3101;
	if(num == 1) return 0x3111;
	if(num == 2) return 0x3121;
	if(num == 3) return 0x3131;
	if(num == 4) return 0x3141;
	if(num == 5) return 0x3151;
	if(num == 6) return 0x3161;
	if(num == 7) return 0x3171;
	if(num == 8) return 0x3181;
	if(num == 9) return 0x3191;

	return -1;

}
//====================================================
void write_AT24Cxx_i2c_LCDAddressing(unsigned char byte){

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
void AT24Cxx_i2c_start(void) { //start condition

	AT24Cxx_SDA_LOW(); //clear SDA
	_delay_us(1);
	AT24Cxx_SCL_LOW();
	_delay_us(1);
}

void AT24Cxx_ACK_send(unsigned char ack_data){
	_delay_us(1);
	if(ack_data) AT24Cxx_SDA_HIGH(); //NACK
	else AT24Cxx_SDA_LOW(); //ACK
	_delay_us(1);
	AT24Cxx_SDA_OUT(); //set SDA to Output
	_delay_us(1);
	AT24Cxx_SCL_HIGH(); //9th clk rising edge
	_delay_us(1);
	AT24Cxx_SCL_LOW();
}

void AT24Cxx_i2c_stop(void){
	//AT24Cxx_SDA_OUT(); //set SDA to output
	AT24Cxx_SDA_LOW(); //clear SDA
	
	_delay_us(1);
	AT24Cxx_SCL_HIGH(); // set SCL High
	_delay_us(1);
	AT24Cxx_SDA_HIGH(); // set SDA High

}

void write_AT24Cxx_i2c_byte(unsigned char byte){
	_delay_us(1);
	if(!(byte & E_BIT)) byte |= E_BIT;
	if(!(byte & B_BIT)) byte |= B_BIT;
	write_AT24Cxx_i2c_LCDAddressing(byte);

}
void LCD_NEBIT(unsigned char byte){
	write_AT24Cxx_i2c_LCDAddressing(byte);
}

//====================================================