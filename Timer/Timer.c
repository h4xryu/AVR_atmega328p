#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
//====================================================
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
#define ACK 0x00 // ACK LOW
#define NACK 0x01 // NACK HIGH
#define EEP_WRITE_WAIT 10 // write wait time
#define AT24Cxx_SDA_HIGH() (AT24CxxPORT |= AT24Cxx_SDA)
#define AT24Cxx_SDA_LOW() (AT24CxxPORT &= ~AT24Cxx_SDA)
#define AT24Cxx_SCL_HIGH() (AT24CxxPORT |= AT24Cxx_SCL)
#define AT24Cxx_SCL_LOW() (AT24CxxPORT &= ~(AT24Cxx_SCL))
#define AT24Cxx_SDA_IN() (AT24CxxDDR &= ~AT24Cxx_SDA)
#define AT24Cxx_SDA_OUT() (AT24CxxDDR |= AT24Cxx_SDA)
//====================================================
volatile unsigned char job = 0; //Timer job flag
volatile unsigned char sel_flag = 0; //Time selector
volatile unsigned char lcd_flag = 0; //lcd on check flag
volatile unsigned char end_flag = 0; //timer end flag
volatile unsigned char addr = 0; 
volatile unsigned char key_flag = 0; //key check flag
volatile unsigned char new_key = 0;
volatile unsigned char key_data = 0;
volatile unsigned char pressing_flag = 0;
volatile unsigned char keyNum = 0;
volatile unsigned char current_key = 0;
volatile unsigned char old_key;
volatile unsigned char flag_time = 0;
volatile unsigned char Wait_time = 0;
volatile unsigned char timer_start_flag = 0;
volatile uint16_t time_1s = 0; // 100 : 1s
volatile uint16_t time_10ms = 0; 
volatile uint16_t time_tmp = 0;
volatile unsigned char min = 0;
volatile unsigned char sec = 0;
volatile unsigned char test = 1;
//====================================================
void Timer0_init();
void Timer1_init();
void Port_init();
void USART_init();
void setup_watchdog();
//====================================================
void Tx_char(unsigned char data);
unsigned char key_scan(void);
unsigned char key_analysis(unsigned char key, unsigned char Raw);
unsigned char read_AT24Cxx_i2c_byte(void);
unsigned char AT24Cxx_Read(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2);
void AT24Cxx_i2c_init(void);
void AT24Cxx_i2c_start(void);
void AT24Cxx_ACK_send(unsigned char ack_data);
void AT24Cxx_i2c_stop(void);
void write_AT24Cxx_i2c_byte(unsigned char byte);
void write_AT24Cxx_i2cControl(unsigned char byte);
void LCD_Write(unsigned char byte);
void write_AT24Cxx_i2c_LCDAddressing(unsigned char byte);
void LCD_NEBIT(unsigned char byte);
void I2C_LCD_init(unsigned char data);
void LCD_clear(void);
void LCD_CharWrite(uint16_t byte);
void I2C_LCD_Reset();
void I2C_LCD_Erase();
void I2C_LCD_MoveLeft(unsigned char cnt);
void I2C_LCD_MoveRight(unsigned char cnt);
void I2C_LCD_SELECT();
void I2C_Timer_Default();
void I2C_Timer_act();
void I2C_Timer_act_ms();
void I2C_Timer_test();
void TIME_SETTING(unsigned char m, unsigned char s);
uint16_t num2lcdbyte(unsigned char num);

//====================================================
struct CursorPoint
{
	int x;
}CursorPoint;

//====================================================
int main(void){
	sei(); //must be set first position
    Port_init();
	I2C_LCD_init(0x27);
	I2C_LCD_Reset();
	I2C_Timer_Default();
    USART_init();
	Timer1_init();
	TIME_SETTING(1,2);
	while(1){
		PORTC ^= 0x01;
		if(end_flag) {
			end_flag = 0;
			timer_start_flag = 0;
			TIME_SETTING(1,2);
			I2C_Timer_Default();
			job = 0;
		}
		switch (job) //can append command and set timer here
		{
		case 0:
			if(timer_start_flag) {
				time_1s = 100;
				job++;
			}
			break;
		case 1:
			if(time_1s) break;
			else job++;
		case 2:
			I2C_Timer_act();
			job++;
		case 3:
			if(end_flag) job++;
			else job = 0;
		default:
			break;
		}
	}
}
//===================================================
void TIME_SETTING(unsigned char m, unsigned char s){
	min = m;
	sec = s;
}
//===================================================
void Timer1_init(){

    TCCR1B =  (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
	TCNT1 = 0;
    OCR1A = 0x09c4; 
    TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);
}

void Port_init(){
    //set B0~D4 as input,  D5, D6, D7 pins are set high to find key vals 
    DDRD = 0b01110000;
    PORTD = 0b00000000;
    DDRB = 0b00110000;


}
//====================================================
void I2C_LCD_Erase(){
	LCD_CharWrite(0x0040);
	LCD_CharWrite(0xA101);
    LCD_CharWrite(0x0040);
    LCD_CharWrite(0xA101);
    LCD_CharWrite(0x0060);
}

void I2C_LCD_Reset(){
	cli();
    LCD_CharWrite(0x0010);
	_delay_us(38);
	LCD_CharWrite(0x0010);
	_delay_us(38);
	LCD_CharWrite(0x0030);
	_delay_us(38);

	I2C_LCD_init(0x27);
	_delay_us(38);

	LCD_CharWrite(0x0010);
	_delay_us(38);
	LCD_CharWrite(0x0030);

	_delay_us(38);

	I2C_LCD_init(0x27);
	_delay_us(38);
	I2C_LCD_init(0x27);
	LCD_CharWrite(0x20C0);
	LCD_CharWrite(0x0020);
	I2C_LCD_init(0x27);
    
	sei();
    LCD_CharWrite(0x00C0);
}
void I2C_LCD_init(unsigned char address){
	cli();

	_delay_ms(18);
	AT24Cxx_i2c_start();
	write_AT24Cxx_i2c_LCDAddressing(address  << 1); //write mode
	LCD_Write(0x20);


	_delay_ms(2);
	write_AT24Cxx_i2c_LCDAddressing(0x20);
	_delay_ms(2);

	write_AT24Cxx_i2c_LCDAddressing(0x30);
	LCD_Write(0x00);
	_delay_us(38);

	LCD_Write(0x00);
	write_AT24Cxx_i2c_LCDAddressing(0x00);
    _delay_us(38);

	LCD_Write(0xE0);
	write_AT24Cxx_i2c_LCDAddressing(0xE0);
	_delay_us(38);

	LCD_Write(0x00);
	write_AT24Cxx_i2c_LCDAddressing(0x00);
	_delay_us(38);

	LCD_Write(0x60);
	write_AT24Cxx_i2c_LCDAddressing(0x60);
	_delay_us(38);

	AT24Cxx_i2c_stop();
    _delay_us(40);
	sei();
	
}
//====================================================
void I2C_Timer_Default(){
	I2C_LCD_Reset();
    LCD_CharWrite(0x2101);
    LCD_CharWrite(0x2101);
    LCD_CharWrite(0x2101);
    LCD_CharWrite(0x2101);
    LCD_CharWrite(num2lcdbyte(0));
    LCD_CharWrite(num2lcdbyte(0));
	LCD_CharWrite(0x31A1);
    LCD_CharWrite(num2lcdbyte(0));
    LCD_CharWrite(num2lcdbyte(0));
    LCD_CharWrite(0x31A1);
    LCD_CharWrite(num2lcdbyte(0));
    LCD_CharWrite(num2lcdbyte(0));
	LCD_CharWrite(0x00C0);
	I2C_LCD_MoveLeft(4);

}

void I2C_Timer_act(){
		/*------- Minutes Part start -------*/
		if(min >= 1){
			I2C_LCD_MoveLeft(3);
			LCD_CharWrite(num2lcdbyte(min));
			I2C_LCD_MoveRight(2);
			LCD_CharWrite(0x00C0);
			LCD_CharWrite(0x00C0);
		}
		/*-------- Minutes part end --------*/

		/*-------- Second part start -------*/
		if(sec == 1){
			if(min){
				if (min == 1){
					I2C_LCD_MoveLeft(3);
					LCD_CharWrite(num2lcdbyte(0));
					I2C_LCD_MoveRight(2);
					LCD_CharWrite(0x00C0);
					LCD_CharWrite(0x00C0);
				}
				LCD_CharWrite(num2lcdbyte(0));
				I2C_LCD_MoveLeft(1);
				time_10ms = 99;
				sec = 60;
				min--;
			}
			else {
				end_flag = 1;
				// LCD_CharWrite(num2lcdbyte(0));
				// I2C_LCD_MoveLeft(1);
			}
		}
		else if(sec == 60){
			I2C_LCD_MoveLeft(1);
			LCD_CharWrite(num2lcdbyte((5)));
			LCD_CharWrite(num2lcdbyte((sec-1)%10));
			I2C_LCD_MoveLeft(1);
			LCD_CharWrite(0x00C0);
			LCD_CharWrite(0x00C0);
			time_10ms = 99;
			--sec;
		}
		else if(sec != 10){
			if(sec % 10 == 0){
				I2C_LCD_MoveLeft(1);
				LCD_CharWrite(num2lcdbyte((sec/10 -1)));
			}
			else{
				I2C_LCD_MoveLeft(1);
				LCD_CharWrite(num2lcdbyte((sec/10)));
			}
			LCD_CharWrite(num2lcdbyte((sec-1)%10));
			I2C_LCD_MoveLeft(1);
			LCD_CharWrite(0x00C0);
			LCD_CharWrite(0x00C0);
			time_10ms = 99;
			--sec;
		}
		else if(sec == 10){
			I2C_LCD_MoveLeft(1);
			LCD_CharWrite(num2lcdbyte((0)));
			LCD_CharWrite(num2lcdbyte((sec-1)%10));
			I2C_LCD_MoveLeft(1);
			LCD_CharWrite(0x00C0);
			LCD_CharWrite(0x00C0);
			time_10ms = 99;
			--sec;
		}
		/*-------- Second part end -------*/

		
	
}
//====================================================
void I2C_LCD_MoveLeft(unsigned char cnt){
	
    while(cnt){
		LCD_CharWrite(0x1000);
		cnt--;
	}
    LCD_CharWrite(0x0060);
    LCD_CharWrite(0x00E0); //must add current state
}

void I2C_LCD_MoveRight(unsigned char cnt){
	while(cnt){
		LCD_CharWrite(0x1040);
		cnt--;
	}
    LCD_CharWrite(0x00E0); //must add current state
}

void I2C_LCD_SELECT(){
	sel_flag ^= 1;
	if(sel_flag){
		LCD_CharWrite(0x00F0);
		LCD_CharWrite(0x00F0);
	} 
	else {
		LCD_CharWrite(0x00E0);
		LCD_CharWrite(0x00E0);
	}
}

//====================================================
ISR(TIMER1_COMPA_vect){
	if(time_1s) time_1s--;
	if(time_10ms) time_10ms--;
	PORTC ^= 0x01;


	if(timer_start_flag && time_10ms){
		
		I2C_LCD_MoveRight(2);
		LCD_CharWrite(num2lcdbyte(time_10ms/10));
		LCD_CharWrite(num2lcdbyte(time_10ms % 10));
		I2C_LCD_MoveLeft(4);
		LCD_CharWrite(0x00C0);
		LCD_CharWrite(0x00C0);
		return;
	}

   	//**NOTE: Timer must perform only the role of timer otherwise, global variables will be reset **
    key_data = key_scan();
	
	if(sel_flag && key_flag){
		key_flag = 0;
		if(key_data == 0xff){
            return;
		}
		if(key_data == 2){
			LCD_CharWrite(num2lcdbyte(2));
        	LCD_CharWrite(0x1000);
        	LCD_CharWrite(0x00F0);
		}
		if(key_data == 8){	
			LCD_CharWrite(num2lcdbyte(8));
			LCD_CharWrite(0x1000);
            LCD_CharWrite(0x00F0);
		}
		if(key_data == 5){
			I2C_LCD_SELECT();
		}
	}

    if(key_flag && !sel_flag){
		key_flag = 0;
		if(key_data == 0xff){
            return;
		}

        if(key_data == 10){
			I2C_Timer_Default();
        }
        if(key_data == 11){
			timer_start_flag ^= 1;
		}                                                       
		if(key_data == 1){
			LCD_CharWrite(0x00E0);
            LCD_CharWrite(0x00E0);
			//key_data += '0';
		}
		
		if(key_data == 3){
			I2C_Timer_act();
		}
		if(key_data == 4){
			I2C_LCD_MoveLeft(1);
		}
		if(key_data == 5){
			I2C_LCD_SELECT();
		}
		if(key_data == 6){
			I2C_LCD_MoveRight(1);
		}
		if(key_data == 7){
			LCD_CharWrite(num2lcdbyte(7));
            LCD_CharWrite(0x00E0);
			//key_data += '0';
		}
		
		if(key_data == 9){
			LCD_CharWrite(num2lcdbyte(9));
            LCD_CharWrite(0x00E0);
			//key_data += '0';
		}
		if(key_data == 0){
			
			LCD_CharWrite(num2lcdbyte(0));
            LCD_CharWrite(0x00E0);
		}
		// LCD_CharWrite(0xF0);
		

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
					// if(!lcd_flag) I2C_LCD_init(0x27);
					// lcd_flag = 1;
                    pressing_flag = 1;
                    key_flag = 1;
                    return key_analysis(new_key, Raw);
                }
            }
        
        }
    }

	return -1;

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