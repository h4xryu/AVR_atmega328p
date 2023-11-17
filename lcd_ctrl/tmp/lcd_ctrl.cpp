#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#define F_SCL 100000UL
#define RS1_EN1 0x05
#define RS1_EN0 0x01
#define RS0_EN0 0x00
#define Backlight 0x08
#endif

uint8_t I2C_addr_PCF8574 = (0x27 << 1);

#include "i2c/i2c.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void twi_init(void);
void i2c_lcd_init(void);
void i2c_lcd_command(uint8_t command);
void i2c_lcd_data(uint8_t data);
void i2c_lcd_goto_XY(uint8_t row, uint8_t col);
void i2c_lcd_write_string(char string[]);

int main(void){
	uint8_t distance;
	char dinstance_char[4];
	stdout = &OUTPUT;
	stdin = &INPUT;
	
	UART0_init();
	i2c_lcd_init();
	
	DDRB |= 0X02; //트리거핀 출력
	DDRB &= 0XFE; // 에코핀 입력
	
	Timer_init();
	i2c_lcd_write_string("Measure Start!");
	_delay_ms(1000);
	i2c_lcd_command(0x01); //lcd clear;
	i2c_lcd_goto_XY(0,0);
	i2c_lcd_write_string("Distance : ");
	i2c_lcd_goto_XY(1,14);
	i2c_lcd_write_string("CM");
	while(1)
	{	

    }
    return 0;
}

void i2c_lcd_init(void){
    lcd_init();
    _delay_ms(50);
	
	i2c_lcd_command_8(0x30); _delay_ms(5);
	i2c_lcd_command_8(0x30); _delay_us(100);
	i2c_lcd_command_8(0x30); _delay_us(100);
	i2c_lcd_command_8(0x20); _delay_us(100);
	
	i2c_lcd_command(0x28); _delay_us(50);
	i2c_lcd_command(0x08); _delay_us(50); //display on/off conrol 
	i2c_lcd_command(0x01); _delay_ms(3); // clear display
	i2c_lcd_command(0x06); _delay_us(50);
	i2c_lcd_command(0x0c); _delay_us(50); // display on , cursor & cursor blink off

}

void i2c_lcd_command(uint8_t command)
{
	uint8_t c_buf[4];
	
	c_buf[0] = (command&0xf0) | RS0_EN1 | Backlight;
	c_buf[1] = (command&0XF0) | RS0_EN0 | Backlight;
	c_buf[2] = ((command<<4)&0XF0) | RS0_EN1 |Backlight;
	c_buf[3] = ((command<<4)&0XF0) | RS0_EN0 |Backlight;
	
	i2c_transmit_nbytes(I2C_addr_PCF8574,c_buf,4);
}

void i2c_lcd_goto_XY(uint8_t row, uint8_t col)
{
	uint8_t address = (0x40 * row) + col;
	uint8_t command = 0x80 | address;
	
	i2c_lcd_command(command);
}