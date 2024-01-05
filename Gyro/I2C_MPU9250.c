#include "I2C_MPU9250.h"

/*MPU 9520 initialization*/
void MPU9250_i2c_init(void){

}

void MPU9250_i2c_start(void){
    MPU9250_SDA_LOW();
    _delay_us(2);
    MPU9250_SDA_HIGH();
    
}

void MPU9250_ACK_send(unsigned char ack_data){

}

void MPU9250_i2c_stop(void){

}

void MPU9250_write(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2, unsigned char data){

}

unsigned char MPU9250_Read(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2){
    
}