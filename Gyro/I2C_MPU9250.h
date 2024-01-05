#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MPU9250PORT PORTA
#define MPU9250DDR DDRA
#define MPU9250PIN PINA
#define MPU9250_SCL 0x01 //A0
#define MPU9250_SDA 0x02 //A1

#define SLAVE1_ADDR 0x50
#define SLAVE2_ADDR 0x51

#define ACK 0x00
#define NACK 0x01

#define MPU9250SDA_HIGH() (MPU9250PORT |= MPU9250_SDA)
#define MPU9250_SDA_LOW() (MPU9250PORT &= ~MPU9250_SDA)
#define MPU9250_SCL_HIGH() (MPU9250PORT |= MPU9250_SCL)
#define MPU9250_SCL_LOW() (MPU9250PORT &= ~(MPU9250_SCL))
#define MPU9250_SDA_IN() (MPU9250DDR &= ~MPU9250_SDA)
#define MPU9250_SDA_OUT() (MPU9250DDR |= MPU9250_SDA)


typedef struct{
    int x;
    int y;
    int z;
}Point3D;

void MPU9250_i2c_init(void);
void MPU9250_i2c_start(void);
void MPU9250_ACK_send(unsigned char ack_data);
void MPU9250_NACK_send(unsigned char nack_data);
void MPU9250_i2c_stop(void);
void MPU9250_write(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2, unsigned char data);
unsigned char MPU9250_Read(unsigned char address, unsigned char sub_addr1, unsigned char sub_addr2);

volatile Point3D myPoint;
volatile Point3D currentPoint;
volatile unsigned char start_flag = 0;
volatile unsigned char stop_flag = 0;

