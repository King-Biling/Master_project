#ifndef _I2C_H_
#define _I2C_H_

#include "system.h"
#include "sys.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

// Set SDA pin to input mode (PB11)
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}
// Set SDA pin to output mode (PB11)
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}

// IO operation macros
#define IIC_SCL    PBout(10) // SCL pin (PB10) output
#define IIC_SDA    PBout(11) // SDA pin (PB11) output
#define READ_SDA   PBin(11)  // SDA pin (PB11) input

// I2C direction definitions
#ifndef I2C_Direction_Transmitter
#define  I2C_Direction_Transmitter      ((uint8_t)0x00) // Write direction
#endif

#ifndef I2C_Direction_Receiver
#define  I2C_Direction_Receiver         ((uint8_t)0x01) // Read direction
#endif

// ACK/NACK definitions
enum
{
    I2C_ACK,  // Acknowledge
    I2C_NACK  // No acknowledge
};

// Function declarations
int IIC_Start(void);                    // Send IIC start signal
void IIC_Stop(void);                    // Send IIC stop signal
void IIC_Send_Byte(u8 txd);             // Send one byte via IIC
u8 IIC_Read_Byte(unsigned char ack);    // Read one byte via IIC (with ACK/NACK)
int IIC_Wait_Ack(void);                 // Wait for ACK signal
void IIC_Ack(void);                     // Send ACK signal
void IIC_NAck(void);                    // Send NACK signal

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
void I2C_GPIOInit(void);                // Initialize IIC GPIO pins

#endif
