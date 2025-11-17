#include "I2C.h"

/**************************************************************************
 * Initialize IIC GPIO pins (PB10:SCL, PB11:SDA)
 **************************************************************************/

void I2C_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ??GPIOB?? */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ??SCL(PB10)?SDA(PB11)?? */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // ??????
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // ????
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // ??
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ??????? */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

/**************************************************************************
 * Send IIC start signal
 * Return: 1=success, 0=fail
 **************************************************************************/
int IIC_Start(void)
{
    SDA_OUT();       // Set SDA to output
    IIC_SDA=1;
    if(!READ_SDA)return 0;    
    IIC_SCL=1;
    delay_us(1);
    IIC_SDA=0;       // Start: SDA falls when SCL is high
    if(READ_SDA)return 0;
    delay_us(1);
    IIC_SCL=0;       // Hold I2C bus, ready to send/receive data
    return 1;
}

/**************************************************************************
 * Send IIC stop signal
 **************************************************************************/
void IIC_Stop(void)
{
    SDA_OUT();       // Set SDA to output
    IIC_SCL=0;
    IIC_SDA=0;       // Stop: SDA rises when SCL is high
    delay_us(1);
    IIC_SCL=1; 
    IIC_SDA=1;       // Send stop signal
    delay_us(1);                   
}

/**************************************************************************
 * Wait for ACK signal
 * Return: 1=ACK received, 0=no ACK
 **************************************************************************/
int IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN();        // Set SDA to input
    IIC_SDA=1;
    delay_us(1);       
    IIC_SCL=1;
    delay_us(1);     
    while(READ_SDA)  // Wait until SDA is low (ACK)
    {
        ucErrTime++;
        if(ucErrTime>50)  // Timeout
        {
            IIC_Stop();
            return 0;
        }
        delay_us(1);
    }
    IIC_SCL=0;       // Lower SCL
    return 1;  
} 

/**************************************************************************
 * Send ACK signal
 **************************************************************************/
void IIC_Ack(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=0;       // ACK: low level
    delay_us(1);
    IIC_SCL=1;
    delay_us(1);
    IIC_SCL=0;
}

/**************************************************************************
 * Send NACK signal
 **************************************************************************/
void IIC_NAck(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=1;       // NACK: high level
    delay_us(1);
    IIC_SCL=1;
    delay_us(1);
    IIC_SCL=0;
}

/**************************************************************************
 * Send one byte via IIC
 * Input: txd=byte to send
 **************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    SDA_OUT();        
    IIC_SCL=0;       // Lower SCL to start sending
    for(t=0;t<8;t++) // Send 8 bits
    {              
        IIC_SDA=(txd&0x80)>>7;  // Send MSB first
        txd<<=1;      
        delay_us(1);   
        IIC_SCL=1;    // High SCL: slave reads bit
        delay_us(1); 
        IIC_SCL=0;    // Lower SCL for next bit
        delay_us(1);
    }     
} 

/**************************************************************************
 * Write data to device register(s)
 * Input: addr=device address, reg=register address, len=byte count, data=data ptr
 * Return: 0=success, 1=fail
 **************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );  // Send write address
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         // Send register address
    IIC_Wait_Ack();
    for (i = 0; i < len; i++) { // Send data bytes
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
 * Read data from device register(s)
 * Input: addr=device address, reg=register address, len=byte count, buf=data buf ptr
 * Return: 0=success, 1=fail
 **************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);   // Send write address
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         // Send register address
    IIC_Wait_Ack();
    IIC_Start();                // Repeated start
    IIC_Send_Byte((addr << 1)+1); // Send read address
    IIC_Wait_Ack();
    while (len) {               // Read data bytes
        if (len == 1)
            *buf = IIC_Read_Byte(0); // Last byte: send NACK
        else
            *buf = IIC_Read_Byte(1); // Middle bytes: send ACK
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
 * Read one byte via IIC
 * Input: ack=1 send ACK, 0 send NACK
 * Return: received byte
 **************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();       // Set SDA to input
    for(i=0;i<8;i++ )  // Read 8 bits
    {
        IIC_SCL=0; 
        delay_us(2);
        IIC_SCL=1;      // High SCL: master reads bit
        receive<<=1;
        if(READ_SDA)receive++;   // LSB first
        delay_us(2); 
    }                     
    if (ack)
        IIC_Ack();      // Send ACK
    else
        IIC_NAck();     // Send NACK
    return receive;
}

/**************************************************************************
 * Read one byte from specific register
 * Input: I2C_Addr=device address, addr=register address
 * Return: read data
 **************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
    unsigned char res=0;
    
    IIC_Start();    
    IIC_Send_Byte(I2C_Addr);    // Send write command
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);        // Send register address
    res++;  
    IIC_Wait_Ack();    
    IIC_Start();
    IIC_Send_Byte(I2C_Addr+1);  // Enter read mode             
    IIC_Wait_Ack();
    res=IIC_Read_Byte(0);       // Read data (send NACK)   
    IIC_Stop();

    return res;
}

/**************************************************************************
 * Continuously read bytes from register
 * Input: dev=device addr, reg=register addr, length=byte count, data=buf ptr
 * Return: count-1 (number of bytes read -1)
 **************************************************************************/
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    
    IIC_Start();
    IIC_Send_Byte(dev);         // Send write command
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         // Send register address
    IIC_Wait_Ack();      
    IIC_Start();
    IIC_Send_Byte(dev+1);       // Enter read mode    
    IIC_Wait_Ack();
    
    for(count=0;count<length;count++){
        if(count!=length-1)   
            data[count]=IIC_Read_Byte(1);  // Read with ACK
        else                  
            data[count]=IIC_Read_Byte(0);  // Last byte with NACK
    }
    IIC_Stop();
    return count;
}

/**************************************************************************
 * Write multiple bytes to register
 * Input: dev=device addr, reg=register addr, length=byte count, data=buf ptr
 * Return: 1=success
 **************************************************************************/
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{  
    u8 count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);         // Send write command
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         // Send register address
    IIC_Wait_Ack();      
    for(count=0;count<length;count++){  // Send data bytes
        IIC_Send_Byte(data[count]); 
        IIC_Wait_Ack(); 
    }
    IIC_Stop();
    return 1; 
}

/**************************************************************************
 * Read one byte from register (store in data)
 * Input: dev=device addr, reg=register addr, data=ptr to store result
 * Return: 1=success
 **************************************************************************/
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
    *data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
 * Write one byte to specific register
 * Input: dev=device addr, reg=register addr, data=byte to write
 * Return: 1=success
 **************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
 * Read-modify-write multiple bits in a register
 * Input: dev=device addr, reg=register addr, bitStart=start bit, length=bit count, data=bit values
 * Return: 1=success, 0=fail
 **************************************************************************/
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {  // Read current value
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;       // Clear target bits
        b |= data;       // Set new bits
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************************************************************
 * Read-modify-write one bit in a register
 * Input: dev=device addr, reg=register addr, bitNum=bit position, data=0(clear)/1(set)
 * Return: 1=success, 0=fail
 **************************************************************************/
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IICreadByte(dev, reg, &b);  // Read current value
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum)); // Modify bit
    return IICwriteByte(dev, reg, b);
}

/**************************************************************************
 * Write one byte to device register (simplified)
 * Input: DevAddr=device addr, RegAddr=register addr, Data=byte to write
 * Return: 1=success
 **************************************************************************/
u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    IIC_Start();
    IIC_Send_Byte(DevAddr | I2C_Direction_Transmitter); // Send write addr
    IIC_Wait_Ack();
    IIC_Send_Byte(RegAddr);  // Send register addr
    IIC_Wait_Ack();
    IIC_Send_Byte(Data);     // Send data
    IIC_Wait_Ack();
    IIC_Stop();
    return 1;
}

