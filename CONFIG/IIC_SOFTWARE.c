/******************** (C) COPYRIGHT 2013 YunMiao ********************
 * File Name          : main.c
 * Author             : YunMiao
 * Version            : V2.0.1
 * Date               : 08/01/20013
 * Description        : IIC basic function
 ********************************************************************************
 ********************************************************************************
 *******************************aircraft****************************************/
#include "I2C.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/******************************************************************************
 * 函数名称: I2c_delay
 * 描述说明: I2c 延时函数
 * 输入参数: 无
 ******************************************************************************/
void I2c_delay()  {
    volatile unsigned char i = 2;
    while (i)
         i--;
}

/******************************************************************************
 * 函数名称: I2c_Init
 * 描述说明: I2c  GPIO初始化
 * 说明: GPIO已经由CubeMX初始化完成，这里不需要重复初始化
 ******************************************************************************/
 void IIC_Init(void)
{
    // GPIO已经在MX_GPIO_Init()中配置完成
    // PB6=SCL, PB7=SDA
    // 释放I2C总线，进入空闲状态（SCL高，SDA高）
    // 因为MX_GPIO_Init提前把所有引脚复位成低电平了，需要手动拉高
    SCL_H;
    SDA_H;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Start
 * 描述说明: I2c  起始信号
 * 输入参数: 无
 ******************************************************************************/
static uint8_t I2c_Start(void)
{
    SDA_H;
    SCL_H;
	I2c_delay();
    if (!SDA_read)
        return FAILED;
    SDA_L;
    I2c_delay();
    if (SDA_read)
        return FAILED;
    SCL_L;
    I2c_delay();
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: I2c_Stop
 * 描述说明: I2c  停止信号
 * 输入参数: 无
 ******************************************************************************/
static void I2c_Stop(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
	I2c_delay();
    I2c_delay();
    SCL_H;
	I2c_delay();
    SDA_H;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Ack
 * 描述说明: I2c  产生应答信号
 * 输入参数: 无
 ******************************************************************************/
static void I2c_Ack(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_NoAck
 * 描述说明: I2c  产生NAck
 * 输入参数: 无
 ******************************************************************************/
static void I2c_NoAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/*******************************************************************************
*函数名:	I2c_WaitAck
*描述:	等待应答信号到来
*返回值:   1:无应答  错误
*           0:应答成功
 *******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
    I2c_delay();

    if (SDA_read) {
        SCL_L;
        return FAILED;
    }
    SCL_L;
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: I2c_SendByte
 * 描述说明: I2c  发送一个字节
 * 输入参数: byte  要发送的数据
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2c_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2c_delay();
        SCL_H;
		I2c_delay();
		I2c_delay();
		I2c_delay();
    }
    SCL_L;
}

/******************************************************************************
 * 函数名称: I2c_ReadByte
 * 描述说明: I2c  读取一个字节
 * 输入参数: 无
 * 返回值	 读取的数据
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2c_delay();
		I2c_delay();
        SCL_H;
		I2c_delay();
        I2c_delay();
		I2c_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

/******************************************************************************
 *函数名称:	i2cWriteBuffer
 *描述说明: I2c       往器件的某个地址写n个字节，接收地址
 *输入参数: addr,     器件地址
 *           reg，     寄存器地址
 *			 len，     数据长度
 *			 *data	   数据指针
 * 返回值	 1
 ******************************************************************************/
int8_t IIC_Write_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    int i;
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    for (i = 0; i < len; i++) {
        I2c_SendByte(data[i]);
        if (I2c_WaitAck() == FAILED) {
            I2c_Stop();
            return FAILED;
        }
    }
    I2c_Stop();
    return SUCCESS;
}

int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
{
	uint8_t recive = 0;
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
	I2c_SendByte(addr+1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
	recive = I2c_ReadByte();
	 I2c_NoAck();
	I2c_Stop();
	return recive;
}

/*****************************************************************************
 *函数名称:	i2cWrite
 *描述:	写入指针，器件 指针，寄存器，一个字节
 *输入参数: addr 从器件地址
 *		     reg   寄存器地址
 *		     data 数据，要写入指针地址
 *****************************************************************************/
int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    I2c_SendByte(data);
    I2c_WaitAck();
    I2c_Stop();
    return SUCCESS;
}

int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
    I2c_SendByte(addr+1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    while (len) {
        *data = I2c_ReadByte();
        if (len == 1)
            I2c_NoAck();
        else
            I2c_Ack();
        data++;
        len--;
    }
    I2c_Stop();
    return SUCCESS;
}
