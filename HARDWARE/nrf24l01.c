#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


#define MAX_TX  		0x10  //到达最大重试次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收完成中断


//发送地址
const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xe5};	//发送地址
const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xe5};	//接收地址RX_ADDR_P0 == RX_ADDR

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01引脚定义
#define Set_NRF24L01_CSN    (GPIOB->BSRR = GPIO_PIN_12) // PB12
#define Clr_NRF24L01_CSN     (GPIOB->BRR = GPIO_PIN_12)    // PB12
#define Set_NRF24L01_CE (GPIOB->BSRR = GPIO_PIN_8)    // PB8
#define Clr_NRF24L01_CE  (GPIOB->BRR = GPIO_PIN_8)    // PB8
#define READ_NRF24L01_IRQ   (GPIOA->IDR&GPIO_PIN_8)//IRQ引脚输入 PA8

//初始化24L01的IO口
void NRF24L01_Configuration(void)
{
	// GPIO已经由CubeMX初始化完成，时钟和模式都配置好了
	// 这里只需要设置初始电平

	Set_NRF24L01_CE;                                    //初始化时候拉高
	Set_NRF24L01_CSN;                                   //初始化时候拉高

	SPI_Config();                                //初始化SPI
	Clr_NRF24L01_CE; 	                                //使能24L01
	Set_NRF24L01_CSN;                                   //SPI片选选中
}
//检测NRF24L01是否在位
//写5个0xa5，然后读取出来比较，
//同时返回值:0表示在位；返回1表示不在位
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i;
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写5个字节地址.
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址
	for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;
	if(i!=5)return 1;                               //NRF24L01不在线
	return 0;		                                //NRF24L01在位
}
//通过SPI写寄存器
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;
    Clr_NRF24L01_CSN;                    //使能SPI总线
  	status =SPI_RW(regaddr); //写寄存器地址
  	SPI_RW(data);            //写寄存器值
  	Set_NRF24L01_CSN;                    //禁止SPI总线
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 regaddr:要读的寄存器
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;
	Clr_NRF24L01_CSN;                //使能SPI总线
  	SPI_RW(regaddr);     //写寄存器地址
  	reg_val=SPI_RW(0XFF);//读取寄存器值
  	Set_NRF24L01_CSN;                //禁止SPI总线
  	return(reg_val);                 //返回状态值
}
//指定位置读取指定长度的数据
//*pBuf:数据指针
//返回值,这里返回状态寄存器的值
u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;
  	Clr_NRF24L01_CSN;                     //使能SPI总线
  	status=SPI_RW(regaddr);   //写寄存器地址(读取)，读取状态值
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_RW(0XFF);//循环读取
  	Set_NRF24L01_CSN;                     //释放SPI总线
  	return status;                        //返回状态寄存器
}
//指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,这里返回状态寄存器的值
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;
	Clr_NRF24L01_CSN;                                    //使能SPI总线
  	status = SPI_RW(regaddr);                //写寄存器地址(读取)，读取状态值
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_RW(*pBuf++); //写数据
  	Set_NRF24L01_CSN;                                    //释放SPI总线
  	return status;                                       //返回状态寄存器
}
//发射NRF24L01发送一个数据包
//txbuf:要发送的数据地址
//返回值:发送结果
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 state;
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32字节
	Set_NRF24L01_CE;                                     //开始发送
	while(READ_NRF24L01_IRQ!=0);                         //等待发送完成
	state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清TX_DS和MAX_RT中断标志
	if(state&MAX_TX)                                     //达到最大重试次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清TX FIFO寄存器
		return MAX_TX;
	}
	if(state&TX_OK)                                      //发送完成
	{
		return SUCCESS;
	}
	return FAILED;                                         //其他原因失败
}

//接收NRF24L01接收一个数据包
//rxbuf:接收的数据地址
//返回值:0没收到包，非0就是收到包
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;
	u8 result;

	state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清TX_DS和MAX_RT中断标志
	if(state&RX_OK)                                 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清RX FIFO寄存器

		return SUCCESS;
	}

	return FAILED;                                      //没收到数据错误
}



void RX_Mode(void)
{

		Clr_NRF24L01_CE;
    //写RX通道地址
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);

    //使能通道0自动应答
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0);
    //使能通道0接收地址
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
    //设置RF通道频率
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,1);
    //设置通道0接收数据宽度
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    //设置TX发射功率,0dbm，2Mbps，增强版 shockburst
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x07);
    //设置配置寄存器模式;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX模式
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f);
    //CE拉高，进入接收模式
  	Set_NRF24L01_CE;
}

void NRF24L01_init(void)
{
	NRF24L01_Configuration();
  do
	{
	    RX_Mode();
	}while(NRF24L01_Check() == FAILED);
}

/*********************END OF FILE******************************************************/
