/*******************************************************************
 *@brief
 *@brief
 *@time ZIN飞控 延时 2017.1.8
 *@editor黑马&zin
 * 技术交流QQ群551883670，邮箱759421287@qq.com
 * 版权禁止商业用途，禁止使用，禁止二次开发，版权所有
 *
 ******************************************************************/
#include "Spi.h"

#define  SCK_H  GPIOB->BSRR=GPIO_PIN_13  //SCK拉高
#define  SCK_L  GPIOB->BRR=GPIO_PIN_13  //SCK拉低
#define  MOSI_H  GPIOB->BSRR=GPIO_PIN_15  //MOSI拉高
#define  MOSI_L  GPIOB->BRR=GPIO_PIN_15  //MOSI拉低
#define  MISO  ((GPIOB->IDR&GPIO_PIN_14)?1:0)  //读取MISO

// SPI软件配置，GPIO已经由CubeMX初始化完成
void SPI_Config(void)//io初始化，nrf
{
    // GPIO已经在MX_GPIO_Init()中配置完成
    // 这里只设置初始电平
    SCK_L;
    MOSI_H;
}

u8 SPI_RW(u8 byte)
{
	uint8_t i;

	u8 Temp=0x00;

	for (i = 0; i < 8; i++)

	{

			SCK_L;//sclk = 0;//时钟低准备

			if (byte&0x80)
			{
					MOSI_H; // //SO=1
			}
			else
			{
					MOSI_L;// //SO=0
			}

			byte <<= 1;
			Temp<<=1;

			SCK_H; //时钟高

			if(MISO) //读取1时
			{
				Temp++;
			}
			SCK_L;
	}
			return Temp; //返回miso读出的值

}
