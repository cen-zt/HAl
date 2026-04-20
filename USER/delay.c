/*******************************************************************
 *MPU6050
 *@brief
 *@brief
 *@time  2016.1.8
 *@editor黑马&zin
 * 技术交流QQ群551883670，邮箱759421287@qq.com
 * 版权禁止商业用途，禁止使用，禁止二次开发，版权所有
 ******************************************************************/
#include "stm32f1xx_hal.h"
#include "delay.h"
#include "ALL_DATA.h"
static volatile uint32_t usTicks = 0;

// HAL的SysTick计数变量
extern volatile uint32_t uwTick;

void cycleCounterInit(void)
{
    // SystemCoreClock 直接给出系统时钟频率 (单位: Hz)
    usTicks = SystemCoreClock / 1000000;
}


uint32_t GetSysTime_us(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = uwTick;
        cycle_cnt = SysTick->VAL;
    	} while (ms != uwTick);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//    系统延时函数
void delay_ms(uint16_t nms)
{
	uint32_t t0=GetSysTime_us();
	while(GetSysTime_us() - t0 < nms * 1000);
}

void delay_us(unsigned int i)
 {
	char x=0;
    while( i--)
    {
       for(x=1;x>0;x--);
    }
 }
