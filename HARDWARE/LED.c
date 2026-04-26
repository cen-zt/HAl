/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief
 ******************************************************************/
#include "stm32f1xx_hal.h"
#include "LED.h"
#include "ALL_DATA.h"

// 四个状态LED使用TIM3 PWM输出
// LED1=PA6(TIM3_CH1), LED2=PA7(TIM3_CH2), LED3=PB0(TIM3_CH3), LED4=PB1(TIM3_CH4)
// 原理图：VBAT → LED → TIM3引脚 → 低电平点亮
// 低电平点亮：CCR=0 = 永远低 = 亮，CCR=999 = 永远高 = 灭
////前LED1 (PA6)
#define fLED1_H()  TIM3->CCR1 = 999  //灭
#define fLED1_L()  TIM3->CCR1 = 0    //亮
#define fLED1_Toggle()  TIM3->CCR1 = (TIM3->CCR1 == 999 ? 0 : 999) //翻转
////前LED3 (PB0)
#define fLED3_H()  TIM3->CCR3 = 999  //灭
#define fLED3_L()  TIM3->CCR3 = 0    //亮
#define fLED3_Toggle()  TIM3->CCR3 = (TIM3->CCR3 == 999 ? 0 : 999) //翻转
//-------------------------------------------------
////后LED2 (PA7)
#define bLED2_H()  TIM3->CCR2 = 999  //灭
#define bLED2_L()  TIM3->CCR2 = 0    //亮
#define bLED2_Toggle()  TIM3->CCR2 = (TIM3->CCR2 == 999 ? 0 : 999) //翻转
////尾LED4 (PB1)
#define bLED4_H()  TIM3->CCR4 = 999  //灭
#define bLED4_L()  TIM3->CCR4 = 0    //亮
#define bLED4_Toggle()  TIM3->CCR4 = (TIM3->CCR4 == 999 ? 0 : 999) //翻转
//-------------------------------------------------

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select a LED statue on enum contains            */
sLED LED = {300, AllFlashLight};  //LED initial status: 300ms interval, flashing (unlocked)

/**************************************************************
 *  LED system
 * @param[in]
 * @param[out]
 * @return
 ***************************************************************/
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(uwTick - LastTime < LED.FlashTime)
	{
		return;
	}
	else
		LastTime = uwTick;

	switch(LED.status)
	{
		case AlwaysOff:      //全灭
			fLED1_H();
			fLED3_H();
			bLED2_H();
			bLED4_H();
			break;

		case AllFlashLight:  //未解锁：全部同时闪烁
			fLED1_Toggle();
			fLED3_Toggle();
			bLED2_Toggle();
			bLED4_Toggle();
			break;

		case AlwaysOn:       //解锁后：全部常亮
			fLED1_L();
			fLED3_L();
			bLED2_L();
			bLED4_L();
			break;

		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



