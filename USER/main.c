/*******************************************************************
 * 主函数
 *@brief 系统初始化，数据读取，控制的输出
 *@brief
 ******************************************************************/
#include "ALL_DEFINE.h"

// 旧的main函数 - 已经和CubeMX生成的Core/Src/main.c冲突，已注释掉
// 所有飞控代码已经整合到Core/Src/main.c中了
/*
int main(void)
{
	cycleCounterInit();  //得到系统每次us的CLK数目，为之后定时计算提供标准的时间计算使用
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2); //2个bit的抢占优先级，2个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);	//系统滴答计时

	ALL_Init();//系统初始化
	 while循环里放一些比较低优先级需要处理的任务，比如指示灯LED，
	 遥控器数据读取，中断处理，只需要一个3ms中断一次的中断，在scheduler.c里面处理
	while(1)
	{
			ANTO_polling(); //遥控器数据处理
			Usb_Hid_Receive();//USB接收数据处理
			PilotLED(); //LED闪烁
	}
}
*/
