#ifndef __SYS_H
#define __SYS_H

//#pragma pack(push,4)
//#pragma pack(4)
//__unaligned
//__packed
//__align(4)

#include "stm32f1xx_hal.h"
#include "delay.h"      //system delay,common.

// HAL中使用HAL自带的uwTick作为系统节拍计数
// 声明为extern，避免每个文件定义副本
extern volatile uint32_t uwTick;
#define SysTick_count uwTick

extern void ALL_Init(void);

#endif

/******************************END OF FILE *******************************************/

