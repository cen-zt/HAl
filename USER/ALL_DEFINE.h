#ifndef __ALL_DEFINE_H__
#define __ALL_DEFINE_H__
#include "stm32f1xx_hal.h"

// sys.h 必须先包含，这里有#define SysTick_count uwTick 宏映射
// 需要在ALL_DATA.h声明之前让宏生效
#include "sys.h"

// 兼容原代码的简写类型定义
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;

#include "ALL_DATA.h"
#include "INIT.h"
#include "I2C.h"
#include "SPI.h"
#include "nrf24l01.h"
#include "USART.h"
#include  "TIM.h"
#include "LED.h"
#include "mpu6050.h"
#include "imu.h"
#include "ANO_DT.h"
#include "Remote.h"
#include "control.h"
#include "myMath.h"
#include "USB_HID.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1



#endif

