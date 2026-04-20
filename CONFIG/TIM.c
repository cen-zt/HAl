#include "stm32f1xx_hal.h"
#include "sys.h"
#include "TIM.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void TIM2_PWM_Config(void)
{
    // 初始化已经由CubeMX完成，这里只需要启动所有PWM通道
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void TIM3_PWM_Config(void)
{
    // 原理图：VBAT → LED → TIM3引脚 → 低电平点亮
    // 恢复TIM3 PWM功能，引脚复用为AF，CCR=999熄灭
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // CCR=999 → CNT < 999 → 输出高 → 低电平点亮=熄灭
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 999);
}

// 通用定时器中断初始化
// 选择TIM1，3ms中断
void TIM1_Config(void)
{
    // 初始化和中断优先级已经由CubeMX完成，这里只需要启动更新中断
    HAL_TIM_Base_Start_IT(&htim1);
}
