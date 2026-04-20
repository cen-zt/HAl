#include "stm32f1xx_hal.h"
#include "sys.h"
#include "ADC.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// 存储ADC采样值，采样10次后平均
uint16_t ADC_Value[10];

void ADC_DMA_Init(void)
{
    // ADC和DMA初始化已经由CubeMX完成
    // 执行ADC校准
    HAL_ADCEx_Calibration_Start(&hadc1);

    // 启动ADC DMA循环转换
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 10);
}

uint16_t GetADCValue(void)
{
    uint8_t i;
    uint32_t tmp = 0;
    for(i = 0; i < 10; i++)
        tmp += ADC_Value[i];
    return (tmp / 10);
}
