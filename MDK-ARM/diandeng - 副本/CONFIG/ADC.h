#ifndef __ADC_H
#define __ADC_H

#include "stm32f1xx_hal.h"

extern uint16_t ADC_Value[10];
extern void ADC_DMA_Init(void);
extern uint16_t GetADCValue(void);

#endif
