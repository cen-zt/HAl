#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f1xx_hal.h"

typedef uint8_t u8;

void SPI_Config(void);
u8 SPI_RW(u8 dat);

#endif
