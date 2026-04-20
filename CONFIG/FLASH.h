#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f1xx_hal.h"

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE    (0x00000400) //FLASH页的大小1K
#endif

#define FLASH_START_ADDR   (0x08007c00) //要写入的开始地址
#define FLASH_END_ADDR     (0x08007fff) //要写入的结束地址
#define DATA_32            (0x12345678) //要写入的标记

// 兼容原代码名称
#define FLASH_Page_Size FLASH_PAGE_SIZE
#define FLASH_Start_Addr FLASH_START_ADDR
#define FLASH_End_Addr FLASH_END_ADDR

extern void FLASH_read(int16_t *data,uint8_t len);
extern void FLASH_write(int16_t *data,uint8_t len);

#endif
