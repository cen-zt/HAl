#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f1xx_hal.h"

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE    (0x00000400) //FLASH页的大小1K
#endif

// STM32F103C8T6 总共128KB FLASH，把校准数据放在**最后一页**，避免被程序下载覆盖
// 128KB - 1KB = 127KB = 0x1F800 → 起始地址 0x08000000 + 0x1F800 = 0x0801F800
#define FLASH_START_ADDR   (0x0801F800) //校准数据保存在最后一页，不会被程序覆盖
#define FLASH_END_ADDR     (0x0801FFFF) //结束地址
#define DATA_32            (0x12345678) //要写入的标记

// 兼容原代码名称
#define FLASH_Page_Size FLASH_PAGE_SIZE
#define FLASH_Start_Addr FLASH_START_ADDR
#define FLASH_End_Addr FLASH_END_ADDR

extern void FLASH_read(int16_t *data,uint8_t len);
extern void FLASH_write(int16_t *data,uint8_t len);

#endif
