#include "FLASH.h"
#include "stm32f1xx_hal.h"

typedef enum {FAILED = 0, PASSED = !FAILED} Status;

uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t First_Page = 0x00;                       //起始页

//=============================================================================
//函数名称，main
//参数需要数据长度
//返回值，int
//=============================================================================
void FLASH_write(int16_t *data,uint8_t len)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    HAL_FLASH_Unlock();//解锁

    // 擦除一页
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_START_ADDR;
    eraseInit.NbPages = 1;
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    /* 写入FLASH */
    Address = FLASH_START_ADDR;
    while (len--)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, (uint32_t)(*data)) != HAL_OK)
        {
            while (1) { } // 错误等待
        }
        data++;
        Address = Address + 2;
    }
    HAL_FLASH_Lock();
}

void FLASH_read(int16_t *data,uint8_t len)
{
    Address = FLASH_START_ADDR;

    while (len--)
    {
        *data = *(__IO int16_t *)Address;
        data++;
        Address = Address + 2;
    }
}
