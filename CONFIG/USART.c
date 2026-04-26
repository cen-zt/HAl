/*******************************************************************
 *MPU6050
 *@brief
 *@brief
 ******************************************************************/

#include "usart.h"
#include "stm32f1xx_hal.h"
#include "delay.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

// 原标准库DMA发送函数，适配HAL
void Uart_Start_DMA_Tx(const int8_t *data, uint8_t size)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, size);
}

// 阻塞方式发送字节
void USART1_SendByte(const int8_t *Data, uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)(Data + i), 1, HAL_MAX_DELAY);
        while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
    }
}

// 设置波特率
void USART1_setBaudRate(uint32_t baudRate)
{
    huart1.Init.BaudRate = baudRate;
    HAL_UART_Init(&huart1);
}

// printf重定向
int fputc(int ch, FILE *f)
{
    /* 将Printf内容发往串口 */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
    return (ch);
}
