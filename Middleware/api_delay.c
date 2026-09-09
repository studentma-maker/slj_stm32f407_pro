/**
  * @file    api_delay.c
  * @brief   延时 API 实现文件
  * @details 基于 SysTick 计数器实现精确的微秒和毫秒级阻塞延时。
  * @version V1.0.0
  * @date    24-Aug-2026
  */

#include "api_delay.h"

/* 私有变量定义 ------------------------------------------------------------*/
static uint32_t g_fac_us = 0; /*!< 微秒延时倍乘数（等于系统时钟频率，单位 MHz） */

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  初始化延时模块
  * @param  None
  * @retval None
  * @note   自动获取当前系统时钟频率，无需手动传入
  */
void Delay_Init(void)
{
    g_fac_us = HAL_RCC_GetSysClockFreq() / 1000000;
}

/**
  * @brief  微秒级延时（阻塞式忙等待）
  * @param  nus  要延时的微秒数
  * @retval None
  * @note   使用 64 位累加防止溢出
  */
void delay_us(uint32_t nus)
{
    /* 参数有效性检查 */
    if (nus == 0 || g_fac_us == 0) {
        return;
    }

    uint32_t reload = SysTick->LOAD;          /* LOAD 寄存器值 */
    uint32_t told   = SysTick->VAL;           /* 进入时的当前计数值 */
    uint64_t ticks  = (uint64_t)nus * g_fac_us; /* 需要的总节拍数（64位防溢出）*/
    uint64_t elapsed = 0;                     /* 已累加的节拍数 */

    while (1) {
        uint32_t tnow = SysTick->VAL;

        if (tnow != told) {
            /* 计算本次变化量，注意递减计数器的溢出 */
            uint32_t delta;
            if (tnow < told) {
                delta = told - tnow;
            } else {
                delta = reload - tnow + told;
            }
            elapsed += delta;
            told = tnow;

            /* 达到或超过要求的节拍数，退出 */
            if (elapsed >= ticks) {
                break;
            }
        }
    }
}

/**
  * @brief  毫秒级延时（阻塞式）
  * @param  nms  要延时的毫秒数
  * @retval None
  * @note   直接调用微秒延时，最大延时受 delay_us 范围限制
  */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));
}

/*----------------------------- End of file -------------------------------*/
