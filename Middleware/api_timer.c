/**
  * @file    api_timer.c
  * @brief   定时器 API 接口源文件
  * @details 本文件提供了上层应用定时器接口的空框架，
  *          包括定时器初始化、启动、停止及回调管理等函数。
  * @version V1.0.0
  * @date    21-Aug-2026
  */
#include "api_timer.h"
#include "tim.h"

/* 私有宏定义 --------------------------------------------------------------*/

/* 周期计数器最大值（基于 100us 基础周期） */
#define BSP_TIMER_CNT_200US         2U          /**< 200us = 2 * 100us */
#define BSP_TIMER_CNT_500US         5U          /**< 500us = 5 * 100us */
#define BSP_TIMER_CNT_1MS           10U         /**< 1ms = 10 * 100us */
#define BSP_TIMER_CNT_2MS           20U         /**< 2ms = 20 * 100us */
#define BSP_TIMER_CNT_5MS           50U         /**< 5ms = 50 * 100us */
#define BSP_TIMER_CNT_10MS          100U        /**< 10ms = 100 * 100us */
#define BSP_TIMER_CNT_100MS         1000U       /**< 100ms = 1000 * 100us */
#define BSP_TIMER_CNT_1S            10000U      /**< 1s = 10000 * 100us */

/* 私有变量定义 ------------------------------------------------------------*/

/** 定时器周期标志位 */
static volatile BSP_TASK_TIMER_ST s_timerFlags = {0};

/** 周期计数器（每次中断递增） */
static volatile uint32_t s_tickCounter = 0U;

/* 公共函数实现 ------------------------------------------------------------*/

/**
 * @brief 获取定时器周期标志位
 */
BSP_TASK_TIMER_ST bsp_timer_GetFlags(void)
{
    return s_timerFlags;
}

/**
 * @brief 清除指定的周期标志位
 */
void bsp_timer_ClearFlag(uint8_t flagMask)
{
    /* 使用原子操作清除指定位 */
    __disable_irq();
    s_timerFlags.byte &= (uint8_t)(~flagMask);
    __enable_irq();
}

/**
 * @brief 清除所有周期标志位
 */
void bsp_timer_ClearAllFlags(void)
{
    __disable_irq();
    s_timerFlags.byte = 0U;
    __enable_irq();
}

/**
 * @brief 检查指定标志位是否置位
 */
bool bsp_timer_IsFlagSet(uint8_t flagMask)
{
    return ((s_timerFlags.byte & flagMask) != 0U);
}


/**
  * @brief  定时器中断回调函数
  * @param  htim: 定时器句柄
  * @retval none
  * @note   none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim11.Instance)
    {
        /* 递增计数器 */
        s_tickCounter++;

        /* ====== 100us 标志（每次中断都置位） ====== */
        s_timerFlags.bit.cycle_100us_flg = 1;

        /* ====== 200us 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_200US) == 0U) {
            s_timerFlags.bit.cycle_200us_flg = 1;
        }

        /* ====== 500us 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_500US) == 0U) {
            s_timerFlags.bit.cycle_500us_flg = 1;
        }

        /* ====== 1ms 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_1MS) == 0U) {
            s_timerFlags.bit.cycle_1ms_flg = 1;
        }

        /* ====== 2ms 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_2MS) == 0U) {
            s_timerFlags.bit.cycle_2ms_flg = 1;
        }

        /* ====== 5ms 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_5MS) == 0U) {
            s_timerFlags.bit.cycle_5ms_flg = 1;
        }
        
        /* ====== 10ms 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_10MS) == 0U) {
            s_timerFlags.bit.cycle_10ms_flg = 1;
        }
        
        /* ====== 100ms 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_100MS) == 0U) {
            s_timerFlags.bit.cycle_100ms_flg = 1;
        }

        /* ====== 1s 标志 ====== */
        if ((s_tickCounter % BSP_TIMER_CNT_1S) == 0U) {
            s_timerFlags.bit.cycle_1s_flg = 1;
            /* 1s 到达时同时重置计数器，防止溢出 */
            s_tickCounter = 0U;
        }
    }
}

/*----------------------------- End of file -------------------------------*/
