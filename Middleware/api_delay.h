/**
  * @file    api_delay.h
  * @brief   延时 API 头文件
  * @details 提供基于 SysTick 的微秒和毫秒级延时接口。
  * @version V1.0.0
  * @date    24-Aug-2026
  */
#ifndef __API_DELAY_H__
#define __API_DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 导入标准库及 HAL 库头文件 ------------------------------------------------*/
#include "main.h"

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  初始化延时模块
  * @param  None
  * @retval None
  * @note   自动获取当前系统时钟频率，无需手动传入
  */
void Delay_Init(void);

/**
  * @brief  微秒级延时（阻塞式忙等待）
  * @param  nus  要延时的微秒数，范围建议 0 ~ 1,000,000（1秒内）
  * @retval None
  * @note   1. 延时过程中会暂时关闭 SysTick 中断，以提高短延时精度；
  *         2. 本函数为阻塞式，不建议在中断服务函数中调用；
  *         3. 超长延时（>1秒）请使用 HAL_Delay 或 RTOS 任务睡眠。
  */
void delay_us(uint32_t nus);

/**
  * @brief  毫秒级延时（阻塞式）
  * @param  nms  要延时的毫秒数，范围 0 ~ 65535
  * @retval None
  * @note   内部调用 delay_us 实现，同样为阻塞式
  */
void delay_ms(uint16_t nms);

#ifdef __cplusplus
}
#endif

#endif /* __API_DELAY_H__ */

/*----------------------------- End of file -------------------------------*/
