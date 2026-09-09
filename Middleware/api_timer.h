/**
  * @file    api_timer.h
  * @brief   定时器 API 接口头文件
  * @details 本文件声明了上层应用使用的定时器相关接口函数及类型定义。
  * @version V1.0.0
  * @date    21-Aug-2026
  */
#ifndef __API_TIMER_H__
#define __API_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 导入标准库及 HAL 库头文件 ------------------------------------------------*/
#include "main.h"
#include "stdbool.h"

/* 导出常量定义 ------------------------------------------------------------*/

/* 各标志位掩码（用于位操作） */
/* 各标志位掩码（用于位操作） */
#define BSP_TIMER_FLAG_100US        (1U << 0)   /**< 100us 标志掩码 */
#define BSP_TIMER_FLAG_200US        (1U << 1)   /**< 200us 标志掩码 */
#define BSP_TIMER_FLAG_500US        (1U << 2)   /**< 500us 标志掩码 */
#define BSP_TIMER_FLAG_1MS          (1U << 3)   /**< 1ms 标志掩码 */
#define BSP_TIMER_FLAG_2MS          (1U << 4)   /**< 2ms 标志掩码 */
#define BSP_TIMER_FLAG_5MS          (1U << 5)   /**< 5ms 标志掩码 */
#define BSP_TIMER_FLAG_10MS         (1U << 6)   /**< 10ms 标志掩码 */
#define BSP_TIMER_FLAG_100MS        (1U << 7)   /**< 100ms 标志掩码 */
#define BSP_TIMER_FLAG_1S           (1U << 8)   /**< 1s 标志掩码 */

/* 导出类型定义 ------------------------------------------------------------*/

/**
 * @brief 定时器周期标志位联合体
 * @note 每个 bit 对应一个周期标志，在定时器中断中自动置位
 */
typedef union {
    uint8_t byte;                           /**< 全部 8 位标志 */
    struct {
        uint16_t cycle_100us_flg : 1;       /**< 100us 周期标志 */
        uint16_t cycle_200us_flg : 1;       /**< 200us 周期标志 */
        uint16_t cycle_500us_flg : 1;       /**< 500us 周期标志 */
        uint16_t cycle_1ms_flg : 1;         /**< 1ms 周期标志 */
        uint16_t cycle_2ms_flg : 1;         /**< 2ms 周期标志 */
        uint16_t cycle_5ms_flg : 1;         /**< 5ms 周期标志 */
        uint16_t cycle_10ms_flg : 1;        /**< 10ms 周期标志 */
        uint16_t cycle_100ms_flg : 1;       /**< 100ms 周期标志 */
        uint16_t cycle_1s_flg : 1;          /**< 1s 周期标志 */
        uint16_t reserved : 7;              /**< 保留位 */
    } bit;
} BSP_TASK_TIMER_ST;

/* 导出函数声明 ------------------------------------------------------------*/

/**
 * @brief 获取定时器周期标志位
 * @return BSP_TASK_TIMER_ST 联合体（包含所有标志位）
 *
 * @note
 * - 读取后标志位不会自动清除
 * - 调用 bsp_timer_ClearFlag() 清除指定标志
 */
BSP_TASK_TIMER_ST bsp_timer_GetFlags(void);

/**
 * @brief 清除指定的周期标志位
 * @param flagMask 要清除的标志位掩码（可使用位或运算组合）
 *
 * @note
 * - 例如：bsp_timer_ClearFlag(BSP_TIMER_FLAG_1MS | BSP_TIMER_FLAG_5MS);
 * - 建议在读取标志并处理完任务后立即清除
 */
void bsp_timer_ClearFlag(uint8_t flagMask);

/**
 * @brief 清除所有周期标志位
 */
void bsp_timer_ClearAllFlags(void);

/**
 * @brief 检查指定标志位是否置位
 * @param flagMask 要检查的标志位掩码
 * @return true: 已置位, false: 未置位
 */
bool bsp_timer_IsFlagSet(uint8_t flagMask);

#ifdef __cplusplus
}
#endif

#endif /* __API_TIMER_H__ */

/*----------------------------- End of file -------------------------------*/
