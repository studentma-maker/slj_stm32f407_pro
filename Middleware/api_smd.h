/**
  * @file    api_smd.h
  * @brief   SMD API 接口头文件
  * @details 本文件声明了上层应用使用的 SMD 相关接口函数及类型定义。
  * @version V1.0.0
  * @date    22-Aug-2026
  */
#ifndef __API_SMD_H__
#define __API_SMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 导入标准库及 HAL 库头文件 ------------------------------------------------*/
#include "main.h"
/* 导出常量定义 ------------------------------------------------------------*/

/**
 * @brief PWM输出频率范围限定
 * @note  硬件/应用层约束，禁止超出此范围配置
 */
#define SMD_PWM_FREQ_MIN    1u          // PWM输出频率下限（1Hz）
#define SMD_PWM_FREQ_MAX    4000u       // PWM输出频率上限（4000Hz）

/**
 * @brief 频率渐变速度范围限定
 * @note  渐变速度单位为Hz/s，禁止超出此范围配置
 */
#define SMD_GRADIENT_MIN    1u          // 渐变速度下限（1Hz/s）
#define SMD_GRADIENT_MAX    1000u       // 渐变速度上限（1000Hz/s）

/**
 * @brief PWM计数器基准频率
 * @note  固定为1MHz（1us分辨率），便于占空比和频率计算
 */
#define SMD_COUNTER_FREQ    50000u   

/* 导出类型定义 ------------------------------------------------------------*/

/**
 * @brief 单个PWM通道的频率渐变控制结构体
 * @note  核心设计：
 *        1. 所有计算/判断使用浮点型变量，仅在更新PWM硬件时转换为整型
 *        2. 浮点型变量保存实时值，保证渐变精度不丢失
 *        3. 整型变量仅用于硬件配置，不参与渐变计算
 */
typedef struct {
    float target_freq_float;    // 目标频率（浮点型，1.0~4000.0Hz）
    float current_freq_float;   // 当前频率（浮点型，核心：保存实时值用于下次计算）
    float step_per_ms;          // 每ms频率变化步长（浮点型，Hz/ms）
    uint8_t is_running;         // 渐变状态标记 (0:停止, 1:运行中)
    uint32_t current_freq_int;  // 当前频率（整型，仅用于PWM硬件配置）
} SMD_Freq_Gradient;

/**
 * @brief 8路PWM通道枚举
 * @note  每个通道对应固定的GPIO口和定时器通道，不可随意修改
 */
typedef enum {
    SMD_CH0 = 0,  // PA7 - TIM14_CH1 (普通PWM通道)
    SMD_CH1 = 1,  // PA6 - TIM13_CH1 (普通PWM通道)
    SMD_CH2 = 2,  // PA5 - TIM8_CH1N (高级定时器互补通道)
    SMD_CH3 = 3,  // PA3 - TIM5_CH4 (普通PWM通道)
    SMD_CH4 = 4,  // PA2 - TIM9_CH1 (普通PWM通道)
    SMD_CH5 = 5,  // PA1 - TIM2_CH2 (普通PWM通道)
    SMD_CH_MAX    // 通道总数（仅用于遍历，不可作为实际通道使用）
} SMD_Channel;


extern SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX];
extern uint16_t SMD_PU_DATA[SMD_CH_MAX];
extern uint16_t SMD_ACC_DATA[SMD_CH_MAX];


/* 导出函数声明 ------------------------------------------------------------*/

/**
 * @brief  设置指定通道的频率渐变（平滑变到目标频率）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  target_freq: 目标频率（1~4000Hz）
 * @param  speed: 渐变速度（1~1000Hz/s）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=参数非法
 * @note   1. 速度单位为Hz/s，函数内转换为Hz/ms（浮点型）
 *         2. 自动判断升/降频方向，设置step_per_ms正负
 *         3. 启动渐变前自动停止当前渐变，避免叠加
 */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t speed);

/**
 * @brief  停止指定通道的频率渐变
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @note   1. 停止后current_freq_float保留当前浮点值
 *         2. is_running置0，step_per_ms置0
 */
void SMD_PWM_StopFreqGradient(SMD_Channel ch);

#ifdef __cplusplus
}
#endif

#endif /* __API_SMD_H__ */

/*----------------------------- End of file -------------------------------*/
