/**
  * @file    api_motor_control.h
  * @brief   电机驱动 API 头文件
  * @details 基于单个定时器4通道 PWM，控制两路直流电机。
  *          M1: CH1(OP) + CH2(ON)    M2: CH3(OP) + CH4(ON)
  *          支持平滑加减速，通过 BSP 定时器 10ms 实现非阻塞控制。
  * @version V1.0.0
  * @date    27-Aug-2026
  */
#ifndef __API_MOTOR_CONTROL_H__
#define __API_MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 导入 HAL 库 ------------------------------------------------------------*/
#include "main.h"

/* 导入 BSP 定时器 ------------------------------------------------------------*/
#include "api_timer.h"

/* 导出常量定义 ------------------------------------------------------------*/

/**
  * @brief 加减速步进配置
  * @note  每次调整的百分比步长（值越小加减速越平滑）
  */
extern uint8_t g_motor_step_percent;


/* 导出类型定义 ------------------------------------------------------------*/

/**
  * @brief 电机编号
  */
typedef enum
{
    API_MOTOR_1 = 0,  /*!< 电机1，使用 CH1(OP) + CH2(ON) */
    API_MOTOR_2 = 1   /*!< 电机2，使用 CH3(OP) + CH4(ON) */
} API_MOTOR_Num_t;

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  电机驱动初始化
  * @param  htim  指向已初始化的定时器句柄（PWM 已配置并启动）
  * @param  frequency  目标 PWM 频率（Hz）
  * @retval None
  */
void API_MOTOR_Init(TIM_HandleTypeDef *htim, uint32_t frequency);

/**
  * @brief  配置 PWM 输出频率
  * @param  frequency  目标 PWM 频率（Hz）
  * @retval None
  */
void API_MOTOR_ConfigPWMFrequency(uint32_t frequency);

/**
  * @brief  设置电机目标速度（非阻塞模式）
  * @param  motor   电机编号（API_MOTOR_1 或 API_MOTOR_2）
  * @param  speed   目标速度百分比，范围 -100 ~ 100
  *                 正数：正转  负数：反转   0：停止
  * @retval None
  * @note   该函数立即返回，速度变换在 BSP 定时器 10ms 中执行
  */
void API_MOTOR_SetSpeed(API_MOTOR_Num_t motor, int8_t speed);

/**
  * @brief  通过通信接口设置电机速度
  * @param  motor   电机编号
  * @param  speed   目标速度百分比
  * @retval None
  * @note   通过通信接口调用，速度变换在 BSP 定时器 10ms 中执行
  */
void API_MOTOR_SetSpeedViaComm(API_MOTOR_Num_t motor, int8_t speed);

/**
  * @brief  通过通信接口设置加减速步进参数
  * @param  step   步进百分比（1-10）
  * @retval None
  */
void API_MOTOR_SetStepViaComm(uint8_t step);

/**
  * @brief  立即停止电机（紧急刹车）
  * @param  motor   电机编号
  * @retval None
  * @note   两路同时输出低电平，无加减速过程，用于紧急情况
  */
void API_MOTOR_Stop(API_MOTOR_Num_t motor);

/**
  * @brief  获取当前实际速度
  * @param  motor   电机编号
  * @retval 当前速度百分比（-100 ~ 100）
  */
int8_t API_MOTOR_GetCurrentSpeed(API_MOTOR_Num_t motor);

/**
  * @brief  M1 推杆电机限位/急停保护，需每 1ms 调用一次
  * @retval None
  * @note   移植自旧板 slj_stm32f407 的 SMD_CheckRelayMotorLimit()：旧板通过
  *         RELAY_MOTOR_1/RELAY_MOTOR_2 两路继电器正反转驱动同一台推杆电机，
  *         新板已改为 M1 独立 PWM+MOS 桥驱动，故限位逻辑仅作用于 API_MOTOR_1，
  *         用当前速度符号代替旧板"读继电器方向"作为运行方向判断：
  *           IN8(索引7)/IN10(索引9)：正转方向限位
  *           IN9(索引8)：反转方向限位
  *           IN20(索引19)：全局急停
  *         IN 序号沿用旧板编号，需与实际接线核实。仅当电机正在运行
  *         （速度非0）时才检测，触发后立即调用 API_MOTOR_Stop(API_MOTOR_1)。
  *         M2 未接限位开关，不做限位保护。
  */
void API_MOTOR_CheckLimit(void);

/**
  * @brief  BSP 定时器 10ms 中的速度更新函数
  * @retval None
  * @note   在 main.c 的 BSP 定时器回调中调用
  */
void API_MOTOR_UpdateSpeed(void);

/**
  * @brief  获取当前加减速步进参数
  * @return 步进百分比
  */
uint8_t API_MOTOR_GetStepPercent(void);

/**
  * @brief  获取当前步进间隔参数
  * @return 步进间隔（毫秒）
  */
uint16_t API_MOTOR_GetStepInterval(void);

#ifdef __cplusplus
}
#endif

#endif /* __API_MOTOR_CONTROL_H__ */

/*----------------------------- End of file -------------------------------*/
