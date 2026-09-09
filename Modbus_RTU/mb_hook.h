/**
  * @file    mb_hook.h
  * @brief   modbus 主机/从机回调函数接口头文件
  * @details 声明 modbus 主机接收回调与从机寄存器读写钩子函数，
  *          并定义 modbus 寄存器地址映射。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_HOOK_H
#define _MB_HOOK_H

#include "main.h"
#include "mb_slave.h"

/* 值<->功能码：WRITE_HOLDING_V 保持不变，上位机批量写寄存器时用该值标记
 * "此寄存器保持不变"，下位机收到该值直接跳过，避免误改（沿用旧项目约定） */
#define WRITE_HOLDING_V            2

/* modbus 寄存器地址定义 ----------------------------------------------------*
 * 每路电机占用 10 个连续寄存器（基址 = 电机序号*10），字段顺序：
 *   +0 AM  - 报警信号（只读）
 *   +1 EN  - 使能控制（读写，直接透传到 SMD_EN 引脚）
 *   +2 DR  - 方向控制（读写，非步数模式下写入触发S曲线换向）
 *   +3 ACC - 最大加速度 Hz/s（读写，所有模式下均可更新）
 *   +4 JRK - Jerk Hz/s²（读写，S曲线加加速度，写0恢复默认值）
 *   +5 STEP- 步数控制目标步数（写入触发步数控制模式；须在 PU 之前写入，
 *            多寄存器连续写入时先触发步数模式，PU 随后按"仅更新最大频率"处理）
 *   +6 PU  - 目标频率 / 步数模式最大脉冲频率 Hz（非步数模式写入触发S曲线渐变）
 *   +7 CS  - 当前步数（只读）
 *   +8 SP  - 实时速度（读=当前Hz；写0=急停，写N>1=直接跳变）
 *   +9 —   - 预留
 *------------------------------------------------------------------------*/
#define SMD_1_AM_ADDR              0        /* 报警信号 */
#define SMD_1_EN_ADDR              1        /* 使能控制 */
#define SMD_1_DR_ADDR              2        /* 方向控制 */
#define SMD_1_ACC_ADDR             3        /* 加速度控制 */
#define SMD_1_JRK_ADDR             4        /* Jerk控制 */
#define SMD_1_STEP_ADDR            5        /* 步数控制 */
#define SMD_1_PU_ADDR              6        /* 频率控制 */
#define SMD_1_CS_ADDR              7        /* 当前步数 */
#define SMD_1_SP_ADDR              8        /* 实时速度控制 */

#define SMD_2_AM_ADDR              10       /* 报警信号 */
#define SMD_2_EN_ADDR              11       /* 使能控制 */
#define SMD_2_DR_ADDR              12       /* 方向控制 */
#define SMD_2_ACC_ADDR             13       /* 加速度控制 */
#define SMD_2_JRK_ADDR             14       /* Jerk控制 */
#define SMD_2_STEP_ADDR            15       /* 步数控制 */
#define SMD_2_PU_ADDR              16       /* 频率控制 */
#define SMD_2_CS_ADDR              17       /* 当前步数 */
#define SMD_2_SP_ADDR              18       /* 实时速度控制 */

#define SMD_3_AM_ADDR              20       /* 报警信号 */
#define SMD_3_EN_ADDR              21       /* 使能控制 */
#define SMD_3_DR_ADDR              22       /* 方向控制 */
#define SMD_3_ACC_ADDR             23       /* 加速度控制 */
#define SMD_3_JRK_ADDR             24       /* Jerk控制 */
#define SMD_3_STEP_ADDR            25       /* 步数控制 */
#define SMD_3_PU_ADDR              26       /* 频率控制 */
#define SMD_3_CS_ADDR              27       /* 当前步数 */
#define SMD_3_SP_ADDR              28       /* 实时速度控制 */

#define SMD_4_AM_ADDR              30       /* 报警信号 */
#define SMD_4_EN_ADDR              31       /* 使能控制 */
#define SMD_4_DR_ADDR              32       /* 方向控制 */
#define SMD_4_ACC_ADDR             33       /* 加速度控制 */
#define SMD_4_JRK_ADDR             34       /* Jerk控制 */
#define SMD_4_STEP_ADDR            35       /* 步数控制 */
#define SMD_4_PU_ADDR              36       /* 频率控制 */
#define SMD_4_CS_ADDR              37       /* 当前步数 */
#define SMD_4_SP_ADDR              38       /* 实时速度控制 */

#define SMD_5_AM_ADDR              40       /* 报警信号 */
#define SMD_5_EN_ADDR              41       /* 使能控制 */
#define SMD_5_DR_ADDR              42       /* 方向控制 */
#define SMD_5_ACC_ADDR             43       /* 加速度控制 */
#define SMD_5_JRK_ADDR             44       /* Jerk控制 */
#define SMD_5_STEP_ADDR            45       /* 步数控制 */
#define SMD_5_PU_ADDR              46       /* 频率控制 */
#define SMD_5_CS_ADDR              47       /* 当前步数 */
#define SMD_5_SP_ADDR              48       /* 实时速度控制 */

#define SMD_6_AM_ADDR              50       /* 报警信号 */
#define SMD_6_EN_ADDR              51       /* 使能控制 */
#define SMD_6_DR_ADDR              52       /* 方向控制 */
#define SMD_6_ACC_ADDR             53       /* 加速度控制 */
#define SMD_6_JRK_ADDR             54       /* Jerk控制 */
#define SMD_6_STEP_ADDR            55       /* 步数控制 */
#define SMD_6_PU_ADDR              56       /* 频率控制 */
#define SMD_6_CS_ADDR              57       /* 当前步数 */
#define SMD_6_SP_ADDR              58       /* 实时速度控制 */

#define IN_1_ADDR                  60       /* 输入信号 */
#define IN_2_ADDR                  61       /* 输入信号 */
#define IN_3_ADDR                  62       /* 输入信号 */
#define IN_4_ADDR                  63       /* 输入信号 */
#define IN_5_ADDR                  64       /* 输入信号 */
#define IN_6_ADDR                  65       /* 输入信号 */
#define IN_7_ADDR                  66       /* 输入信号 */
#define IN_8_ADDR                  67       /* 输入信号 */
#define IN_9_ADDR                  68       /* 输入信号 */
#define IN_10_ADDR                 69       /* 输入信号 */
#define IN_11_ADDR                 70       /* 输入信号 */
#define IN_12_ADDR                 71       /* 输入信号 */
#define IN_13_ADDR                 72       /* 输入信号 */
#define IN_14_ADDR                 73       /* 输入信号 */
#define IN_15_ADDR                 74       /* 输入信号 */
#define IN_16_ADDR                 75       /* 输入信号 */
#define IN_17_ADDR                 76       /* 输入信号 */
#define IN_18_ADDR                 77       /* 输入信号 */
#define IN_19_ADDR                 78       /* 输入信号 */
#define IN_20_ADDR                 79       /* 输入信号 */

#define OUT_1_ADDR                 80      /* 输出信号 */
#define OUT_2_ADDR                 81      /* 输出信号 */
#define OUT_3_ADDR                 82      /* 输出信号 */
#define OUT_4_ADDR                 83      /* 输出信号 */
#define OUT_5_ADDR                 84      /* 输出信号 */
#define OUT_6_ADDR                 85      /* 输出信号 */
#define OUT_7_ADDR                 86      /* 输出信号 */
#define OUT_8_ADDR                 87      /* 输出信号 */
#define OUT_9_ADDR                 88      /* 输出信号 */
#define OUT_10_ADDR                89      /* 输出信号 */
#define OUT_11_ADDR                90      /* 输出信号 */
#define OUT_12_ADDR                91      /* 输出信号 */

#define MOTOR_1_TARGET_SP_ADDR     100     /* 电机1目标速度（可读可写） */
#define MOTOR_1_CURRENT_SP_ADDR    101     /* 电机1当前速度（只读） */
#define MOTOR_2_TARGET_SP_ADDR     102     /* 电机2目标速度（可读可写） */
#define MOTOR_2_CURRENT_SP_ADDR    103     /* 电机2当前速度（只读） */

#define SYS_TO_ORIGIN_ADDR         104     /* 系统上电回原点状态（只读，对应 GrippertoOrigin_P 枚举） */

#define STOP_ALL_MOTOR_ADDR        (REG_HOLDING_NREGS - 1) /* 全部步进电机急停（写1触发） */

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  主机模式下接收到从机不同功能码回复的回调处理
  * @param  add      从机地址
  * @param  data     接收到的从机数据指针
  * @param  datalen  接收到的数据长度
  * @retval None
  */
void mbh_hook_rec01(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec02(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec03(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec04(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec05(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec06(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec15(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec16(uint8_t add, uint8_t *data, uint8_t datalen);

/**
  * @brief  主机读写从机超过最大错误次数回调
  * @param  add   从机地址
  * @param  cmd   功能码
  * @param  data  异常码
  * @retval None
  */
void mbh_hook_timesErr(uint8_t add, uint8_t cmd, uint8_t data);

/**
  * @brief  从机更新保持寄存器值
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_holding(mbs *_mbs);

/**
  * @brief  从机提取保持寄存器值
  * @param  _mbs  从机结构体
  * @param  _reg  寄存器地址
  * @param  _val  寄存器值
  * @retval None
  */
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val);

/**
  * @brief  从机更新线圈状态
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_coils(mbs *_mbs);

/**
  * @brief  从机提取线圈状态
  * @param  _mbs  从机结构体
  * @param  _reg  线圈地址
  * @param  _val  线圈值
  * @retval None
  */
void mbs_hook_extract_coils(mbs *_mbs, uint16_t _reg, uint8_t _val);

#endif

/*----------------------------- End of file -------------------------------*/
