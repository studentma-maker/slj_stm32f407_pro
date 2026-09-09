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
 * 每路电机占用 10 个连续寄存器（基址 = 电机序号*10），字段顺序与地址
 * 均与旧板 slj_stm32f407 完全一致：
 *   +0 AM  - 报警信号（只读）
 *   +1 DR  - 方向控制（读写，非步数模式下写入触发S曲线换向）
 *   +2 ACC - 最大加速度 Hz/s（读写，所有模式下均可更新）
 *   +3 JRK - Jerk Hz/s²（读写，S曲线加加速度，写0恢复默认值）
 *   +4 STEP- 步数控制目标步数（写入触发步数控制模式；须在 PU 之前写入，
 *            多寄存器连续写入时先触发步数模式，PU 随后按"仅更新最大频率"处理）
 *   +5 PU  - 目标频率 / 步数模式最大脉冲频率 Hz（非步数模式写入触发S曲线渐变）
 *   +6 SP  - 实时速度（读=当前Hz；写0=急停，写N>1=直接跳变）
 *   +7~9 — - 预留
 *
 * 未移植 EN（驱动使能）/ CS（当前步数）两个字段：
 *   - EN：不是所有电机都需要通过 Modbus 控制驱动器使能，暂不在寄存器表中
 *     暴露；SMD_EN/SMD_EN_READ 宏仍保留在 main.h，硬件层面按 GPIO 复位后的
 *     默认电平工作，如后续确认驱动器使能极性，可再决定是否需要固件主动置位。
 *   - CS：并非所有电机都支持步数控制，因此不作为逐路通用字段。仅
 *     MOTOR_FBack / MOTOR_UpDown / MOTOR_GripperMove 三路支持步数控制的
 *     电机，各自使用扩展状态区的专用寄存器回读当前步数（见下方
 *     GRIPPER_CUR_STEPS / UPDOWN_CUR_STEPS / FBACK_CUR_STEPS，地址与旧板一致）。
 *------------------------------------------------------------------------*/
#define SMD_1_AM_ADDR              0        /* 报警信号 */
#define SMD_1_DR_ADDR              1        /* 方向控制 */
#define SMD_1_ACC_ADDR             2        /* 加速度控制 */
#define SMD_1_JRK_ADDR             3        /* Jerk控制 */
#define SMD_1_STEP_ADDR            4        /* 步数控制 */
#define SMD_1_PU_ADDR              5        /* 频率控制 */
#define SMD_1_SP_ADDR              6        /* 实时速度控制 */

#define SMD_2_AM_ADDR              10       /* 报警信号 */
#define SMD_2_DR_ADDR              11       /* 方向控制 */
#define SMD_2_ACC_ADDR             12       /* 加速度控制 */
#define SMD_2_JRK_ADDR             13       /* Jerk控制 */
#define SMD_2_STEP_ADDR            14       /* 步数控制 */
#define SMD_2_PU_ADDR              15       /* 频率控制 */
#define SMD_2_SP_ADDR              16       /* 实时速度控制 */

#define SMD_3_AM_ADDR              20       /* 报警信号 */
#define SMD_3_DR_ADDR              21       /* 方向控制 */
#define SMD_3_ACC_ADDR             22       /* 加速度控制 */
#define SMD_3_JRK_ADDR             23       /* Jerk控制 */
#define SMD_3_STEP_ADDR            24       /* 步数控制 */
#define SMD_3_PU_ADDR              25       /* 频率控制 */
#define SMD_3_SP_ADDR              26       /* 实时速度控制 */

#define SMD_4_AM_ADDR              30       /* 报警信号 */
#define SMD_4_DR_ADDR              31       /* 方向控制 */
#define SMD_4_ACC_ADDR             32       /* 加速度控制 */
#define SMD_4_JRK_ADDR             33       /* Jerk控制 */
#define SMD_4_STEP_ADDR            34       /* 步数控制 */
#define SMD_4_PU_ADDR              35       /* 频率控制 */
#define SMD_4_SP_ADDR              36       /* 实时速度控制 */

#define SMD_5_AM_ADDR              40       /* 报警信号 */
#define SMD_5_DR_ADDR              41       /* 方向控制 */
#define SMD_5_ACC_ADDR             42       /* 加速度控制 */
#define SMD_5_JRK_ADDR             43       /* Jerk控制 */
#define SMD_5_STEP_ADDR            44       /* 步数控制 */
#define SMD_5_PU_ADDR              45       /* 频率控制 */
#define SMD_5_SP_ADDR              46       /* 实时速度控制 */

#define SMD_6_AM_ADDR              50       /* 报警信号 */
#define SMD_6_DR_ADDR              51       /* 方向控制 */
#define SMD_6_ACC_ADDR             52       /* 加速度控制 */
#define SMD_6_JRK_ADDR             53       /* Jerk控制 */
#define SMD_6_STEP_ADDR            54       /* 步数控制 */
#define SMD_6_PU_ADDR              55       /* 频率控制 */
#define SMD_6_SP_ADDR              56       /* 实时速度控制 */

#define OUT_1_ADDR                 80      /* 输出信号，地址与旧板 slj_stm32f407 一致 */
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

/* 92~99 预留（旧板此区间为 EC_CLEAR/EC/ADC，新板未实现该功能，暂不使用） */

/* M1/M2 直流电机调速：旧板无此功能，没有对应的旧地址，暂沿用原有取值 */
#define MOTOR_1_TARGET_SP_ADDR     100     /* 电机1目标速度（可读可写） */
#define MOTOR_1_CURRENT_SP_ADDR    101     /* 电机1当前速度（只读） */
#define MOTOR_2_TARGET_SP_ADDR     102     /* 电机2目标速度（可读可写） */
#define MOTOR_2_CURRENT_SP_ADDR    103     /* 电机2当前速度（只读） */

/* 104~111 预留 */

#define IN_1_ADDR                  112      /* 输入信号，地址与旧板 slj_stm32f407 一致 */
#define IN_2_ADDR                  113      /* 输入信号 */
#define IN_3_ADDR                  114      /* 输入信号 */
#define IN_4_ADDR                  115      /* 输入信号 */
#define IN_5_ADDR                  116      /* 输入信号 */
#define IN_6_ADDR                  117      /* 输入信号 */
#define IN_7_ADDR                  118      /* 输入信号 */
#define IN_8_ADDR                  119      /* 输入信号 */
#define IN_9_ADDR                  120      /* 输入信号 */
#define IN_10_ADDR                 121      /* 输入信号 */
#define IN_11_ADDR                 122      /* 输入信号 */
#define IN_12_ADDR                 123      /* 输入信号 */
#define IN_13_ADDR                 124      /* 输入信号 */
#define IN_14_ADDR                 125      /* 输入信号 */
#define IN_15_ADDR                 126      /* 输入信号 */
#define IN_16_ADDR                 127      /* 输入信号 */
#define IN_17_ADDR                 128      /* 输入信号 */
#define IN_18_ADDR                 129      /* 输入信号 */
#define IN_19_ADDR                 130      /* 输入信号 */
#define IN_20_ADDR                 131      /* 输入信号 */

/* 步数只读回读：旧板同样只为夹爪电机、进退电机两路开了专用寄存器
 * （升降电机在旧板上也没有当前步数回读寄存器），地址与旧板完全一致 */
#define GRIPPER_CUR_STEPS_ADDR     132     /* 夹爪电机当前步数（只读） */
#define SYS_TO_ORIGIN_ADDR         133     /* 系统上电回原点状态（只读，对应 GrippertoOrigin_P 枚举） */
#define FBACK_CUR_STEPS_ADDR       134     /* 进退电机当前步数（只读） */

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
