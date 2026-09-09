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

/* modbus 寄存器地址定义 ----------------------------------------------------*/
#define SMD_1_AM_ADDR              0        /* 报警信号 */
#define SMD_1_EN_ADDR              1        /* 使能控制 */
#define SMD_1_DR_ADDR              2        /* 方向控制 */
#define SMD_1_PU_ADDR              3        /* 频率控制 */
#define SMD_1_ACC_ADDR             4        /* 加速度控制 */
#define SMD_1_SP_ADDR              5        /* 实时速度控制 */

#define SMD_2_AM_ADDR              10       /* 报警信号 */
#define SMD_2_EN_ADDR              11       /* 使能控制 */
#define SMD_2_DR_ADDR              12       /* 方向控制 */
#define SMD_2_PU_ADDR              13       /* 频率控制 */
#define SMD_2_ACC_ADDR             14       /* 加速度控制 */
#define SMD_2_SP_ADDR              15       /* 实时速度控制 */

#define SMD_3_AM_ADDR              20       /* 报警信号 */
#define SMD_3_EN_ADDR              21       /* 使能控制 */
#define SMD_3_DR_ADDR              22       /* 方向控制 */
#define SMD_3_PU_ADDR              23       /* 频率控制 */
#define SMD_3_ACC_ADDR             24       /* 加速度控制 */
#define SMD_3_SP_ADDR              25       /* 实时速度控制 */

#define SMD_4_AM_ADDR              30       /* 报警信号 */
#define SMD_4_EN_ADDR              31       /* 使能控制 */
#define SMD_4_DR_ADDR              32       /* 方向控制 */
#define SMD_4_PU_ADDR              33       /* 频率控制 */
#define SMD_4_ACC_ADDR             34       /* 加速度控制 */
#define SMD_4_SP_ADDR              35       /* 实时速度控制 */

#define SMD_5_AM_ADDR              40       /* 报警信号 */
#define SMD_5_EN_ADDR              41       /* 使能控制 */
#define SMD_5_DR_ADDR              42       /* 方向控制 */
#define SMD_5_PU_ADDR              43       /* 频率控制 */
#define SMD_5_ACC_ADDR             44       /* 加速度控制 */
#define SMD_5_SP_ADDR              45       /* 实时速度控制 */

#define SMD_6_AM_ADDR              50       /* 报警信号 */
#define SMD_6_EN_ADDR              51       /* 使能控制 */
#define SMD_6_DR_ADDR              52       /* 方向控制 */
#define SMD_6_PU_ADDR              53       /* 频率控制 */
#define SMD_6_ACC_ADDR             54       /* 加速度控制 */
#define SMD_6_SP_ADDR              55       /* 实时速度控制 */

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
