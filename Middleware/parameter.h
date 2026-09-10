/**
  * @file    parameter.h
  * @brief   系统参数配置头文件
  * @details 定义 EEPROM 读写及通信参数相关常量。
  * @version V1.0.0
  * @date    22-Aug-2026
  */
#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* EEPROM 数据初始化开关, 0为禁止;1为开启 */
#define AT24Cxx_DATA_INIT    1

/* 日期信息标识 每次修改参数都需要修改日期信息
   否则修改的信息无法写入到存储器中*/
#define DATE_INFO            0x20260910

/* 默认通信参数 */
#define RS485_SLAVE_ADDR     1
#define RS485_BAUD_RATE      115200
#define RS485_PARITY         MB_PARITY_NONE

#ifdef __cplusplus
}
#endif

#endif /* __PARAMETER_H__ */

/*----------------------------- End of file -------------------------------*/
