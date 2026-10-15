/**
  * @file    api_eeprom.h
  * @brief   EEPROM API 接口头文件
  * @details 声明上层应用使用的 EEPROM 相关接口函数。
  * @version V1.0.0
  * @date    22-Aug-2026
  */
#ifndef __API_EEPROM_H__
#define __API_EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* EEPROM 存储地址定义 -------------------------------------------------------*/
/* 舵机参数存储地址：4×uint16 = 8字节（依次为 初始角度/角度a/角度b/保留），
 * 接在通信参数区之后（api_eeprom.c 的 AT24CXX_MCU_PARITY_ADDR=15 之后），
 * 24C02 共256字节空间充足。由 api_smd.c 舵机模块（M6TOSERVO_ENABLE=1）读写 */
#define AT24Cxx_SERVO_PARAMS_ADDR   16

/* 排发皮带一圈步数（uint32，4字节），接在舵机参数(16~23)之后，由 api_smd.c 的
 * 皮带零点模块读写：开机读取，手动标定（Modbus 198 写1）成功后写入。
 * 读到 0 或 0xFFFFFFFF（新芯片空白值）按未标定处理，使用默认值 BELT_CYCLE_STEPS_DEFAULT */
#define AT24Cxx_BELT_CYCLE_ADDR     24

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  从EEPROM读取双字数据
  * @param  readAddr  读取地址
  * @param  data      读取数据
  * @param  len       读取长度
  * @retval None
  */
void EEPROM_ReadData(uint16_t readAddr, uint8_t *data, uint16_t len);

/**
  * @brief  写入双字数据到EEPROM
  * @param  writeAddr  写入地址
  * @param  data       写入数据
  * @param  len        写入长度
  * @retval None
  */
void EEPROM_WriteData(uint16_t writeAddr, uint8_t *data, uint16_t len);

/**
  * @brief  EEPROM初始化
  * @param  None
  * @retval None
  */
void EEPROM_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __API_EEPROM_H__ */

/*----------------------------- End of file -------------------------------*/
