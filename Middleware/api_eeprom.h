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
/* 舵机参数存储地址：4×uint16 = 8字节（依次为 初始角度/角度a/角度b/行程时长ms），
 * 接在通信参数区之后（api_eeprom.c 的 AT24CXX_MCU_PARITY_ADDR=15 之后），
 * 24C02 共256字节空间充足。由 api_smd.c 舵机模块（M6TOSERVO_ENABLE=1）读写 */
#define AT24Cxx_SERVO_PARAMS_ADDR   16

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
