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
