/**
  * @file    mb_crc.h
  * @brief   modbus CRC16 校验接口头文件
  * @details 提供 modbus RTU 帧的 CRC16 校验接口。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_CRC16_H
#define _MB_CRC16_H

#include "main.h"

/**
  * @brief  modbus CRC16 校验
  * @param  pFrame  校验数据指针
  * @param  len     数据长度
  * @retval 校验结果
  */
uint16_t mb_crc16( uint8_t * pFrame, uint16_t len );

#endif // _MB_CRC16_H

/*----------------------------- End of file -------------------------------*/
