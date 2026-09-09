/**
  * @file    api_i2c.h
  * @brief   I2C API 接口头文件
  * @details 声明模拟 I2C 总线的接口函数及常量定义。
  * @version V1.0.0
  * @date    22-Aug-2026
  */
#ifndef __API_I2C_H__
#define __API_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 导出常量定义 ------------------------------------------------------------*/
#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  模拟I2C起始信号
  * @param  None
  * @retval None
  */
void I2C_Start(void);

/**
  * @brief  模拟I2C停止信号
  * @param  None
  * @retval None
  */
void I2C_Stop(void);

/**
  * @brief  模拟I2C等待应答信号
  * @param  None
  * @retval result: 0表示正确应答, 1表示无器件响应
  */
uint8_t I2C_WaitAck(void);

/**
  * @brief  模拟I2C应答信号
  * @param  None
  * @retval None
  */
void I2C_YAck(void);

/**
  * @brief  模拟I2C非应答信号
  * @param  None
  * @retval None
  */
void I2C_NAck(void);

/**
  * @brief  模拟I2C发送一个字节
  * @param  data  发送的字节数据
  * @retval None
  */
void I2C_SendByte(uint8_t data);

/**
  * @brief  模拟I2C接收一个字节
  * @param  None
  * @retval 接收的字节数据
  */
uint8_t I2C_ReadByte(void);

#ifdef __cplusplus
}
#endif

#endif /* __API_I2C_H__ */

/*----------------------------- End of file -------------------------------*/
