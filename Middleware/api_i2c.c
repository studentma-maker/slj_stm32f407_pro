/**
  * @file    api_i2c.c
  * @brief   I2C API 接口源文件
  * @details 实现模拟 I2C 总线的起始/停止/应答/收发字节等函数。
  * @version V1.0.0
  * @date    22-Aug-2026
  */

#include "api_i2c.h"
#include "api_delay.h"

/* 私有宏定义 --------------------------------------------------------------*/
#define I2C_SCL_H()     HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET)
#define I2C_SCL_L()     HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET)

#define I2C_SDA_H()     HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET)
#define I2C_SDA_L()     HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET)

#define I2C_SDA_READ()  HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  模拟I2C起始信号
  * @param  None
  * @retval None
  */
void I2C_Start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    delay_us(4);
    I2C_SDA_L();
    delay_us(4);
    I2C_SCL_L();
}

/**
  * @brief  模拟I2C停止信号
  * @param  None
  * @retval None
  */
void I2C_Stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    delay_us(4);
    I2C_SDA_H();
}

/**
  * @brief  模拟I2C等待应答信号
  * @param  None
  * @retval result: 0表示正确应答, 1表示无器件响应
  */
uint8_t I2C_WaitAck(void)
{
    uint8_t errorCount = 0;

    I2C_SDA_H();
    delay_us(1);
    I2C_SCL_H();
    delay_us(1);

    /* 等待应答信号拉低 */
    while(I2C_SDA_READ())
    {
        errorCount++;

        if(errorCount > 250)
        {
            I2C_Stop();
            return 1;
        }
    }

    I2C_SCL_L();
    delay_us(1);

    return 0;
}

/**
  * @brief  模拟I2C应答信号
  * @param  None
  * @retval None
  */
void I2C_YAck(void)
{
    I2C_SCL_L();
    I2C_SDA_L();
    delay_us(2);
    I2C_SCL_H();
    delay_us(2);
    I2C_SCL_L();
    delay_us(2);
    I2C_SDA_H();
}

/**
  * @brief  模拟I2C非应答信号
  * @param  None
  * @retval None
  */
void I2C_NAck(void)
{
    I2C_SCL_L();
    I2C_SDA_H();
    delay_us(2);
    I2C_SCL_H();
    delay_us(2);
    I2C_SCL_L();
}

/**
  * @brief  模拟I2C发送一个字节
  * @param  data  发送的字节数据
  * @retval None
  */
void I2C_SendByte(uint8_t data)
{
    uint8_t i = 0;

    I2C_SCL_L();

    /* 循环发送8位数据 */
    for(i = 0; i < 8; i++)
    {
        /* 从高位开始发送 */
        if(data & 0x80)
        {
            I2C_SDA_H();
        }
        else
        {
            I2C_SDA_L();
        }

        delay_us(2);
        I2C_SCL_H();
        delay_us(2);
        I2C_SCL_L();
        data <<= 1;
        delay_us(2);
    }
}

/**
  * @brief  模拟I2C接收一个字节
  * @param  None
  * @retval 接收的字节数据
  */
uint8_t I2C_ReadByte(void)
{
    uint8_t i = 0;
    uint8_t data = 0;

    /* 循环读取8位数据 */
    for(i = 0; i < 8; i++)
    {
        I2C_SCL_L();
        delay_us(2);
        I2C_SCL_H();
        data <<= 1;

        /* 从低位开始读取 */
        if(I2C_SDA_READ())
        {
            data |= 0x01;
        }

        delay_us(1);
    }

    return data;
}

/*----------------------------- End of file -------------------------------*/
