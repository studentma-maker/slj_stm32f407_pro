/**
  * @file    mb_port.h
  * @brief   modbus 驱动接口头文件
  * @details 定义 modbus 通信所需的串口/定时器接口及校验位枚举。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_PORTS_H
#define _MB_PORTS_H

#include "main.h"

/* 接口定义 ----------------------------------------------------------------*/
#define HUART_USB               huart1
#define USART_USB               USART1
#define USART_USB_IRQ           USART1_IRQn
#define USART_USB_IRQHandler    USART1_IRQHandler  /* 串口中断处理函数 */

#define HTIM_USB                htim6
#define TIM_USB                 TIM6
#define TIM_USB_IRQHandler      TIM6_DAC_IRQHandler    /* 定时器中断处理函数 */

#define HUART_MCU               huart5
#define USART_MCU               UART5
#define USART_MCU_IRQ           UART5_IRQn
#define USART_MCU_IRQHandler    UART5_IRQHandler  /* 串口中断处理函数 */

#define HTIM_MCU                htim7
#define TIM_MCU                 TIM7
#define TIM_MCU_IRQHandler      TIM7_IRQHandler    /* 定时器中断处理函数 */

/* 校验位枚举 --------------------------------------------------------------*/
typedef enum
{
    MB_PARITY_NONE = 0X00,	/* 无奇偶校验，两个停止位 */
    MB_PARITY_ODD, 			/* 奇校验 */
    MB_PARITY_EVEN			/* 偶校验 */
} mbParity;

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  modbus 串口初始化接口
  * @param  huart    UART句柄
  * @param  usart    串口类型
  * @param  baud     串口波特率
  * @param  parity   奇偶校验位设置
  * @retval None
  */
void mb_port_uartInit(UART_HandleTypeDef *huart, USART_TypeDef *usart, uint32_t baud, uint8_t parity);

/**
  * @brief  串口 TX\RX 使能
  * @param  huart  UART句柄
  * @param  txen   0-关闭tx中断 1-打开tx中断
  * @param  rxen   0-关闭rx中断 1-打开rx中断
  * @retval None
  */
void mb_port_uartEnable(UART_HandleTypeDef *huart, uint8_t txen, uint8_t rxen);

/**
  * @brief  串口发送一个 byte
  * @param  huart  UART句柄
  * @param  ch     要发送的 byte
  * @retval None
  */
void mb_port_putchar(UART_HandleTypeDef *huart, uint8_t ch);

/**
  * @brief  串口读取一个 byte
  * @param  huart  UART句柄
  * @param  ch     存放读取一个 byte 的指针
  * @retval None
  */
void mb_port_getchar(UART_HandleTypeDef *huart, uint8_t *ch);

/**
  * @brief  定时器初始化
  * @param  htim    Timer句柄
  * @param  tim     定时器类型
  * @param  baud    串口波特率，根据波特率生成 3.5T 的定时
  * @retval None
  */
void mb_port_timerInit(TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t baud);

/**
  * @brief  定时器使能
  * @param  htim  Timer句柄
  * @retval None
  */
void mb_port_timerEnable(TIM_HandleTypeDef *htim);

/**
  * @brief  定时器关闭
  * @param  htim  Timer句柄
  * @retval None
  */
void mb_port_timerDisable(TIM_HandleTypeDef *htim);

#endif // _MB_PORTS_H

/*----------------------------- End of file -------------------------------*/
