/**
  * @file    mb_host.h
  * @brief   modbus 主机程序接口头文件
  * @details 定义 modbus 主机通信结构体、状态机及相关接口函数。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_HOST_H
#define _MB_HOST_H

#include "main.h"
#include "mb_port.h"
#include "mb_hook.h"

/* 参数定义 ----------------------------------------------------------------*/
#define MBH_RTU_MIN_SIZE	4
#define MBH_RTU_MAX_SIZE	255	/* 最大不超过255 */
#define MBH_ERR_MAX_TIMES	3
#define MBH_REC_TIMEOUT		100  /* 单位3.5T */

/* 主机结构体定义 ----------------------------------------------------------*/
typedef struct
{
    uint8_t state;						/* modbus 状态 */
    uint8_t errTimes;  					/* 失败次数计数 */
    uint8_t txLen;     					/* 需要发送的帧长度 */
    uint8_t txCounter;					/* 已发送bytes计数 */
    uint8_t txBuf[MBH_RTU_MAX_SIZE];	/* 发送缓冲区 */
    uint8_t rxCounter;					/* 接收计数 */
    uint8_t rxBuf[MBH_RTU_MAX_SIZE];	/* 接收缓冲区 */
    uint8_t rxTimeOut;					/* 接收时的超时计数 */

    uint8_t regCoilsBuf[REG_COILS_SIZE];
    uint16_t regHoldingBuf[REG_HOLDING_NREGS];

    uint8_t slaveAddr;                          /* 地址码 */
    uint8_t parity;                             /* 校验位 */
    uint32_t baudRate;                          /* 波特率 */

    UART_HandleTypeDef huart;
    USART_TypeDef *USART;

    TIM_HandleTypeDef htim;
    TIM_TypeDef *TIM;

    void (*putchar)(UART_HandleTypeDef *, uint8_t);                   /* 发送数据函数 */
    void (*getchar)(UART_HandleTypeDef *, uint8_t *);                 /* 接收数据函数 */
    void (*uartEnable)(UART_HandleTypeDef *, uint8_t, uint8_t);       /* 发送接收使能函数 */
    void (*timerEnable)(TIM_HandleTypeDef *);                  /* 定时器开启 */
    void (*timerDisable)(TIM_HandleTypeDef *);                 /* 定时器关闭 */
} mbh;

extern mbh mbhMCU;

/* 状态枚举定义 ------------------------------------------------------------*/
typedef enum
{
    MBH_STATE_IDLE = 0X00,
    MBH_STATE_TX,
    MBH_STATE_TX_END,
    MBH_STATE_RX,
    MBH_STATE_RX_CHECK,
    MBH_STATE_EXEC,
    MBH_STATE_REC_ERR,		/* 接收错误状态 */
    MBH_STATE_TIMES_ERR,	/* 传输 */

} mb_host_state;

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  modbus 主机初始化
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_init(mbh *_mbh);

/**
  * @brief  modbus 主机给从机发送一条命令
  * @param  _mbh      通信参数
  * @param  add       从机地址
  * @param  cmd       功能码
  * @param  data      要发送的数据
  * @param  data_len  发送的数据长度
  * @retval -1:发送失败 0:发送成功
  */
int8_t mbh_send(mbh *_mbh, uint8_t add, uint8_t cmd, uint8_t *data, uint8_t data_len);

/**
  * @brief  获取 modbus 主机运行状态
  * @param  _mbh  通信参数
  * @retval mb_host_state 中状态
  */
uint8_t mbh_getState(mbh *_mbh);

/**
  * @brief  modbus 主机状态轮询
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_poll(mbh *_mbh);

/**
  * @brief  modbus 主机定时器中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_timer3T5Isr(mbh *_mbh);

/**
  * @brief  modbus 主机串口接收中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_uartRxIsr(mbh *_mbh);

/**
  * @brief  modbus 主机串口发送中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_uartTxIsr(mbh *_mbh);

#endif

/*----------------------------- End of file -------------------------------*/
