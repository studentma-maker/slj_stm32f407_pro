/**
  * @file    mb_slave.h
  * @brief   modbus 从机程序接口头文件
  * @details 定义 modbus 从机通信结构体、状态机及相关接口函数。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_SLAVE_H
#define _MB_SLAVE_H

#include "main.h"
#include "mb_port.h"

/* 参数定义 ----------------------------------------------------------------*/
#define MBS_RTU_MIN_SIZE	4
#define MBS_RTU_MAX_SIZE	255	/* 最大不超过255 */
#define MBS_ERR_MAX_TIMES	3
#define MBS_REC_TIMEOUT		100  /* 单位3.5T */

#define REG_HOLDING_NREGS   150 /* 保持寄存器数量 */
#define REG_COILS_SIZE      16 /* 线圈数量 */

/* 从机结构体定义 ----------------------------------------------------------*/
typedef struct
{
    uint8_t state;						/* modbus 状态 */
    uint8_t errTimes;  					/* 失败次数计数 */
    uint8_t txLen;     					/* 需要发送的帧长度 */
    uint8_t txCounter;					/* 已发送bytes计数 */
    uint8_t txBuf[MBS_RTU_MAX_SIZE];	/* 发送缓冲区 */
    uint8_t rxCounter;					/* 接收计数 */
    uint8_t rxBuf[MBS_RTU_MAX_SIZE];	/* 接收缓冲区 */
    uint8_t rxTimeOut;					/* 接收时的超时计数 */
    uint8_t rspCode;                    /* 应答码 */
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
} mbs;

/* 异常枚举定义 ------------------------------------------------------------*/
typedef enum
{
    RSP_OK = 0,         /* 成功 */
    RSP_ERR_CMD,        /* 不支持的功能码 */
    RSP_ERR_REG_ADDR,   /* 寄存器地址错误 */
    RSP_ERR_VALUE,      /* 数据值域错误 */
    RSP_ERR_WRITE       /* 写入失败 */

} mb_slave_rsp;

/* 状态枚举定义 ------------------------------------------------------------*/
typedef enum
{
    MBS_STATE_IDLE = 0X00,
    MBS_STATE_TX,
    MBS_STATE_TX_END,
    MBS_STATE_RX,
    MBS_STATE_RX_CHECK,
    MBS_STATE_EXEC,
    MBS_STATE_REC_ERR,		/* 接收错误状态 */
    MBS_STATE_TIMES_ERR,	/* 传输 */

} mb_slave_state;

extern mbs mbsUSB, mbsHmi;

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  modbus 从机初始化
  * @param  _mbs  通信参数
  * @retval None
  */
void mbs_init(mbs *_mbs);

/**
  * @brief  modbus 从机状态轮询
  * @param  _mbs  通信参数
  * @retval None
  */
void mbs_poll(mbs *_mbs);

/**
  * @brief  modbus 从机定时器中断处理
  * @param  _mbs  通信参数
  * @retval None
  */
void mbs_timer3T5Isr(mbs *_mbs);

/**
  * @brief  modbus 从机串口接收中断处理
  * @param  _mbs  通信参数
  * @retval None
  */
void mbs_uartRxIsr(mbs *_mbs);

/**
  * @brief  modbus 从机串口发送中断处理
  * @param  _mbs  通信参数
  * @retval None
  */
void mbs_uartTxIsr(mbs *_mbs);

#endif

/*----------------------------- End of file -------------------------------*/
