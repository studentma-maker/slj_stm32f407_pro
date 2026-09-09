/**
  * @file    mb_host.c
  * @brief   modbus 主机程序实现文件
  * @details 实现 modbus 主机通信状态机及相关中断处理函数。
  * @version V1.0.0
  * @date    20-Jun-2025
  */

#include "mb_host.h"
#include "mb_port.h"
#include "mb_crc.h"
#include "string.h"
#include "usart.h"
#include "tim.h"

/* 私有变量定义 ------------------------------------------------------------*/
mbh mbhMCU;

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  modbus 主机初始化
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_init(mbh *_mbh)
{
    if(_mbh == &mbhMCU)
    {
        _mbh->huart = HUART_MCU;
        _mbh->USART = USART_MCU;
        _mbh->htim = HTIM_MCU;
        _mbh->TIM = TIM_MCU;
    }

    mb_port_uartInit(&_mbh->huart, _mbh->USART, _mbh->baudRate, _mbh->parity);
    mb_port_timerInit(&_mbh->htim, _mbh->TIM, _mbh->baudRate);

    _mbh->putchar = mb_port_putchar;
    _mbh->getchar = mb_port_getchar;
    _mbh->uartEnable = mb_port_uartEnable;
    _mbh->timerEnable = mb_port_timerEnable;
    _mbh->timerDisable = mb_port_timerDisable;
}

/**
  * @brief  modbus 主机给从机发送一条命令
  * @param  _mbh      通信参数
  * @param  add       从机地址
  * @param  cmd       功能码
  * @param  data      要发送的数据
  * @param  data_len  发送的数据长度
  * @retval -1:发送失败 0:发送成功
  */
int8_t mbh_send(mbh *_mbh, uint8_t add, uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    uint16_t crc;

    if(_mbh->state != MBH_STATE_IDLE)return -1; /* busy state */

    _mbh->txCounter = 0;
    _mbh->rxCounter = 0;
    _mbh->txBuf[0] = add;
    _mbh->txBuf[1] = cmd;
    memcpy((_mbh->txBuf + 2), data, data_len);
    _mbh->txLen = data_len + 2; /* data(n)+add(1)+cmd(1) */
    crc = mb_crc16(_mbh->txBuf, _mbh->txLen);
    _mbh->txBuf[_mbh->txLen++] = (uint8_t)(crc & 0xff);
    _mbh->txBuf[_mbh->txLen++] = (uint8_t)(crc >> 8);

    _mbh->state = MBH_STATE_TX;
    mb_port_uartEnable(&_mbh->huart, 1, 0); /* enable tx, disable rx */

    return 0;
}

/**
  * @brief  获取 modbus 主机运行状态
  * @param  _mbh  通信参数
  * @retval mb_host_state 中状态
  */
uint8_t mbh_getState(mbh *_mbh)
{
    return _mbh->state;
}

/**
  * @brief  modbus 主机回调函数处理
  * @param  pframe  接收帧数据
  * @param  len     帧长度
  * @retval None
  */
void mbh_exec(uint8_t *pframe, uint8_t len)
{
    uint8_t datalen = len - 2;

    switch(pframe[1])
    {
        case 1:
            mbh_hook_rec01(pframe[0], (pframe + 3), datalen);
            break;

        case 2:
            mbh_hook_rec02(pframe[0], (pframe + 3), datalen);
            break;

        case 3:
            mbh_hook_rec03(pframe[0], (pframe + 3), datalen);
            break;

        case 4:
            mbh_hook_rec04(pframe[0], (pframe + 3), datalen);
            break;

        case 5:
            mbh_hook_rec05(pframe[0], (pframe + 3), datalen);
            break;

        case 6:
            mbh_hook_rec06(pframe[0], (pframe + 3), datalen);
            break;

        case 15:
            mbh_hook_rec15(pframe[0], (pframe + 3), datalen);
            break;

        case 16:
            mbh_hook_rec16(pframe[0], (pframe + 3), datalen);
            break;
    }
}

/**
  * @brief  modbus 主机状态轮询
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_poll(mbh *_mbh)
{
    switch(_mbh->state)
    {
        /* 接收完一帧数据，开始进行校验 */
        case MBH_STATE_RX_CHECK:
            if((_mbh->rxCounter >= MBH_RTU_MIN_SIZE) && (mb_crc16(_mbh->rxBuf, _mbh->rxCounter) == 0))
            {
                if((_mbh->txBuf[0] == _mbh->rxBuf[0]) && (_mbh->txBuf[1] == _mbh->rxBuf[1]))
                {
                    _mbh->state = MBH_STATE_EXEC;
                }
                else _mbh->state = MBH_STATE_REC_ERR;

            }
            else _mbh->state = MBH_STATE_REC_ERR;

            break;

        /* 接收一帧数据出错 */
        case MBH_STATE_REC_ERR:
            _mbh->errTimes++;

            if(_mbh->errTimes >= MBH_ERR_MAX_TIMES)
            {
                _mbh->state = MBH_STATE_TIMES_ERR;
            }
            else  /* 重新再启动一次传输 */
            {
                _mbh->txCounter = 0;
                _mbh->rxCounter = 0;
                _mbh->state = MBH_STATE_TX;
                mb_port_uartEnable(&_mbh->huart, 1, 0); /* enable tx,disable rx */
            }

            break;

        /* 超过最大错误传输次数 */
        case MBH_STATE_TIMES_ERR:
            mbh_hook_timesErr(_mbh->rxBuf[0], _mbh->rxBuf[1], _mbh->rxBuf[2]);
            _mbh->state = MBH_STATE_IDLE;
            _mbh->txCounter = 0;
            _mbh->rxCounter = 0;
            break;

        /* 确定接收正确执行回调 */
        case MBH_STATE_EXEC:
            mbh_exec(_mbh->rxBuf, _mbh->rxCounter);
            _mbh->state = MBH_STATE_IDLE;
            break;

    }
}

/**
  * @brief  modbus 主机定时器中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_timer3T5Isr(mbh *_mbh)
{
    switch(_mbh->state)
    {
        /* 发送完但没有接收到数据 */
        case MBH_STATE_TX_END:
            _mbh->rxTimeOut++;

            if(_mbh->rxTimeOut >= MBH_REC_TIMEOUT) /* 接收超时 */
            {
                _mbh->rxTimeOut = 0;
                _mbh->state = MBH_STATE_REC_ERR;
                mb_port_timerDisable(&_mbh->htim);		/* 关闭定时器 */
                mb_port_uartEnable(&_mbh->huart, 0, 0); 	/* 串口tx、rx都关闭 */
            }

            break;

        case MBH_STATE_RX:     	/* 3.5T到,接收一帧完成 */
            _mbh->state = MBH_STATE_RX_CHECK;
            mb_port_timerDisable(&_mbh->htim);		/* 关闭定时器 */
            mb_port_uartEnable(&_mbh->huart, 0, 0); 	/* 串口tx、rx都关闭 */
            break;
    }
}

/**
  * @brief  modbus 主机串口接收中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_uartRxIsr(mbh *_mbh)
{
    uint8_t ch;
    mb_port_getchar(&_mbh->huart, &ch);

    switch(_mbh->state)
    {
        case MBH_STATE_TX_END:
            _mbh->rxCounter = 0;
            _mbh->rxBuf[_mbh->rxCounter++] = ch;
            _mbh->state = MBH_STATE_RX;
            mb_port_timerEnable(&_mbh->htim);
            break;

        case MBH_STATE_RX:
            if(_mbh->rxCounter < MBH_RTU_MAX_SIZE)
            {
                _mbh->rxBuf[_mbh->rxCounter++] = ch;
            }

            mb_port_timerEnable(&_mbh->htim);
            break;

        default:
            mb_port_timerEnable(&_mbh->htim);
            break;
    }
}

/**
  * @brief  modbus 主机串口发送中断处理
  * @param  _mbh  通信参数
  * @retval None
  */
void mbh_uartTxIsr(mbh *_mbh)
{
    switch (_mbh->state)
    {
        case MBH_STATE_TX:
            if(_mbh->txCounter == _mbh->txLen)      /* 全部发送完 */
            {
                _mbh->state = MBH_STATE_TX_END;
                mb_port_uartEnable(&_mbh->huart, 0, 1); /* disable tx,enable rx */
                _mbh->rxTimeOut = 0;                    /* 清除接收超时计数 */
                mb_port_timerEnable(&_mbh->htim);       /* 打开定时器 */
            }
            else
            {
                mb_port_putchar(&_mbh->huart, _mbh->txBuf[_mbh->txCounter++]);
            }

            break;

        case MBH_STATE_TX_END:
            mb_port_uartEnable(&_mbh->huart, 0, 1);  /* disable tx,enable rx */
            break;
    }
}

/*----------------------------- End of file -------------------------------*/
