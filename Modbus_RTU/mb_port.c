/**
  * @file    mb_port.c
  * @brief   modbus 驱动接口实现文件
  * @details 实现 modbus 串口/定时器底层驱动及中断服务函数。
  * @version V1.0.0
  * @date    20-Jun-2025
  */

#include "mb_port.h"
#include "mb_host.h"
#include "mb_slave.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  modbus 串口初始化接口
  * @param  huart    UART句柄
  * @param  usart    串口类型
  * @param  baud     串口波特率
  * @param  parity   奇偶校验位设置
  * @retval None
  */
void mb_port_uartInit(UART_HandleTypeDef *huart, USART_TypeDef *usart, uint32_t baud, uint8_t parity)
{
    /* 串口部分初始化 */
    huart->Instance = usart;
    huart->Init.BaudRate = baud;

    if(parity == MB_PARITY_ODD)
    {
        huart->Init.WordLength = UART_WORDLENGTH_9B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_ODD;
    }
    else if(parity == MB_PARITY_EVEN)
    {
        huart->Init.WordLength = UART_WORDLENGTH_9B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_EVEN;
    }
    else
    {
        huart->Init.WordLength = UART_WORDLENGTH_8B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_NONE;
    }

    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  串口 TX\RX 使能
  * @param  huart  UART句柄
  * @param  txen   0-关闭tx中断 1-打开tx中断
  * @param  rxen   0-关闭rx中断 1-打开rx中断
  * @retval None
  */
void mb_port_uartEnable(UART_HandleTypeDef *huart, uint8_t txen, uint8_t rxen)
{
    if(txen)
    {
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
    }

    if(rxen)
    {
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    }
}

/**
  * @brief  串口发送一个 byte
  * @param  huart  UART句柄
  * @param  ch     要发送的 byte
  * @retval None
  */
void mb_port_putchar(UART_HandleTypeDef *huart, uint8_t ch)
{
    huart->Instance->DR = ch;
}

/**
  * @brief  串口读取一个 byte
  * @param  huart  UART句柄
  * @param  ch     存放读取一个 byte 的指针
  * @retval None
  */
void mb_port_getchar(UART_HandleTypeDef *huart, uint8_t *ch)
{
    *ch = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
}

/**
  * @brief  定时器初始化
  * @param  htim    Timer句柄
  * @param  tim     定时器类型
  * @param  baud    串口波特率，根据波特率生成 3.5T 的定时
  * @retval None
  */
void mb_port_timerInit(TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t baud)
{
    /* 定时器部分初始化 */
    htim->Instance = tim;
    htim->Init.Prescaler = 48; /* 1us记一次数 */
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;

    if(baud > 19200) /* 波特率大于19200固定使用1800作为3.5T */
    {
        htim->Init.Period = 1800;
    }
    else   /* 其他波特率的需要根据计算 */
    {
        /*	us=1s/(baud/11)*1000000*3.5
        *			=(11*1000000*3.5)/baud
        *			=38500000/baud
        */
        htim->Init.Period = (uint32_t)38500000 / baud;

    }

    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  定时器使能
  * @param  htim  Timer句柄
  * @retval None
  */
void mb_port_timerEnable(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);               /* 清除中断位 */
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);               /* 使能中断位 */
    __HAL_TIM_SET_COUNTER(htim, 0);                          /* 设置定时器计数为0 */
    __HAL_TIM_ENABLE(htim);                                 /* 使能定时器 */
}

/**
  * @brief  定时器关闭
  * @param  htim  Timer句柄
  * @retval None
  */
void mb_port_timerDisable(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
}

/**
  * @brief  串口中断服务函数（主机通道）
  * @param  None
  * @retval None
  */
void USART_MCU_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART_MCU_IRQ);

    if((__HAL_UART_GET_FLAG(&HUART_MCU, UART_FLAG_RXNE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_MCU, UART_FLAG_RXNE);
        mbh_uartRxIsr(&mbhMCU);
    }

    if((__HAL_UART_GET_FLAG(&HUART_MCU, UART_FLAG_TXE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_MCU, UART_FLAG_TXE);
        mbh_uartTxIsr(&mbhMCU);
    }
}

/**
  * @brief  串口中断服务函数（从机通道）
  * @param  None
  * @retval None
  */
void USART_USB_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART_USB_IRQ);

    if((__HAL_UART_GET_FLAG(&HUART_USB, UART_FLAG_RXNE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_USB, UART_FLAG_RXNE);
        mbs_uartRxIsr(&mbsUSB);
    }

    if((__HAL_UART_GET_FLAG(&HUART_USB, UART_FLAG_TXE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_USB, UART_FLAG_TXE);
        mbs_uartTxIsr(&mbsUSB);
    }
}

/**
  * @brief  定时器中断服务函数（主机通道）
  * @param  None
  * @retval None
  */
void TIM_MCU_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&HTIM_MCU, TIM_IT_UPDATE);
    mbh_timer3T5Isr(&mbhMCU);
}

/**
  * @brief  定时器中断服务函数（从机通道）
  * @param  None
  * @retval None
  */
void TIM_USB_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&HTIM_USB, TIM_IT_UPDATE);
    mbs_timer3T5Isr(&mbsUSB);
}

/*----------------------------- End of file -------------------------------*/
