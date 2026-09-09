/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SMD_DR_6_Pin|SMD_DR_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SMD_EN_6_Pin|SMD_EN_5_Pin|I2C_WR_Pin|I2C_SCL_Pin
                          |I2C_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SMD_DR_4_Pin|SMD_DR_3_Pin|SMD_EN_4_Pin|SMD_EN_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SMD_DR_1_Pin|SMD_EN_2_Pin|SMD_EN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUT12_Pin|OUT11_Pin|OUT10_Pin|OUT9_Pin
                          |OUT8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, OUT7_Pin|OUT6_Pin|OUT5_Pin|OUT4_Pin
                          |OUT3_Pin|OUT2_Pin|OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN2_Pin IN4_Pin IN6_Pin IN8_Pin
                           SMD_AM_5_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN4_Pin|IN6_Pin|IN8_Pin
                          |SMD_AM_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_DR_6_Pin SMD_DR_5_Pin */
  GPIO_InitStruct.Pin = SMD_DR_6_Pin|SMD_DR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_EN_6_Pin SMD_EN_5_Pin I2C_WR_Pin */
  GPIO_InitStruct.Pin = SMD_EN_6_Pin|SMD_EN_5_Pin|I2C_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SMD_AM_6_Pin */
  GPIO_InitStruct.Pin = SMD_AM_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMD_AM_6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_DR_4_Pin SMD_DR_3_Pin SMD_EN_4_Pin SMD_EN_3_Pin */
  GPIO_InitStruct.Pin = SMD_DR_4_Pin|SMD_DR_3_Pin|SMD_EN_4_Pin|SMD_EN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_AM_4_Pin SMD_AM_3_Pin IN13_Pin IN15_Pin
                           IN17_Pin IN19_Pin IN20_Pin IN18_Pin
                           IN16_Pin */
  GPIO_InitStruct.Pin = SMD_AM_4_Pin|SMD_AM_3_Pin|IN13_Pin|IN15_Pin
                          |IN17_Pin|IN19_Pin|IN20_Pin|IN18_Pin
                          |IN16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SMD_DR_2_Pin */
  GPIO_InitStruct.Pin = SMD_DR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SMD_DR_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_DR_1_Pin SMD_EN_2_Pin SMD_EN_1_Pin */
  GPIO_InitStruct.Pin = SMD_DR_1_Pin|SMD_EN_2_Pin|SMD_EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD_AM_2_Pin SMD_AM_1_Pin */
  GPIO_InitStruct.Pin = SMD_AM_2_Pin|SMD_AM_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN3_Pin IN5_Pin IN7_Pin
                           IN9_Pin IN11_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN3_Pin|IN5_Pin|IN7_Pin
                          |IN9_Pin|IN11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IN14_Pin IN12_Pin IN10_Pin */
  GPIO_InitStruct.Pin = IN14_Pin|IN12_Pin|IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT12_Pin OUT11_Pin OUT10_Pin OUT9_Pin
                           OUT8_Pin */
  GPIO_InitStruct.Pin = OUT12_Pin|OUT11_Pin|OUT10_Pin|OUT9_Pin
                          |OUT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT7_Pin OUT6_Pin OUT5_Pin OUT4_Pin
                           OUT3_Pin OUT2_Pin OUT1_Pin */
  GPIO_InitStruct.Pin = OUT7_Pin|OUT6_Pin|OUT5_Pin|OUT4_Pin
                          |OUT3_Pin|OUT2_Pin|OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_SCL_Pin I2C_SDA_Pin */
  GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
