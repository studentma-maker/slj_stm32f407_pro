/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN2_Pin GPIO_PIN_6
#define IN2_GPIO_Port GPIOF
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOF
#define IN6_Pin GPIO_PIN_8
#define IN6_GPIO_Port GPIOF
#define IN8_Pin GPIO_PIN_9
#define IN8_GPIO_Port GPIOF
#define SMD_DR_6_Pin GPIO_PIN_4
#define SMD_DR_6_GPIO_Port GPIOC
#define SMD_DR_5_Pin GPIO_PIN_5
#define SMD_DR_5_GPIO_Port GPIOC
#define SMD_EN_6_Pin GPIO_PIN_0
#define SMD_EN_6_GPIO_Port GPIOB
#define SMD_EN_5_Pin GPIO_PIN_1
#define SMD_EN_5_GPIO_Port GPIOB
#define SMD_AM_6_Pin GPIO_PIN_2
#define SMD_AM_6_GPIO_Port GPIOB
#define SMD_AM_5_Pin GPIO_PIN_11
#define SMD_AM_5_GPIO_Port GPIOF
#define SMD_DR_4_Pin GPIO_PIN_12
#define SMD_DR_4_GPIO_Port GPIOF
#define SMD_DR_3_Pin GPIO_PIN_13
#define SMD_DR_3_GPIO_Port GPIOF
#define SMD_EN_4_Pin GPIO_PIN_14
#define SMD_EN_4_GPIO_Port GPIOF
#define SMD_EN_3_Pin GPIO_PIN_15
#define SMD_EN_3_GPIO_Port GPIOF
#define SMD_AM_4_Pin GPIO_PIN_0
#define SMD_AM_4_GPIO_Port GPIOG
#define SMD_AM_3_Pin GPIO_PIN_1
#define SMD_AM_3_GPIO_Port GPIOG
#define SMD_DR_2_Pin GPIO_PIN_7
#define SMD_DR_2_GPIO_Port GPIOE
#define SMD_DR_1_Pin GPIO_PIN_8
#define SMD_DR_1_GPIO_Port GPIOE
#define SMD_EN_2_Pin GPIO_PIN_9
#define SMD_EN_2_GPIO_Port GPIOE
#define SMD_EN_1_Pin GPIO_PIN_10
#define SMD_EN_1_GPIO_Port GPIOE
#define SMD_AM_2_Pin GPIO_PIN_11
#define SMD_AM_2_GPIO_Port GPIOE
#define SMD_AM_1_Pin GPIO_PIN_12
#define SMD_AM_1_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_10
#define IN1_GPIO_Port GPIOD
#define IN3_Pin GPIO_PIN_11
#define IN3_GPIO_Port GPIOD
#define IN5_Pin GPIO_PIN_12
#define IN5_GPIO_Port GPIOD
#define IN7_Pin GPIO_PIN_13
#define IN7_GPIO_Port GPIOD
#define IN9_Pin GPIO_PIN_14
#define IN9_GPIO_Port GPIOD
#define IN11_Pin GPIO_PIN_15
#define IN11_GPIO_Port GPIOD
#define IN13_Pin GPIO_PIN_2
#define IN13_GPIO_Port GPIOG
#define IN15_Pin GPIO_PIN_3
#define IN15_GPIO_Port GPIOG
#define IN17_Pin GPIO_PIN_4
#define IN17_GPIO_Port GPIOG
#define IN19_Pin GPIO_PIN_5
#define IN19_GPIO_Port GPIOG
#define IN20_Pin GPIO_PIN_6
#define IN20_GPIO_Port GPIOG
#define IN18_Pin GPIO_PIN_7
#define IN18_GPIO_Port GPIOG
#define IN16_Pin GPIO_PIN_8
#define IN16_GPIO_Port GPIOG
#define IN14_Pin GPIO_PIN_6
#define IN14_GPIO_Port GPIOC
#define IN12_Pin GPIO_PIN_7
#define IN12_GPIO_Port GPIOC
#define IN10_Pin GPIO_PIN_8
#define IN10_GPIO_Port GPIOC
#define OUT12_Pin GPIO_PIN_3
#define OUT12_GPIO_Port GPIOD
#define OUT11_Pin GPIO_PIN_4
#define OUT11_GPIO_Port GPIOD
#define OUT10_Pin GPIO_PIN_5
#define OUT10_GPIO_Port GPIOD
#define OUT9_Pin GPIO_PIN_6
#define OUT9_GPIO_Port GPIOD
#define OUT8_Pin GPIO_PIN_7
#define OUT8_GPIO_Port GPIOD
#define OUT7_Pin GPIO_PIN_9
#define OUT7_GPIO_Port GPIOG
#define OUT6_Pin GPIO_PIN_10
#define OUT6_GPIO_Port GPIOG
#define OUT5_Pin GPIO_PIN_11
#define OUT5_GPIO_Port GPIOG
#define OUT4_Pin GPIO_PIN_12
#define OUT4_GPIO_Port GPIOG
#define OUT3_Pin GPIO_PIN_13
#define OUT3_GPIO_Port GPIOG
#define OUT2_Pin GPIO_PIN_14
#define OUT2_GPIO_Port GPIOG
#define OUT1_Pin GPIO_PIN_15
#define OUT1_GPIO_Port GPIOG
#define I2C_WR_Pin GPIO_PIN_3
#define I2C_WR_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_4
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_5
#define I2C_SDA_GPIO_Port GPIOB
#define M2_PWM2_Pin GPIO_PIN_6
#define M2_PWM2_GPIO_Port GPIOB
#define M2_PWM1_Pin GPIO_PIN_7
#define M2_PWM1_GPIO_Port GPIOB
#define M1_PWM1_Pin GPIO_PIN_8
#define M1_PWM1_GPIO_Port GPIOB
#define M1_PWM2_Pin GPIO_PIN_9
#define M1_PWM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ��ͨ�����ȡ��IN_READ(����) �� ���� 0���ͣ�/1���ߣ�
// ������0~19 �� ��ӦIN1~IN20��0��ʼ��
#define IN_READ(index) ( \
    (index <= 19) ? \
    (HAL_GPIO_ReadPin( \
        /* IN1~IN20 �˿�ӳ��� */ \
        ((GPIO_TypeDef*[]) { \
            IN1_GPIO_Port, IN2_GPIO_Port, IN3_GPIO_Port, IN4_GPIO_Port, IN5_GPIO_Port, \
            IN6_GPIO_Port, IN7_GPIO_Port, IN8_GPIO_Port, IN9_GPIO_Port, IN10_GPIO_Port, \
            IN11_GPIO_Port, IN12_GPIO_Port, IN13_GPIO_Port, IN14_GPIO_Port, IN15_GPIO_Port, \
            IN16_GPIO_Port, IN17_GPIO_Port, IN18_GPIO_Port, IN19_GPIO_Port, IN20_GPIO_Port \
        })[index], \
        /* IN1~IN20 ����ӳ��� */ \
        ((uint16_t[]) { \
            IN1_Pin, IN2_Pin, IN3_Pin, IN4_Pin, IN5_Pin, \
            IN6_Pin, IN7_Pin, IN8_Pin, IN9_Pin, IN10_Pin, \
            IN11_Pin, IN12_Pin, IN13_Pin, IN14_Pin, IN15_Pin, \
            IN16_Pin, IN17_Pin, IN18_Pin, IN19_Pin, IN20_Pin \
        })[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)
        
// ���������ȡ��ALARM_READ(����) �� ���� 0���ͣ�/1���ߣ�
// ������0~5 �� ��ӦSMD_AM_1~SMD_AM_6��0��ʼ��
#define SMD_AM_READ(index) ( \
    (index <= 5) ? \
    (HAL_GPIO_ReadPin( \
        /* SMD_AM_1~8 �˿�ӳ��� */ \
        ((GPIO_TypeDef*[]) { \
            SMD_AM_1_GPIO_Port, SMD_AM_2_GPIO_Port, SMD_AM_3_GPIO_Port, \
            SMD_AM_4_GPIO_Port, SMD_AM_5_GPIO_Port, SMD_AM_6_GPIO_Port \
        })[index], \
        /* SMD_AM_1~8 ����ӳ��� */ \
        ((uint16_t[]) { \
            SMD_AM_1_Pin, SMD_AM_2_Pin, SMD_AM_3_Pin, \
            SMD_AM_4_Pin, SMD_AM_5_Pin, SMD_AM_6_Pin \
        })[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)
        
// ���ĺ꣺OUT(����, ��ƽ)
// ������0~11 �� ��ӦOUT1~OUT12��0��ʼ��
// ��ƽ��0=�͵�ƽ��1=�ߵ�ƽ��2=��ת��ƽ
#define OUT(index, level) do{ \
    /* ����ӳ�������0��OUT1������1��OUT2...����15��OUT16 */ \
    GPIO_TypeDef* OUT_PORT[] = { \
        OUT1_GPIO_Port, OUT2_GPIO_Port, OUT3_GPIO_Port, OUT4_GPIO_Port, \
        OUT5_GPIO_Port, OUT6_GPIO_Port, OUT7_GPIO_Port, OUT8_GPIO_Port, \
        OUT9_GPIO_Port, OUT10_GPIO_Port, OUT11_GPIO_Port, OUT12_GPIO_Port \
    }; \
    uint16_t OUT_PIN[] = { \
        OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, \
        OUT5_Pin, OUT6_Pin, OUT7_Pin, OUT8_Pin, \
        OUT9_Pin, OUT10_Pin, OUT11_Pin, OUT12_Pin \
    }; \
    /* ��Խ�磺������0~11������ */ \
    if(index <= 11) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(OUT_PORT[index], OUT_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(OUT_PORT[index], OUT_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

// �����ȡ��OUT_READ(����) �� ���� 0���ͣ�/1���ߣ�
// ������0~11 �� ��ӦOUT1~OUT12��0��ʼ��
#define OUT_READ(index) ( \
    (index <= 11) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {OUT1_GPIO_Port, OUT2_GPIO_Port, OUT3_GPIO_Port, OUT4_GPIO_Port, \
         OUT5_GPIO_Port, OUT6_GPIO_Port, OUT7_GPIO_Port, OUT8_GPIO_Port, \
         OUT9_GPIO_Port, OUT10_GPIO_Port, OUT11_GPIO_Port, OUT12_GPIO_Port})[index], \
        ((uint16_t[]) {OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, \
         OUT5_Pin, OUT6_Pin, OUT7_Pin, OUT8_Pin, \
         OUT9_Pin, OUT10_Pin, OUT11_Pin, OUT12_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// SMD_DR֧��˫������SMD_DR(����, ��ƽ)
// ������0~5 �� ��ӦSMD_DR_1~SMD_DR_6����ƽ��0=��/1=��/2=��ת
#define SMD_DR(index, level) do{ \
    GPIO_TypeDef* SMD_DR_PORT[] = { \
        SMD_DR_1_GPIO_Port, SMD_DR_2_GPIO_Port, SMD_DR_3_GPIO_Port, \
        SMD_DR_4_GPIO_Port, SMD_DR_5_GPIO_Port, SMD_DR_6_GPIO_Port \
    }; \
    uint16_t SMD_DR_PIN[] = { \
        SMD_DR_1_Pin, SMD_DR_2_Pin, SMD_DR_3_Pin, \
        SMD_DR_4_Pin, SMD_DR_5_Pin, SMD_DR_6_Pin \
    }; \
    if(index <= 5) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(SMD_DR_PORT[index], SMD_DR_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(SMD_DR_PORT[index], SMD_DR_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

#define SMD_DR_READ(index) ( \
    (index <= 5) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {SMD_DR_1_GPIO_Port, SMD_DR_2_GPIO_Port, SMD_DR_3_GPIO_Port, \
         SMD_DR_4_GPIO_Port, SMD_DR_5_GPIO_Port, SMD_DR_6_GPIO_Port})[index], \
        ((uint16_t[]) {SMD_DR_1_Pin, SMD_DR_2_Pin, SMD_DR_3_Pin, \
         SMD_DR_4_Pin, SMD_DR_5_Pin, SMD_DR_6_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// SMD_EN֧��˫������SMD_EN(����, ��ƽ)
#define SMD_EN(index, level) do{ \
    GPIO_TypeDef* SMD_EN_PORT[] = { \
        SMD_EN_1_GPIO_Port, SMD_EN_2_GPIO_Port, SMD_EN_3_GPIO_Port, \
        SMD_EN_4_GPIO_Port, SMD_EN_5_GPIO_Port, SMD_EN_6_GPIO_Port \
    }; \
    uint16_t SMD_EN_PIN[] = { \
        SMD_EN_1_Pin, SMD_EN_2_Pin, SMD_EN_3_Pin, \
        SMD_EN_4_Pin, SMD_EN_5_Pin, SMD_EN_6_Pin \
    }; \
    if(index <= 5) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(SMD_EN_PORT[index], SMD_EN_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(SMD_EN_PORT[index], SMD_EN_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

#define SMD_EN_READ(index) ( \
    (index <= 5) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {SMD_EN_1_GPIO_Port, SMD_EN_2_GPIO_Port, SMD_EN_3_GPIO_Port, \
         SMD_EN_4_GPIO_Port, SMD_EN_5_GPIO_Port, SMD_EN_6_GPIO_Port})[index], \
        ((uint16_t[]) {SMD_EN_1_Pin, SMD_EN_2_Pin, SMD_EN_3_Pin, \
         SMD_EN_4_Pin, SMD_EN_5_Pin, SMD_EN_6_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

/* ==================== 继电器输出别名（OUT索引，0~11） ====================
 * 移植自旧项目 slj_stm32f407（同一台机械结构，仅下位机板卡不同）。
 * 旧项目源码（Core/Src/smd.c）中直接使用了这些别名，但源码本身缺失其宏定义，
 * 以下取值依据旧项目 README.md 的"继电器映射"章节还原，需与实际继电器接线核对：
 *   RELAY_1(OUT1)=备用/联锁；RELAY_FeedHair(OUT2)=送发气缸；
 *   RELAY_PressHair(OUT3)=压发气缸；RELAY_WarnYELLOW(OUT8)=黄色警示灯 */
#define RELAY_1             0
#define RELAY_FeedHair      1
#define RELAY_PressHair     2
#define RELAY_WarnYELLOW    7

/* 继电器/指示灯电平语义（0=低电平/1=高电平） */
#define WarnLED_on              1
#define WarnLED_off             0
#define RELAY_FeedHair_up       1   // 送发气缸：0=下降/1=上升
#define RELAY_PressHair_up      1   // 压发气缸：0=下降/1=上升

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
