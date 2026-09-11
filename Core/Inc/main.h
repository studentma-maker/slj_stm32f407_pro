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
// Incom_ID：与上位机 slj_kickpi_qt_pro（common/reg_map.h）逐位同步的输入别名，
// 取值即为 IN_READ() 的索引（0~19），供本文件下方限位/急停判断代码按名字调用，
// 不再直接写 IN_READ(数字)。别名依据现有限位/联锁代码逻辑，并与上位机
// AutoFeedWorker/SewingWorker/WarnWorker 的实际用法交叉核对得出。
// 后续如物理接线调换，只需把对应别名改指到新的 Incom_N 即可。
enum Incom_ID {
    Incom_1 = 0,
    Incom_LiftLimit = Incom_1,           // 升降电机(MOTOR_UpDown)上限位/原点限位——该电机槽位在上位机已改用于舵机测试，未接入自动流程，此限位含义存疑，需现场核对接线
    Incom_2 ,
    Incom_FBackRetreatLimit = Incom_2,   // 进退电机回退(维护位)限位，cur_dir=0 方向触发
    Incom_3 ,
    Incom_FBackWorkLimit = Incom_3,      // 进退电机前进(工作位)限位，cur_dir=1 方向触发；上位机以此判定"维护状态"
    Incom_4 ,
    Incom_FeedHair = Incom_4,            // 送料气缸(RELAY_FeedHair)位置确认，兼作夹爪移动电机的联锁输入
    Incom_5 ,
    Incom_PressHair = Incom_5,           // 压料气缸(RELAY_PressHair)位置确认
    Incom_6 ,
    Incom_GripperHomeLimit = Incom_6,    // 夹爪移动电机原点/退回限位，触发时步数清零
    Incom_7 ,
    Incom_GripperGripLimit = Incom_7,    // 夹爪移动电机抓取位/前进限位
    Incom_8 ,
    Incom_CartGripLimit = Incom_8,       // 料车升降推杆：夹取位限位
    Incom_9 ,
    Incom_CartLowerLimit = Incom_9,      // 料车升降推杆：下限位
    Incom_10 ,
    Incom_CartUpperLimit = Incom_10,     // 料车升降推杆：上限位/无料
    Incom_11 ,
    Incom_CartPresent = Incom_11,        // 料车到位/固定检测（本体传感器，非推杆限位，本文件未使用）
    Incom_12 ,
    Incom_LineBroken1 = Incom_12,        // 断线检测 1（本文件未使用，仅上位机读取）
    Incom_13 ,
    Incom_LineBroken2 = Incom_13,
    Incom_14 ,
    Incom_LineBroken3 = Incom_14,
    Incom_15 ,
    Incom_LineBroken4 = Incom_15,
    Incom_16 ,
    Incom_Ch3 = Incom_16,                // 缝纫：物料尾部检测（本文件未使用，仅上位机读取）
    Incom_17 ,
    Incom_Ch1 = Incom_17,
    Incom_18 ,
    Incom_Ch2 = Incom_18,
    Incom_19 ,                           // 备用，上下位机均未使用
    Incom_20 ,
    Incom_Estop = Incom_20,              // 急停开关，作用于全部电机通道
    Incom_ID_MAX
};

// IN1~IN20 端口/引脚映射表：与 CubeMX 生成的 IN1_Pin/IN1_GPIO_Port 等宏一一对应，
// 不做顺序调整。丝印编号 N 对应寄存器索引 N-1（即 IN_READ(N-1)），若丝印与实际
// 接线不一致，请在硬件上重新标注丝印，不要在此处调整数组顺序（gpio.c 的
// MX_GPIO_Init() 是按这些宏的原始 Pin/Port 取值硬编码初始化的，调整顺序或互换
// 宏取值都可能导致某些引脚脱离原有初始化分组，读回恒为 0）。
#define IN_READ(index) ( \
    (index <= 19) ? \
    (HAL_GPIO_ReadPin( \
        /* IN1~IN20 端口映射表 */ \
        ((GPIO_TypeDef*[]) { \
            IN1_GPIO_Port, IN2_GPIO_Port, IN3_GPIO_Port, IN4_GPIO_Port, IN5_GPIO_Port, \
            IN6_GPIO_Port, IN7_GPIO_Port, IN8_GPIO_Port, IN9_GPIO_Port, IN10_GPIO_Port, \
            IN11_GPIO_Port, IN12_GPIO_Port, IN13_GPIO_Port, IN14_GPIO_Port, IN15_GPIO_Port, \
            IN16_GPIO_Port, IN17_GPIO_Port, IN18_GPIO_Port, IN19_GPIO_Port, IN20_GPIO_Port \
        })[index], \
        /* IN1~IN20 引脚映射表 */ \
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
