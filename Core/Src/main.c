/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb_host.h"
#include "mb_slave.h"
#include "api_delay.h"
#include "api_timer.h"
#include "api_smd.h"
#include "api_motor_control.h"
#include "api_eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM13_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM14_Init();
    /* USER CODE BEGIN 2 */
    /* 修复S曲线抖动：CubeMX生成的定时器初始化默认关闭ARR预装载（ARPE=0），
     * 而 api_smd.c 的 SMD_ApplyFreqToHW() 每1ms在周期中途写ARR，依赖ARPE=1
     * 保证修改在下一个更新事件生效（无毛刺）。ARPE=0时，加减速阶段新ARR
     * 小于当前CNT会使16位定时器一路计到0xFFFF才回绕（最长约65ms的拉长脉冲），
     * 导致步数控制模式下电机抖动、且停不到目标位置。
     * 这里把6路步进PWM定时器按ARPE=1重新初始化（旧板smd.c即如此）。
     * 注意：必须改Init字段并重调HAL_TIM_PWM_Init，因为SMD_PWM_SetFreq()
     * 内部会再次调用HAL_TIM_PWM_Init，按Init字段把ARPE写回寄存器。 */
    htim2.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim2);
    htim5.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim5);
    htim8.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim8);
    htim9.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim9);
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim13);
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_PWM_Init(&htim14);
    Delay_Init();
    EEPROM_Init();
    mbh_init(&mbhMCU);
    mbs_init(&mbsUSB);
    API_MOTOR_Init(&htim4, 5000);
#if M6TOSERVO_ENABLE
    /* 舵机初始化：TIM2_CH2 配 50Hz 基时，从EEPROM载入参数
     * （全0/全0xFF时写默认值 120/120/50/5500ms），并按初始角度直接
     * 输出 PWM；DR6 拉低作舵机地线（EN6 已硬件飞线接地） */
    SMD_Servo_Init();
#endif
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
#if M6TOSERVO_ENABLE
        if (ch == SERVO_CH) continue;  /* 舵机通道：跳过步进S曲线启动 */
#endif
        SMD_PWM_SetFreqGradient((SMD_Channel)ch, SMD_PU_DATA[ch], SMD_ACC_DATA[ch]);
    }

    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Base_Start_IT(&htim11);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        
        if (bsp_timer_IsFlagSet(BSP_TIMER_FLAG_10MS))
        {
            bsp_timer_ClearFlag(BSP_TIMER_FLAG_10MS);

            uint8_t data[] = {0, 1, 0, 10};
            mbh_send(&mbhMCU, 1, 3, data, 4);

            /* 更新电机速度（非阻塞式）：由100ms周期改为10ms，配合默认2%步进，
             * 0->100%加减速时间从5秒缩短到0.5秒 */
            API_MOTOR_UpdateSpeed();
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        mbh_poll(&mbhMCU);
        mbs_poll(&mbsUSB);
    }

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
