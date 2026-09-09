/**
  * @file    api_motor_control.c
  * @brief   电机驱动 API 实现文件
  * @details 基于 HAL 库，支持平滑加减速，通过 BSP 定时器 10ms 实现非阻塞控制
  * @version V1.0.0
  * @date    27-Aug-2026
  */

#include "api_motor_control.h"
#include "api_delay.h"

/* 私有变量定义 ------------------------------------------------------------*/

/* 电机状态结构体 */
typedef struct
{
    uint32_t ch_op;             /* OP 通道 */
    uint32_t ch_on;             /* ON 通道 */
    int8_t current_speed;       /* 当前实际速度 (-100 ~ 100) */
    int8_t target_speed;        /* 目标速度 (-100 ~ 100) */
    uint8_t direction;          /* 当前方向: 0=停止, 1=正转, 2=反转 */
} MOTOR_State_t;

static TIM_HandleTypeDef *g_htim = NULL;       /*!< 定时器句柄指针 */
static uint32_t g_timer_period = 0;            /*!< 定时器自动重载值（ARR） */
uint8_t g_motor_step_percent = 2;              /*!< 加减速步进百分比 */

/* 通道映射表
 * M1: OP=CH3, ON=CH4
 * M2: OP=CH2, ON=CH1
 */
static const uint32_t CH_OP[] = { TIM_CHANNEL_3, TIM_CHANNEL_2 };
static const uint32_t CH_ON[] = { TIM_CHANNEL_4, TIM_CHANNEL_1 };

/* 电机状态数组 */
static MOTOR_State_t g_motor_state[2];

/* 私有函数声明 ------------------------------------------------------------*/
static void Motor_SetPWM(MOTOR_State_t *pState, int8_t speed);
static void Motor_Brake(MOTOR_State_t *pState);
static uint8_t Motor_GetDirection(int8_t speed);

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  电机驱动初始化
  * @param  htim  指向已初始化的定时器句柄（PWM 已配置并启动）
  * @param  frequency  目标 PWM 频率（Hz）
  * @retval None
  */
void API_MOTOR_Init(TIM_HandleTypeDef *htim, uint32_t frequency)
{
    uint8_t i;

    if (htim == NULL) return;

    g_htim = htim;
    g_timer_period = htim->Init.Period;

    API_MOTOR_ConfigPWMFrequency(frequency);

    /* 初始化两个电机的状态 */
    for (i = 0; i < 2; i++)
    {
        g_motor_state[i].ch_op = CH_OP[i];
        g_motor_state[i].ch_on = CH_ON[i];
        g_motor_state[i].current_speed = 0;
        g_motor_state[i].target_speed = 0;
        g_motor_state[i].direction = 0;
    }

    /* 初始状态：所有通道输出 0（低电平），防止上电误动作 */
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_4, 0);

    /* 启动所有 PWM 通道（每次修改比较值后仍需调用 Start，但这里先启动一次） */
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_4);
}

/**
  * @brief  配置 PWM 输出频率
  * @param  frequency  目标 PWM 频率（Hz）
  * @retval None
  */
void API_MOTOR_ConfigPWMFrequency(uint32_t frequency)
{
    uint32_t timer_clock;
    uint32_t prescaler;

    if (g_htim == NULL || frequency == 0) return;

    timer_clock = HAL_RCC_GetHCLKFreq() / 2;
    if ((g_htim->Instance == TIM1) || (g_htim->Instance == TIM8) ||
        (g_htim->Instance == TIM9) || (g_htim->Instance == TIM10) ||
        (g_htim->Instance == TIM11))
    {
        timer_clock = HAL_RCC_GetHCLKFreq();
    }

    prescaler = (timer_clock / frequency) / (g_timer_period + 1) - 1;
    if (prescaler < 1) prescaler = 1;

    g_htim->Init.Prescaler = prescaler;
    HAL_TIM_PWM_Init(g_htim);
}

/**
  * @brief  设置电机目标速度（非阻塞模式）
  * @param  motor   电机编号（API_MOTOR_1 或 API_MOTOR_2）
  * @param  speed   目标速度百分比，范围 -100 ~ 100
  *                 正数：正转  负数：反转   0：停止
  * @retval None
  * @note   该函数立即返回，速度变换在 BSP 定时器 10ms 中执行
  */
void API_MOTOR_SetSpeed(API_MOTOR_Num_t motor, int8_t speed)
{
    /* 边界检查：只允许 API_MOTOR_1(0) 和 API_MOTOR_2(1) */
    if (motor > API_MOTOR_2) return;

    /* 限幅（支持负数） */
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    g_motor_state[motor].target_speed = speed;
    g_motor_state[motor].direction = Motor_GetDirection(speed);
}

/**
  * @brief  通过通信接口设置电机速度
  * @param  motor   电机编号
  * @param  speed   目标速度百分比
  * @retval None
  * @note   通过通信接口调用，速度变换在 BSP 定时器 10ms 中执行
  */
void API_MOTOR_SetSpeedViaComm(API_MOTOR_Num_t motor, int8_t speed)
{
    API_MOTOR_SetSpeed(motor, speed);
}

/**
  * @brief  通过通信接口设置加减速步进参数
  * @param  step   步进百分比（1-10）
  * @retval None
  */
void API_MOTOR_SetStepViaComm(uint8_t step)
{
    if (step > 0 && step <= 10)
    {
        g_motor_step_percent = step;
    }
}

/**
  * @brief  立即停止电机（紧急刹车）
  * @param  motor   电机编号
  * @retval None
  * @note   两路同时输出低电平，无加减速过程，用于紧急情况
  */
void API_MOTOR_Stop(API_MOTOR_Num_t motor)
{
    /* 边界检查：只允许 API_MOTOR_1(0) 和 API_MOTOR_2(1) */
    if (motor > API_MOTOR_2) return;

    Motor_Brake(&g_motor_state[motor]);
    g_motor_state[motor].current_speed = 0;
    g_motor_state[motor].target_speed = 0;
    g_motor_state[motor].direction = 0;
}

/**
  * @brief  获取当前速度
  * @param  motor   电机编号
  * @retval 当前速度百分比（-100 ~ 100）
  */
int8_t API_MOTOR_GetCurrentSpeed(API_MOTOR_Num_t motor)
{
    if (motor > API_MOTOR_2) return 0;
    return g_motor_state[motor].current_speed;
}

/**
  * @brief  推杆电机限位/急停保护，需每 1ms 调用一次
  * @retval None
  * @note   M1/M2 是同一台电机的双路冗余接线（哪路 MOS 桥烧了就把电机接线
  *         换到另一路，程序不用改），所以两路受完全相同的限位/急停信号
  *         保护，分别独立判断、独立停止，详见 api_motor_control.h 中的说明
  */
void API_MOTOR_CheckLimit(void)
{
    for (API_MOTOR_Num_t m = API_MOTOR_1; m <= API_MOTOR_2; m++)
    {
        int8_t speed = g_motor_state[m].current_speed;
        if (speed == 0) continue;

        uint8_t cur_dir = (speed > 0) ? 1u : 0u;  // 1=正转，0=反转
        uint8_t hit = (((!IN_READ(7) || !IN_READ(9)) && cur_dir) || (!IN_READ(8) && !cur_dir));
        if (!IN_READ(19)) hit = 1;  // 急停按钮

        if (hit)
        {
            API_MOTOR_Stop(m);
        }
    }
}

/**
  * @brief  获取当前加减速步进参数
  * @return 步进百分比
  */
uint8_t API_MOTOR_GetStepPercent(void)
{
    return g_motor_step_percent;
}

/**
  * @brief  BSP 定时器 10ms 中的速度更新函数
  * @retval None
  * @note   在 main.c 的 BSP 定时器回调中调用
  *         平滑减速过零，无急刹车
  */
void API_MOTOR_UpdateSpeed(void)
{
    API_MOTOR_Num_t motor;
    MOTOR_State_t *pState;
    int8_t current, target;
    int8_t next;

    for (motor = API_MOTOR_1; motor <= API_MOTOR_2; motor++)
    {
        pState = &g_motor_state[motor];
        current = pState->current_speed;
        target = pState->target_speed;

        /* 已到达目标，跳过 */
        if (current == target)
            continue;

        /* 步进计算（自然过零，无需特殊分支） */
        if (target > current)
        {
            next = current + g_motor_step_percent;
            if (next > target) next = target;
        }
        else /* target < current */
        {
            next = current - g_motor_step_percent;
            if (next < target) next = target;
        }

        /* 输出到硬件（同时会调用 Start 重新使能 PWM） */
        Motor_SetPWM(pState, next);
        pState->current_speed = next;
    }
}

/* 私有函数实现 ------------------------------------------------------------*/

/**
  * @brief  设置 PWM 输出（修改比较值并重新启动 PWM 输出）
  * @param  pState  电机状态指针
  * @param  speed   速度值（-100 ~ 100）
  * @note   每次修改比较值后调用 HAL_TIM_PWM_Start 以确保立即生效
  */
static void Motor_SetPWM(MOTOR_State_t *pState, int8_t speed)
{
    uint32_t pulse;
    TIM_HandleTypeDef *htim = g_htim;
    if (htim == NULL) return;

    if (speed == 0)
    {
        __HAL_TIM_SET_COMPARE(htim, pState->ch_op, 0);
        __HAL_TIM_SET_COMPARE(htim, pState->ch_on, 0);
        HAL_TIM_PWM_Start(htim, pState->ch_op);
        HAL_TIM_PWM_Start(htim, pState->ch_on);
        pState->direction = 0;
        return;
    }

    if (speed > 0)
    {
        pulse = (uint32_t)((uint32_t)speed * g_timer_period / 100);
        __HAL_TIM_SET_COMPARE(htim, pState->ch_op, pulse);
        __HAL_TIM_SET_COMPARE(htim, pState->ch_on, 0);
        HAL_TIM_PWM_Start(htim, pState->ch_op);
        HAL_TIM_PWM_Start(htim, pState->ch_on);
        pState->direction = 1;
    }
    else
    {
        pulse = (uint32_t)((uint32_t)(-speed) * g_timer_period / 100);
        __HAL_TIM_SET_COMPARE(htim, pState->ch_op, 0);
        __HAL_TIM_SET_COMPARE(htim, pState->ch_on, pulse);
        HAL_TIM_PWM_Start(htim, pState->ch_op);
        HAL_TIM_PWM_Start(htim, pState->ch_on);
        pState->direction = 2;
    }
}

/**
  * @brief  刹车：两路同时输出低电平并重新启动（紧急停止）
  */
static void Motor_Brake(MOTOR_State_t *pState)
{
    TIM_HandleTypeDef *htim = g_htim;
    if (htim == NULL) return;
    __HAL_TIM_SET_COMPARE(htim, pState->ch_op, 0);
    __HAL_TIM_SET_COMPARE(htim, pState->ch_on, 0);
    HAL_TIM_PWM_Start(htim, pState->ch_op);
    HAL_TIM_PWM_Start(htim, pState->ch_on);
    pState->direction = 0;
}

/**
  * @brief  获取速度对应的方向
  * @param  speed  速度值
  * @retval 0=停止, 1=正转, 2=反转
  */
static uint8_t Motor_GetDirection(int8_t speed)
{
    if (speed > 0) return 1;
    if (speed < 0) return 2;
    return 0;
}

/*----------------------------- End of file -------------------------------*/
