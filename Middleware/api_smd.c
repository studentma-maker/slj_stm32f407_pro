/**
  * @file    api_smd.c
  * @brief   SMD API 接口源文件
  * @details 本文件提供了上层应用 SMD 接口的空框架，
  *          包括 SMD 初始化、数据处理及控制等函数。
  * @version V1.0.0
  * @date    22-Aug-2026
  */
#include "api_smd.h"
#include "tim.h"
#include <math.h>  // 浮点运算必备头文件

/* 私有宏定义 --------------------------------------------------------------*/

/* 私有变量定义 ------------------------------------------------------------*/

/**
 * @brief 8路PWM通道渐变参数初始化
 * @note  1. 初始频率200Hz（浮点型200.0f，整型200）
 *        2. 渐变状态默认停止，步长默认0
 */
SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX] = {
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH0
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH1
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH2
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH3
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH4
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH5
};

uint16_t SMD_PU_DATA[SMD_CH_MAX] = {1,1,1,1,1,1};
uint16_t SMD_ACC_DATA[SMD_CH_MAX] = {100,100,100,100,100,100};

/* 私有函数声明 ------------------------------------------------------------*/

/**
 * @brief  设置指定通道的频率渐变（平滑变到目标频率）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  target_freq: 目标频率（1~4000Hz）
 * @param  speed: 渐变速度（1~1000Hz/s）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=参数非法
 * @note   1. 速度单位为Hz/s，函数内转换为Hz/ms（浮点型）
 *         2. 自动判断升/降频方向，设置step_per_ms正负
 *         3. 启动渐变前自动停止当前渐变，避免叠加
 */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t speed)
{
    // 1. 严格参数校验（所有参数必须在限定范围）
    if (ch >= SMD_CH_MAX ||                          // 通道非法
        target_freq < SMD_PWM_FREQ_MIN || target_freq > SMD_PWM_FREQ_MAX || // 频率超界
        speed < SMD_GRADIENT_MIN || speed > SMD_GRADIENT_MAX) // 速度超界
    {
        return HAL_ERROR;
    }

    // 2. 停止当前通道的渐变（避免叠加）
    SMD_PWM_StopFreqGradient(ch);

    // 3. 浮点型计算每ms步长：speed(Hz/s) / 100 → Hz/ms
    float step_per_ms = (float)speed / 100.0f;

    // 4. 获取当前浮点频率和目标浮点频率
    float current_freq = smd_freq_gradient[ch].current_freq_float;
    float target_freq_float = (float)target_freq;

    // 5. 确定步长方向：升频（+）/降频（-）
    if (current_freq > target_freq_float)
    {
        step_per_ms = -step_per_ms; // 降频则步长为负
    }

    // 6. 剩余步长保护：避免单次步长超过目标值
    if ((step_per_ms > 0 && current_freq + step_per_ms > target_freq_float) ||
        (step_per_ms < 0 && current_freq + step_per_ms < target_freq_float))
    {
        step_per_ms = target_freq_float - current_freq; // 剩余步长直接到位
    }

    // 7. 设置渐变参数（全浮点型）
    smd_freq_gradient[ch].target_freq_float = target_freq_float; // 目标浮点频率
    smd_freq_gradient[ch].step_per_ms = step_per_ms;             // 浮点步长
    smd_freq_gradient[ch].is_running = 1;                        // 标记渐变开始

    return HAL_OK;
}

/**
 * @brief  停止指定通道的频率渐变
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @note   1. 停止后current_freq_float保留当前浮点值
 *         2. is_running置0，step_per_ms置0
 */
void SMD_PWM_StopFreqGradient(SMD_Channel ch)
{
    // 参数校验：避免数组越界
    if (ch >= SMD_CH_MAX)
    {
        return;
    }

    // 停止渐变：状态置0 + 步长置0（浮点型）
    smd_freq_gradient[ch].is_running = 0;
    smd_freq_gradient[ch].step_per_ms = 0.0f;
}

/**
 * @brief  启动指定通道的PWM输出
 * @param  ch: 要启动的通道（SMD_Channel枚举值）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 * @note   高级定时器互补通道（CH3/CH6）自动调用HAL_TIMEx_PWMN_Start
 */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch)
{
    // 根据通道类型选择启动函数（普通/PWMN）
    switch(ch)
    {
        case SMD_CH0:
            return HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
        case SMD_CH1:
            return HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
        case SMD_CH2: // 互补通道需启动PWMN
            return HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
        case SMD_CH3:
            return HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
        case SMD_CH4:
            return HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
        case SMD_CH5:
            return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        default:
            return HAL_ERROR;
    }
}

/**
 * @brief  停止指定通道的PWM输出
 * @param  ch: 要停止的通道（SMD_Channel枚举值）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 */
HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch)
{
    // 根据通道类型选择停止函数
    switch(ch)
    {
        case SMD_CH0:
            return HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
        case SMD_CH1:
            return HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
        case SMD_CH2: // 互补通道需停止PWMN
            return HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
        case SMD_CH3:
            return HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
        case SMD_CH4:
            return HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
        case SMD_CH5:
            return HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        default:
            return HAL_ERROR;
    }
}

/**
 * @brief 根据目标频率计算定时器PSC和ARR值
 * @param freq: 目标频率（整型，1~4000Hz）
 * @param psc: 输出参数，预分频值
 * @param arr: 输出参数，自动重装载值
 * @note   1. 系统时钟84MHz，目标计数器频率1MHz → PSC = 84-1 = 83
 *         2. ARR = (1MHz / 目标频率) - 1 → 决定PWM周期
 *         3. 自动做频率边界保护，避免非法值
 */
static void SMD_Calc_PSC_ARR(uint32_t freq, uint32_t *psc, uint32_t *arr ,int ch)
{
    // 频率边界保护：强制限定在1~4000Hz
    if (freq < SMD_PWM_FREQ_MIN)
    {
        freq = SMD_PWM_FREQ_MIN;
    }
    else if (freq > SMD_PWM_FREQ_MAX)
    {
        freq = SMD_PWM_FREQ_MAX;
    }

    if(ch == 2 || ch == 4)
	{
	  // 计算PSC：168MHz / 1MHz - 1 = 167
      *psc = (168000000 / SMD_COUNTER_FREQ) - 1;
	}
	else
	{
	  // 计算PSC：84MHz / 1MHz - 1 = 83
      *psc = (84000000 / SMD_COUNTER_FREQ) - 1;
	}

    
    // 计算ARR：1MHz / 目标频率 - 1（例：200Hz → 1e6/200 -1 = 4999）
    *arr = (SMD_COUNTER_FREQ / freq) - 1;
}

/**
 * @brief  定时器中断服务函数
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    // 检查TIM10更新中断标志
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE); // 清楚中断更新标志
        
        // 遍历所有通道
        for (int ch = 0; ch < SMD_CH_MAX; ch++)
        {
          // 只处理正在渐变的通道
          if (smd_freq_gradient[ch].is_running)
          {
            // ===================== 1. 浮点计算（全程浮点，不丢精度） =====================
            float current_freq = smd_freq_gradient[ch].current_freq_float;
            float target_freq  = smd_freq_gradient[ch].target_freq_float;
            float step         = smd_freq_gradient[ch].step_per_ms;

            // 新频率（浮点）
            float new_freq_float = current_freq + step;

            // 边界限制
            if (new_freq_float < SMD_PWM_FREQ_MIN)
              new_freq_float = SMD_PWM_FREQ_MIN;
            if (new_freq_float > SMD_PWM_FREQ_MAX)
              new_freq_float = SMD_PWM_FREQ_MAX;

            // 到达目标，停止渐变
            if (fabs(new_freq_float - target_freq) < 0.001f)
            {
              new_freq_float = target_freq;
              smd_freq_gradient[ch].is_running = 0;
            }

            // ===================== 2. 变化足够大才更新硬件 =====================
            if (fabs(new_freq_float - current_freq) >= 0.1f)
            {
              // 浮点转整型（给硬件用）
              uint32_t new_freq_int = (uint32_t)round(new_freq_float);

              

              // 计算新的 PSC 和 ARR
              uint32_t psc, arr;
              SMD_Calc_PSC_ARR(new_freq_int, &psc, &arr ,ch);
              
              // 重新配置定时器频率
              HAL_StatusTypeDef ret = HAL_ERROR;
              // 先停PWM
              SMD_PWM_Stop((SMD_Channel)ch);
              switch(ch)
              {
                case SMD_CH0:
                  htim14.Init.Prescaler = psc;
                  htim14.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim14);
                  break;
                case SMD_CH1:
                  htim13.Init.Prescaler = psc;
                  htim13.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim13);
                  break;
                case SMD_CH2:
                  htim8.Init.Prescaler = psc;
                  htim8.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim8);
                  break;
                case SMD_CH3:
                  htim5.Init.Prescaler = psc;
                  htim5.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim5);
                  break;
                case SMD_CH4:
                  htim9.Init.Prescaler = psc;
                  htim9.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim9);
                  break;
                case SMD_CH5:
                  htim2.Init.Prescaler = psc;
                  htim2.Init.Period    = arr;
                  ret = HAL_TIM_PWM_Init(&htim2);
                  break;
                default:
                  break;
              }

              if(ret == HAL_OK)
              {
                // ===================== 关键：变频后强制保持 50% 占空比 =====================
                uint32_t ccr_val = arr / 2;

                switch(ch)
                {
                  case SMD_CH0: __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ccr_val); break;
                  case SMD_CH1: __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, ccr_val); break;
                  case SMD_CH2: __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_1, ccr_val); break;
                  case SMD_CH3: __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_4, ccr_val); break;
                  case SMD_CH4: __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_1, ccr_val); break;
                  case SMD_CH5: __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, ccr_val); break;
                }

                // 保存全局浮点频率（给下一次计算用）
                smd_freq_gradient[ch].current_freq_float = new_freq_float;
                smd_freq_gradient[ch].current_freq_int   = new_freq_int;

                // 重新开PWM
                SMD_PWM_Start((SMD_Channel)ch);
              }
            }
          }
        }

    }
}

/*----------------------------- End of file -------------------------------*/
