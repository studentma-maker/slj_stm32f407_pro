/**
  * @file    api_smd.c
  * @brief   SMD API 接口源文件
  * @details 6路步进电机 S 曲线加减速 + 步数控制核心实现。
  *          移植自 slj_stm32f407 项目 Core/Src/smd.c（8路S曲线版本），
  *          按本板 6 路通道与 EN/AM 信号重新适配：
  *            - S曲线速度更新 / 换向保护 / 步数控制刹车预判：逻辑与原项目一致
  *            - 上电回原点状态机（SMD_SysToOrigin）与限位急停（SMD_IsLimited）：
  *              沿用原项目对应机械结构（进退/升降/夹爪）的判定逻辑，
  *              IN/OUT 序号沿用原项目编号，需与实际线束核对
  *            - 继电器电机（原 OUT12/13 通过继电器正反转驱动的推杆电机）：
  *              新板已改为 M1 独立 PWM+MOS桥驱动，对应限位急停逻辑已移植到
  *              api_motor_control.c 的 API_MOTOR_CheckLimit()，同样在本文件的
  *              TIM10 1ms中断里调用
  * @version V2.0.0
  * @date    09-Sep-2026
  */
#include "api_smd.h"
#include "tim.h"
#include "api_motor_control.h"
#include <math.h>

/* ========================= 硬件定时器映射表 ========================= */
/**
 * @brief 把"哪个通道用哪个定时器/哪个比较通道/是否为互补输出"集中到一张表里。
 * @note  is_complementary=1 的通道（CH2=TIM8）需要用 HAL_TIMEx_PWMN_xxx，
 *        其余通道用普通的 HAL_TIM_PWM_xxx。定时器本身已由 tim.c(CubeMX)
 *        完成 MX_TIMx_Init()，这里只复用已初始化好的全局句柄。
 */
typedef struct
{
    TIM_HandleTypeDef *handle;
    uint32_t            tim_channel;
    uint8_t             is_complementary;
} SMD_TimerMap;

static const SMD_TimerMap smd_timer_map[SMD_CH_MAX] =
{
    /* handle   channel         is_complementary */
    { &htim14, TIM_CHANNEL_1, 0 },  // CH0 - PA7
    { &htim13, TIM_CHANNEL_1, 0 },  // CH1 - PA6
    { &htim8,  TIM_CHANNEL_1, 1 },  // CH2 - PA5 (TIM8_CH1N)
    { &htim5,  TIM_CHANNEL_4, 0 },  // CH3 - PA3
    { &htim9,  TIM_CHANNEL_1, 0 },  // CH4 - PA2
    { &htim2,  TIM_CHANNEL_2, 0 },  // CH5 - PA1
};

/* ========================= 全局变量 ========================= */

/* Modbus可读写的电机参数 */
uint16_t SMD_PU_DATA[SMD_CH_MAX] = {
    SMD_PWM_FREQ_MIN, SMD_PWM_FREQ_MIN, SMD_PWM_FREQ_MIN,
    SMD_PWM_FREQ_MIN, SMD_PWM_FREQ_MIN, SMD_PWM_FREQ_MIN
};
uint16_t SMD_ACC_DATA[SMD_CH_MAX] = {
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT,
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT
};
uint16_t SMD_JERK_DATA[SMD_CH_MAX] = {
    SMD_JERK_DEFAULT, SMD_JERK_DEFAULT, SMD_JERK_DEFAULT,
    SMD_JERK_DEFAULT, SMD_JERK_DEFAULT, SMD_JERK_DEFAULT
};

uint16_t MotorCurStepsU[SMD_CH_MAX] = {0};
static uint32_t MotorCurStepsSub[SMD_CH_MAX] = {0}; /* 子步累加器 (1步=1000子步)，消除浮点累积误差 */

SMD_StepsCtl_t g_motorStepsCtl[SMD_CH_MAX] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
};

GrippertoOrigin_P g_sysToOrigin = defaultset;

SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX] = {
    /* v_c   a_c   v_n   jerk  acc_max dir_change dir_state       freq_int is_running */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH0
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH1
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH2
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH3
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH4
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH5
};

/* 回原点相关目标脉冲频率（沿用原项目取值） */
#define GripperToOriginPU   1600u

/* ========================= 内部函数前置声明 ========================= */
static void SMD_Calc_PSC_ARR    (uint32_t freq, uint32_t *psc, uint32_t *arr, int ch);
static void SMD_AccumulateSteps (SMD_Channel ch, uint32_t freq_int);
static void SMD_ApplyFreqToHW   (SMD_Channel ch, uint32_t freq_int);
static void SMD_UpdateVelocity  (SMD_Freq_Gradient *m, float delta_v, float accel_max, float jerk);
static uint8_t SMD_IsLimited    (int ch, uint8_t cur_dir, SMD_Freq_Gradient *m);
static void SMD_MotorStepsCtl   (SMD_Channel ch);
static void SMD_SysToOrigin     (void);
static void SMD_ProcessChannel  (SMD_Channel ch);
static void SMD_RunSCurve       (SMD_Channel ch, SMD_Freq_Gradient *m);

/* ========================= 内部：频率转PSC/ARR ========================= */
/**
 * @brief  根据目标频率和通道计算PSC和ARR
 * @note   APB2（TIM8/9）挂168MHz，APB1（TIM2/5/13/14）挂84MHz。
 *         本板通道->定时器：CH2=TIM8(168MHz)，CH4=TIM9(168MHz)，其余为84MHz总线。
 */
static void SMD_Calc_PSC_ARR(uint32_t freq, uint32_t *psc, uint32_t *arr, int ch)
{
    if (freq < SMD_PWM_FREQ_MIN) freq = SMD_PWM_FREQ_MIN;
    if (freq > SMD_PWM_FREQ_MAX) freq = SMD_PWM_FREQ_MAX;

    if (ch == SMD_CH2 || ch == SMD_CH4)
        *psc = (168000000 / SMD_COUNTER_FREQ) - 1;  // 167
    else
        *psc = (84000000 / SMD_COUNTER_FREQ) - 1;   // 83

    *arr = (SMD_COUNTER_FREQ / freq) - 1;
}

/* ========================= 内部：步数累加（子步定点累加，消除浮点误差） ========================= */
static void SMD_AccumulateSteps(SMD_Channel ch, uint32_t freq_int)
{
    MotorCurStepsSub[ch] += freq_int;  /* freq_int 子步/ms，1000子步 = 1整步 */
    uint16_t whole = (uint16_t)(MotorCurStepsSub[ch] / 1000u);
    uint8_t dir = SMD_DR_READ(ch);
    if (ch == MOTOR_FBack) dir = !dir;  /* FBack方向反逻辑：DR=0时步数增加 */
    if (whole > 0)
    {
        MotorCurStepsSub[ch] -= (uint32_t)whole * 1000u;
        if (dir)
            MotorCurStepsU[ch] += whole;
        else {
            if (MotorCurStepsU[ch] < whole) MotorCurStepsU[ch] = 0;
            else MotorCurStepsU[ch] -= whole;
        }
    }
}

/* ========================= 内部：直接写入硬件定时器频率（不停PWM） ========================= */
/**
 * @brief  仅更新PSC/ARR寄存器，不停止PWM，适合中断内调用
 * @note   利用 ARR 预装载，修改在下一个周期生效，无毛刺
 */
static void SMD_ApplyFreqToHW(SMD_Channel ch, uint32_t freq_int)
{
    if (ch >= SMD_CH_MAX) return;

    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq_int, &psc, &arr, (int)ch);

    const SMD_TimerMap *t = &smd_timer_map[ch];
    t->handle->Instance->PSC = psc;
    t->handle->Instance->ARR = arr;
    __HAL_TIM_SET_COMPARE(t->handle, t->tim_channel, arr / 2);  // 保持50%占空比

    smd_freq_gradient[ch].current_freq_int = freq_int;
    SMD_AccumulateSteps(ch, freq_int);
}

/* ========================= 内部：S曲线速度更新（移植自旧项目/ESP32 update_velocity） ========================= */
/**
 * @brief  S曲线加减速核心：根据当前速度与目标速度的差距动态调整加速度
 * @param  m:         通道状态指针
 * @param  delta_v:   |v_n - v_c|，剩余速度差（Hz）
 * @param  accel_max: 本通道最大加速度（Hz/s），来自 SMD_ACC_DATA[ch]
 * @param  jerk:      本通道jerk（Hz/s²），来自 SMD_JERK_DATA[ch]
 */
static void SMD_UpdateVelocity(SMD_Freq_Gradient *m, float delta_v, float accel_max, float jerk)
{
    float decel_dist = 0.5f * m->a_c * m->a_c / jerk;
    uint8_t need_max_accel = (delta_v > decel_dist) ? 1 : 0;

    if (need_max_accel)
    {
        m->a_c += jerk * SMD_UPDATE_DT_s;
        if (m->a_c > accel_max)
            m->a_c = accel_max;
    }
    else
    {
        if (delta_v > 0.01f)
            m->jerk = 0.5f * m->a_c * m->a_c / delta_v;
        else
            m->jerk = jerk;

        m->a_c -= m->jerk * SMD_UPDATE_DT_s;
        if (m->a_c < 0.0f)
            m->a_c = 0.0f;
    }
}

/* ========================= PWM启停 ========================= */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return HAL_ERROR;
    const SMD_TimerMap *t = &smd_timer_map[ch];
    return t->is_complementary ? HAL_TIMEx_PWMN_Start(t->handle, t->tim_channel)
                                : HAL_TIM_PWM_Start   (t->handle, t->tim_channel);
}

HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return HAL_ERROR;
    const SMD_TimerMap *t = &smd_timer_map[ch];
    return t->is_complementary ? HAL_TIMEx_PWMN_Stop(t->handle, t->tim_channel)
                                : HAL_TIM_PWM_Stop   (t->handle, t->tim_channel);
}

/* ========================= 频率直接设置（阻塞，用于SP寄存器直接跳变） ========================= */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq)
{
    if (ch >= SMD_CH_MAX || freq < SMD_PWM_FREQ_MIN || freq > SMD_PWM_FREQ_MAX)
        return HAL_ERROR;
    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq, &psc, &arr, (int)ch);

    HAL_StatusTypeDef ret = SMD_PWM_Stop(ch);
    if (ret != HAL_OK) return ret;

    const SMD_TimerMap *t = &smd_timer_map[ch];
    t->handle->Init.Prescaler = psc;
    t->handle->Init.Period    = arr;
    ret = HAL_TIM_PWM_Init(t->handle);
    if (ret != HAL_OK) return ret;

    smd_freq_gradient[ch].current_freq_int = freq;

    return SMD_PWM_Start(ch);
}

/* ========================= S曲线渐变接口（对外） ========================= */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t accel)
{
    if (ch >= SMD_CH_MAX)                                                 return HAL_ERROR;
    if (target_freq < SMD_PWM_FREQ_MIN || target_freq > SMD_PWM_FREQ_MAX) return HAL_ERROR;
    if (accel < SMD_ACC_MAX_MIN || accel > SMD_ACC_MAX_MAX)               return HAL_ERROR;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];

    m->v_n     = (float)target_freq;
    m->acc_max = (float)accel;  // 本次运动的加速度上限，不影响用户写入的 SMD_ACC_DATA

    if (target_freq > SMD_PWM_FREQ_MIN) SMD_PWM_Start(ch);
    m->is_running = 1;

    return HAL_OK;
}

void SMD_PWM_StopFreqGradient(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return;
    smd_freq_gradient[ch].is_running = 0;
    smd_freq_gradient[ch].a_c        = 0.0f;
}

/* ========================= 步数控制刹车距离预判（移植自旧项目） ========================= */
int32_t SMD_CalcAccNeedSteps(float v_current, float a_current,
                              float v_target,  float acc_max, float jerk)
{
    if (v_current <= v_target) return 0;
    if (jerk <= 0.0f) return 0;

    double vc = (double)v_current;
    double ac = (double)a_current;
    double vt = (double)v_target;
    double j  = (double)jerk;
    double am = (double)acc_max;

    double total = 0.0;

    /* 阶段0：若 a_c < acc_max，制动后 a_c 会先继续增大到 acc_max，
     * 同时速度已经开始减小（制动方向） */
    double vc_A = vc;
    if (ac < am)
    {
        double t0      = (am - ac) / j;
        double s0      = vc * t0 - 0.5 * ac * t0 * t0 - (j / 6.0) * t0 * t0 * t0;
        double v_after = vc - ac * t0 - 0.5 * j * t0 * t0;
        if (s0 < 0.0) s0 = 0.0;
        if (v_after <= vt) return (int32_t)(s0 + 0.5);
        total += s0;
        vc_A   = v_after;
    }

    /* 段A：以 acc_max 匀减速，直到剩余速差 = decel_dist = 0.5*am²/j */
    double vc_mid = vt + 0.5 * am * am / j;
    if (vc_A > vc_mid)
    {
        double s_A = (vc_A * vc_A - vc_mid * vc_mid) / (2.0 * am);
        total += s_A;
    }
    else
    {
        vc_mid = vc_A;
    }

    /* 段B：S曲线收尾，a_c 从 am 线性降到 0，速度从 vc_mid 减至 vt */
    double t_B = am / j;
    double s_B = vc_mid * t_B - 0.5 * am * t_B * t_B + (j / 6.0) * t_B * t_B * t_B;
    total += s_B;

    return (int32_t)(total + 0.5);
}

/* ========================= 内部：限位/急停检测 ========================= */
/**
 * @brief  各通道限位开关检测，触发时立即急停该通道
 * @note   IN/OUT 序号与判定逻辑沿用旧项目 slj_stm32f407（同一台机械结构，
 *         仅下位机板卡不同），需与实际线束核对一致：
 *           升降电机(MOTOR_UpDown)：IN1(索引0) 上限位
 *           进退电机(MOTOR_FBack) ：IN2(索引1)/IN3(索引2) 双向限位
 *           夹爪电机(MOTOR_GripperMove)：IN6(索引5)/IN7(索引6) 双向限位，
 *             IN4(索引3) 联锁触发、RELAY_1 联锁触发
 *           急停：IN20(索引19)，作用于全部通道
 */
static uint8_t SMD_IsLimited(int ch, uint8_t cur_dir, SMD_Freq_Gradient *m)
{
    uint8_t hit = 0;
    switch (ch)
    {
        case MOTOR_UpDown:
            hit = (!IN_READ(0) && !cur_dir);
            break;
        case MOTOR_FBack:
            hit = ((!IN_READ(1) && !cur_dir) || (!IN_READ(2) && cur_dir));
            break;
        case MOTOR_GripperMove:
            hit = ((!IN_READ(5) && !cur_dir) ||
                   (!IN_READ(6) && cur_dir)  ||
                   (IN_READ(3)) ||
                   (!OUT_READ(RELAY_1)));
            break;
        default:
            break;
    }
    if (!IN_READ(19)) hit = 1;  // 急停按钮

    if (hit)
    {
        SMD_PWM_Stop((SMD_Channel)ch);
        SMD_PU_DATA[ch]     = SMD_PWM_FREQ_MIN;
        m->v_c              = 0.0f;
        m->a_c              = 0.0f;
        m->v_n              = (float)SMD_PWM_FREQ_MIN;
        m->is_running       = 0;
        m->current_freq_int = SMD_PWM_FREQ_MIN;
        m->dir_change       = 0;
        m->dir_state        = SMD_DIR_NORMAL;
        if (g_motorStepsCtl[ch].is_running)
        {
            g_motorStepsCtl[ch].is_running = 0;
            g_motorStepsCtl[ch].braking = 0;
        }
        /* 触发原点限位时步数清零 */
        if (ch == MOTOR_GripperMove && !IN_READ(5))
        {
            MotorCurStepsU[ch] = 0;
            MotorCurStepsSub[ch] = 0;
        }
    }
    return hit;
}

/* ========================= 内部：步数控制决策（每通道每1ms） ========================= */
/**
 * @brief  步数控制状态机：给定目标步数，自动判断方向、加速->巡航->预判刹车->精确停止
 * @note   仅 MOTOR_FBack / MOTOR_UpDown / MOTOR_GripperMove 三个通道支持步数控制，
 *         其余通道（PaiFa/Trans/Feed）仅支持连续S曲线调速（PU/ACC/JRK/SP），
 *         与旧项目一致。
 */
static void SMD_MotorStepsCtl(SMD_Channel ch)
{
    if (!g_motorStepsCtl[ch].is_running) return;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];
    int32_t step_remain = (int32_t)g_motorStepsCtl[ch].targetSteps - (int32_t)MotorCurStepsU[ch];

    /* 限位直达模式：targetSteps==0 向原点限位，targetSteps==0xFFFF 向远端限位
     * 升降电机  cur_dir=0 上升->上限位 IN_READ(0)，只有原点限位，不支持 0xFFFF
     * 夹爪电机  cur_dir=0->IN_READ(5)  cur_dir=1->IN_READ(6)
     * 进退电机  targetSteps==0 走正常步数控制回零，不走限位直达 */
    if ((g_motorStepsCtl[ch].targetSteps == 0 && ch != MOTOR_FBack) ||
        (g_motorStepsCtl[ch].targetSteps == 0xFFFFu && ch != MOTOR_UpDown))
    {
        uint8_t target_dir = (g_motorStepsCtl[ch].targetSteps == 0) ? 0u : 1u;
        if (ch == MOTOR_FBack) target_dir = 0u;
        uint16_t max_pu = SMD_PU_DATA[ch];

        if (SMD_DR_READ(ch) != target_dir || !m->is_running || m->dir_change)
        {
            SMD_PWM_Stop(ch);
            SMD_DR(ch, target_dir);
            m->v_c = 0.0f;
            m->a_c = 0.0f;
            m->dir_change = 0;
            m->dir_state = SMD_DIR_NORMAL;

            SMD_PWM_SetFreqGradient(ch, max_pu, SMD_ACC_DATA[ch]);
        }
        return;
    }

    /* 阶段1：S曲线运动 (is_running == 1) */

    /* 1a. 刹车监视：每 tick 检查 step_remain，归零立刻停 */
    if (g_motorStepsCtl[ch].braking)
    {
        if (step_remain == 0)
        {
            SMD_PWM_Stop(ch);
            m->v_c              = 0.0f;
            m->a_c              = 0.0f;
            m->is_running       = 0;
            m->current_freq_int = SMD_PWM_FREQ_MIN;
            SMD_PU_DATA[ch]     = SMD_PWM_FREQ_MIN;
            g_motorStepsCtl[ch].is_running = 0;
            g_motorStepsCtl[ch].braking    = 0;
        }
        return;
    }

    /* 1b. 正常运动决策 */

    /* 已精确到达目标：立即停止 */
    if (step_remain == 0)
    {
        SMD_PWM_Stop(ch);
        m->v_c              = 0.0f;
        m->a_c              = 0.0f;
        m->is_running       = 0;
        m->current_freq_int = SMD_PWM_FREQ_MIN;
        SMD_PU_DATA[ch] = SMD_PWM_FREQ_MIN;
        g_motorStepsCtl[ch].is_running = 0;
        return;
    }

    uint8_t need_dir = (step_remain > 0) ? 1u : 0u;
    if (ch == MOTOR_FBack) need_dir = !need_dir;
    uint16_t max_pu = SMD_PU_DATA[ch];

    /* 换向处理 */
    if (!m->dir_change && m->dir_state == SMD_DIR_NORMAL)
    {
        if (need_dir != (uint8_t)SMD_DR_READ(ch))
        {
            m->is_running = 1;
            m->dir_change = 1;
            m->dir_state  = SMD_DIR_NORMAL;
            return;
        }
    }
    else
    {
        m->is_running = 1;
        return;
    }

    /* 方向正确，计算刹车距离：减速到 100Hz 所需步数 + 100 步巡航缓冲 */
    m->is_running = 1;
    int32_t abs_remain = (step_remain >= 0) ? step_remain : -step_remain;

    int32_t brake_steps = SMD_CalcAccNeedSteps(
                  m->v_c, m->a_c,
                  (float)BRAKE_TARGET_HZ,
                  (float)SMD_ACC_DATA[ch],
                  (float)SMD_JERK_DATA[ch]);
    if (brake_steps < 0) brake_steps = 0;

    uint16_t freq_int;
    if (abs_remain < brake_steps + BRAKE_BUFFER)
    {
        freq_int = BRAKE_TARGET_HZ;
    }
    else
    {
        int32_t next_remain = abs_remain - (int32_t)(m->v_c / 1000u);
        if (next_remain < brake_steps)
            freq_int = BRAKE_TARGET_HZ;
        else
            freq_int = max_pu;
    }

    if (freq_int != SMD_PU_DATA[ch] || m->v_n != (float)freq_int)
    {
        SMD_PU_DATA[ch] = freq_int;
        SMD_PWM_SetFreqGradient(ch, freq_int, SMD_ACC_DATA[ch]);

        if (freq_int == BRAKE_TARGET_HZ)
            g_motorStepsCtl[ch].braking = 1;
    }
}

/* ========================= 内部：上电回原点状态机 ========================= */
/**
 * @note   与旧项目一致的 5 阶段复位流程：初始化继电器 -> 设置夹爪方向 ->
 *         等待气缸升起 -> 发夹爪原点脉冲 -> 等待到达原点。
 *         RELAY_1/RELAY_FeedHair/RELAY_PressHair/RELAY_WarnYELLOW 及
 *         WarnLED_on/off、RELAY_FeedHair_up/RELAY_PressHair_up 的取值
 *         按旧项目 README 文档还原（旧项目源码本身缺失这些宏定义），
 *         定义于本文件下方，需与实际继电器接线核对。
 */
static void SMD_SysToOrigin(void)
{
    switch (g_sysToOrigin)
    {
        case defaultset:
            OUT(RELAY_1, 1);
            OUT(RELAY_FeedHair, RELAY_FeedHair_up);
            OUT(RELAY_PressHair, RELAY_PressHair_up);
            OUT(RELAY_WarnYELLOW, WarnLED_on);
            g_sysToOrigin++;
            return;

        case gripperSetDR:
            if (0 != SMD_DR_READ(MOTOR_GripperMove))
            {
                smd_freq_gradient[MOTOR_GripperMove].is_running = 1;
                smd_freq_gradient[MOTOR_GripperMove].dir_change = 1;
                smd_freq_gradient[MOTOR_GripperMove].dir_state  = SMD_DIR_NORMAL;
            }
            g_sysToOrigin++;
            return;

        case waitPressUP:
            if ((!IN_READ(3) && !IN_READ(4) && IN_READ(19) /* 急停 */) || !IN_READ(5))
            {
                g_sysToOrigin++;
            }
            return;

        case gripperSettoOriginPU:
            if (!smd_freq_gradient[MOTOR_GripperMove].dir_change &&
                 smd_freq_gradient[MOTOR_GripperMove].dir_state == SMD_DIR_NORMAL)
            {
                if (GripperToOriginPU != SMD_PU_DATA[MOTOR_GripperMove])
                {
                    SMD_PU_DATA[MOTOR_GripperMove] = GripperToOriginPU;
                    SMD_PWM_SetFreqGradient((SMD_Channel)MOTOR_GripperMove,
                                             SMD_PU_DATA[MOTOR_GripperMove],
                                             SMD_ACC_DATA[MOTOR_GripperMove]);
                }
                g_sysToOrigin++;
            }
            return;

        case waitToOrigin:
            if (!IN_READ(5))
            {
                MotorCurStepsU[MOTOR_GripperMove] = 0;
                MotorCurStepsSub[MOTOR_GripperMove] = 0;
                OUT(RELAY_WarnYELLOW, WarnLED_off);
                g_sysToOrigin++;
            }
            return;

        case toOriginSuccess:
        default:
            break;
    }
}

/* ========================= 内部：S曲线正常加减速一步（每通道每1ms） ========================= */
static void SMD_RunSCurve(SMD_Channel ch, SMD_Freq_Gradient *m)
{
    float v_target  = m->v_n;
    float delta_v   = v_target - m->v_c;
    float abs_dv    = (delta_v >= 0.0f) ? delta_v : -delta_v;
    float accel_max = m->acc_max;
    float jerk_val  = (float)SMD_JERK_DATA[ch];

    /* 到达目标 */
    if (abs_dv <= 0.5f)
    {
        m->v_c = v_target;
        m->a_c = 0.0f;

        /* 目标为 SMD_PWM_FREQ_MIN（1Hz）表示"减速停止"语义 */
        if (v_target <= (float)SMD_PWM_FREQ_MIN)
        {
            SMD_PWM_Stop(ch);
            m->v_c        = 0.0f;
            m->a_c        = 0.0f;
            m->is_running = 0;
            return;
        }

        uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
        if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
        if (freq_int > SMD_PWM_FREQ_MAX) freq_int = SMD_PWM_FREQ_MAX;
        SMD_ApplyFreqToHW(ch, freq_int);
        return;
    }

    /* S曲线：更新加速度 */
    SMD_UpdateVelocity(m, abs_dv, accel_max, jerk_val);

    /* 积分速度 */
    if (delta_v > 0.0f)
        m->v_c += m->a_c * SMD_UPDATE_DT_s;
    else
        m->v_c -= m->a_c * SMD_UPDATE_DT_s;

    if (m->v_c < (float)SMD_PWM_FREQ_MIN) m->v_c = (float)SMD_PWM_FREQ_MIN;
    if (m->v_c > (float)SMD_PWM_FREQ_MAX) m->v_c = (float)SMD_PWM_FREQ_MAX;

    uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
    if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
    if (freq_int > SMD_PWM_FREQ_MAX) freq_int = SMD_PWM_FREQ_MAX;
    SMD_ApplyFreqToHW(ch, freq_int);
}

/* ========================= 内部：单通道每1ms调度（换向 + 限位 + S曲线） ========================= */
static void SMD_ProcessChannel(SMD_Channel ch)
{
    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];
    if (!m->is_running) return;

    /* 分支A：v_c == 0，先换向再检测限位 */
    if (m->v_c <= 1.0f)
    {
        uint8_t cur_dir = (uint8_t)SMD_DR_READ(ch);

        if (m->dir_change)
        {
            m->v_c        = 0.0f;
            m->a_c        = 0.0f;
            cur_dir       = !cur_dir;
            SMD_DR(ch, cur_dir);
            m->dir_change = 0;
            m->dir_state  = SMD_DIR_NORMAL;
        }

        if (SMD_IsLimited((int)ch, cur_dir, m))
        {
            m->dir_change = 0;
            return;
        }
    }
    /* 分支B：v_c != 0，先检测限位再决定换向/S曲线 */
    else
    {
        uint8_t cur_dir = (uint8_t)SMD_DR_READ(ch);
        if (SMD_IsLimited((int)ch, cur_dir, m))
        {
            return;
        }

        if (m->dir_change)
        {
            if (m->dir_state == SMD_DIR_NORMAL)
                m->dir_state = SMD_DIR_DECEL;

            if (m->dir_state == SMD_DIR_DECEL)
            {
                float delta_v = m->v_c;
                if (delta_v <= 1.0f)
                {
                    m->v_c = 0.0f;
                    m->a_c = 0.0f;
                    SMD_PWM_Stop(ch);
                    uint8_t cur_dir2 = (uint8_t)SMD_DR_READ(ch);
                    SMD_DR(ch, !cur_dir2);
                    m->dir_state  = SMD_DIR_WAIT;
                    m->dir_change = 0;
                    SMD_PWM_Start(ch);
                    return;
                }
                SMD_UpdateVelocity(m, delta_v, m->acc_max, (float)SMD_JERK_DATA[ch]);
                m->v_c -= m->a_c * SMD_UPDATE_DT_s;
                if (m->v_c < 0.0f) m->v_c = 0.0f;
                uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
                if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
                SMD_ApplyFreqToHW(ch, freq_int);
                return;
            }

            if (m->dir_state == SMD_DIR_WAIT)
            {
                m->dir_state = SMD_DIR_NORMAL;
                return;
            }
        }
    }

    /* 未换向、未限位：走正常S曲线运动 */
    SMD_RunSCurve(ch, m);
}

/* ========================= TIM10 1ms中断服务函数（S曲线调度） ========================= */
/**
 * @brief  TIM1更新/TIM10全局中断，1ms周期。
 *         TIM10 本身的初始化（PSC/ARR/NVIC）由 tim.c 的 MX_TIM10_Init() 完成，
 *         中断使能由 main.c 的 HAL_TIM_Base_Start_IT(&htim10) 完成。
 * 每1ms依次：
 *   1. 对每个通道调用 SMD_ProcessChannel()（换向/限位检测 + S曲线一步）
 *   2. 对支持步数控制的通道做步数控制决策 SMD_MotorStepsCtl()
 *   3. 推进"回原点"状态机 SMD_SysToOrigin()
 *   4. M1 推杆电机的独立限位/急停保护 API_MOTOR_CheckLimit()
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
        SMD_ProcessChannel((SMD_Channel)ch);

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
        SMD_MotorStepsCtl((SMD_Channel)ch);

    SMD_SysToOrigin();

    API_MOTOR_CheckLimit();
}

/*----------------------------- End of file -------------------------------*/
