/**
  * @file    api_smd.c
  * @brief   SMD API 接口源文件
  * @details 6路步进电机 S 曲线加减速 + 步数控制核心实现。
  *          移植自 slj_stm32f407 项目 Core/Src/smd.c（8路S曲线版本），
  *          按本板 6 路通道与 EN/AM 信号重新适配：
  *            - S曲线速度更新 / 换向保护 / 步数控制刹车预判：逻辑与原项目一致
  *            - 上电回原点状态机（SMD_SysToOrigin）与限位急停（SMD_IsLimited）：
  *              沿用原项目对应机械结构（进退/夹爪）的判定逻辑，
  *              IN/OUT 序号沿用原项目编号，需与实际线束核对
  *            - 升降电机已取消：原 MOTOR_UpDown 通道（SMD_CH3，4号接口）现为
  *              夹爪移动电机；6号接口（SMD_CH5）改造为舵机（M6TOSERVO_ENABLE=1，
  *              见 api_smd.h 宏说明）
  *            - 继电器电机（原 OUT12/13 通过继电器正反转驱动的推杆电机）：
  *              硬件已改为 M1/M2 独立 PWM+MOS桥驱动并调通，限位急停逻辑见
  *              api_motor_control.c 的 API_MOTOR_CheckLimit()，在 TIM10 1ms
  *              中断里调用；旧板 OUT11(上升)/OUT12(下降) 继电器方案及对应的
  *              SMD_CheckRelayMotorLimit() 限位保护已停用（保留代码以注释形式
  *              存档，见本文件对应位置），不再驱动料车电机
  * @version V2.1.0
  * @date    22-Sep-2026
  */
#include "api_smd.h"
#include "tim.h"
#include "api_motor_control.h"
#include "api_eeprom.h"
#include "mb_hook.h"
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

#if M6TOSERVO_ENABLE
/* ========================= 舵机控制（6号步进电机改造，见 api_smd.h 的 M6TOSERVO_ENABLE） ========================= */
/* 配置参数默认值：EEPROM 未初始化（全0/全0xFF）时写入保存 */
static const uint16_t servo_default_params[4] = {
    SERVO_DEFAULT_INIT_ANGLE, SERVO_DEFAULT_ANGLE_A,
    SERVO_DEFAULT_ANGLE_B,    SERVO_DEFAULT_DURATION_MS
};

/* 配置参数（EEPROM 持久化，AM/DR/ACC/JRK 寄存器写入时保存） */
typedef struct
{
    uint16_t initial_angle;  /* 初始角度 0~360（输出限幅0~180） */
    uint16_t angle_a;        /* 角度a */
    uint16_t angle_b;        /* 角度b */
    uint16_t duration_ms;    /* a→b行程时长 ms */
} ServoCfg_t;

static ServoCfg_t g_servo_cfg = {
    SERVO_DEFAULT_INIT_ANGLE, SERVO_DEFAULT_ANGLE_A,
    SERVO_DEFAULT_ANGLE_B,    SERVO_DEFAULT_DURATION_MS
};

/* 运动状态：IDLE=空闲保持当前角度；A_TO_B=按行程时长线性插值 a→b；
 * HOLD_B=到达b后停留 SERVO_HOLD_AT_B_MS 再跳回初始角 */
typedef enum
{
    SERVO_IDLE = 0,
    SERVO_A_TO_B = 1,
    SERVO_HOLD_B = 2
} ServoState_t;

static ServoState_t g_servo_state       = SERVO_IDLE;
static uint32_t     g_servo_elapsed_ms  = 0;     /* 本次插值已运行 ms */
static uint32_t     g_servo_duration_ms = 0;     /* 本次插值时长快照 */
static uint32_t     g_servo_hold_ms     = 0;     /* 在b已停留 ms */
static float        g_servo_start_angle = 0.0f;  /* 本次插值起点快照 */
static float        g_servo_target_angle = 0.0f; /* 本次插值终点快照 */
static float        g_servo_cur_angle   = 0.0f;  /* 当前输出角度（未限幅） */
#endif

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

/* 各通道"已输出脉冲总数"（自由运行、只增不清、不分方向）。
 * 与上面的 MotorCurStepsU 不同：MotorCurStepsU 是带方向、会被限位/回零清零的 uint16 位置量，
 * 这里是给上位机做位移差分用的累计量（皮带位置），因此：
 *   - 32 位：8kHz 满速连续运行约 149 小时才回绕；回绕无妨，上位机用无符号取模做差分
 *   - 不清零：避免上位机清零命令与计数并发造成丢脉冲；上位机需要零点时在本地记 base
 *   - 按 PWM 输出的真实状态计数（见 SMD_PulseCountTick），不依赖 S 曲线/直接跳变/步数模式等
 *     具体是哪条路径改的频率 */
static volatile uint32_t MotorPulseTotal[SMD_CH_MAX] = {0};
static uint32_t MotorPulseSub[SMD_CH_MAX] = {0};    /* 微脉冲累加器 (1脉冲=1000000微脉冲) */

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
/* SMD_CheckRelayMotorLimit(void); -- 旧板OUT11/12继电器料车电机方案已停用，
 * M1/M2 PWM方案已调通，见下方函数定义处的注释 */

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

    /* 硬件最低 50Hz（20ms周期）：1Hz 是"减速停止"的软件哨兵值，不能让硬件
     * 真正按 1Hz 输出——32位定时器（TIM2/5）的 ARR 能容纳 999999（1秒周期），
     * 该值一旦锁进预装载影子寄存器，下一个更新事件要等 1 秒，期间 S曲线
     * 缓起在软件里"空跑"完，1 秒后频率直接跳到接近目标值，表现为没有
     * 加减速过程（16位定时器写 999999 截断成 16959≈65.5ms，所以以前
     * 夹爪在 TIM9 上感觉不到）。钳位后最长的陈旧锁存周期 ≤20ms，缓起
     * 最多提前 20ms，不可感知。真实运动最低 100Hz（BRAKE_TARGET_HZ），
     * 不受影响。与 main.c 的 Init.Period=19999 占位周期配合，保证 ARR
     * 影子寄存器里永远不存在超长周期，运行时无需生成更新事件。 */
    if (freq < 50u) freq = 50u;

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

/* ========================= 内部：脉冲总数计数（每1ms调用一次） ========================= */
/**
 * @brief  判断通道 PWM 输出当前是否使能（CCxE / 互补通道 CCxNE）
 * @note   SMD_PWM_Stop() 会关闭对应位，因此"停止"期间不计数
 */
static uint8_t SMD_PWM_IsOutputOn(SMD_Channel ch)
{
    const SMD_TimerMap *t = &smd_timer_map[ch];
    const uint32_t mask = t->is_complementary ? (TIM_CCER_CC1NE << t->tim_channel)
                                              : (TIM_CCER_CC1E  << t->tim_channel);
    return (t->handle->Instance->CCER & mask) ? 1u : 0u;
}

/**
 * @brief  按各通道 PWM 的真实输出频率累加脉冲总数
 * @note   实际输出频率 = 定时器时钟 / ((PSC+1)*(ARR+1))，直接读硬件寄存器：
 *         - 不用 freq_int：SMD_Calc_PSC_ARR 对 ARR 整除截断，实际频率比设定值偏高，
 *           最坏约 +0.8%（8kHz 附近），且随频率变化，用设定值积分会有随速度变化的系统偏差
 *         - 不依赖 SMD_ApplyFreqToHW：SP 直接跳变(SMD_PWM_SetFreq)等路径不经过它，
 *           从寄存器读才不会漏
 *         每 1ms 走过的脉冲 = f(Hz)/1000，用微脉冲(1e-6)定点累加，无浮点累积误差
 *         ARR 有预装载，寄存器值可能比实际输出超前不到一个周期（低频时才明显），
 *         对累计值的影响不超过 1 个脉冲量级
 */
static void SMD_PulseCountTick(void)
{
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
#if M6TOSERVO_ENABLE
        if (ch == SERVO_CH) continue;  /* 舵机通道：50Hz是伺服信号不是步进脉冲，不计数 */
#endif
        if (!SMD_PWM_IsOutputOn((SMD_Channel)ch)) continue;

        const SMD_TimerMap *t = &smd_timer_map[ch];
        const uint32_t arr    = t->handle->Instance->ARR;
        const uint32_t psc    = t->handle->Instance->PSC;
        const uint32_t timclk = (ch == SMD_CH2 || ch == SMD_CH4) ? 168000000u : 84000000u;
        const uint32_t cnt_hz = timclk / (psc + 1u);

        /* 每 1ms 微脉冲数 = f(Hz) * 1000 = cnt_hz * 1000 / (arr+1) */
        const uint32_t up = (uint32_t)(((uint64_t)cnt_hz * 1000u) / ((uint64_t)arr + 1u));
        if (up > (SMD_PWM_FREQ_MAX + 1000u) * 1000u) continue;   /* 定时器尚未配置合理频率(ARR过小)，不计 */

        MotorPulseSub[ch] += up;
        const uint32_t whole = MotorPulseSub[ch] / 1000000u;
        if (whole > 0u)
        {
            MotorPulseSub[ch] -= whole * 1000000u;
            MotorPulseTotal[ch] += whole;
        }
    }
}

/* 皮带位置用：读取指定通道的脉冲总数（32位读在 Cortex-M4 上是原子的） */
uint32_t SMD_GetPulseTotal(SMD_Channel ch)
{
    return (ch < SMD_CH_MAX) ? MotorPulseTotal[ch] : 0u;
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

#if M6TOSERVO_ENABLE
/* ========================= 舵机控制（仅 M6TOSERVO_ENABLE=1 时编译） ========================= */

/**
 * @brief  角度->比较寄存器值（正极性输出，与旧板 slj_stm32f407 舵机测试一致）
 * @note   旧板映射（Core/Src/smd.c 的 SMD_DUTY_DATA + mb_hook.c 舵机测试）：
 *         DUTY 25~125（0.1%单位）对应 0.5~2.5ms 高电平脉宽，0°=0.5ms、
 *         180°=2.5ms、90°=1.5ms 中位。PWM1+OCPolarity HIGH 下 CCR 即高电平
 *         计数：CCR = 500 + 角度×1000/90（计数频率 1MHz，PSC=83）。
 *         注意：上位机 servoworker 的 sysfs PWM 写 100%-logicDuty 是因为它的
 *         输出经外部驱动级反相，舵机端必须是正脉冲。
 *         SERVO_PWM_INVERT=1 时 PA1 与舵机之间串了反相光耦，引脚输出
 *         反相波形（CCR = 20000 - 脉宽），舵机端仍是上述正脉冲。
 *         超过180°按180°输出（与上位机 clampAngle 0~180 一致），因此寄存器
 *         虽接受0~360，实际输出封顶180°。
 */
static uint32_t Servo_AngleToCCR(float angle_deg)
{
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > (float)SERVO_ANGLE_PWM_MAX) angle_deg = (float)SERVO_ANGLE_PWM_MAX;

    /* 舵机端正脉冲宽度计数：0°=500(0.5ms) ~ 180°=2500(2.5ms) */
    const uint32_t pulse = 500u + (uint32_t)(angle_deg * 1000.0f / 90.0f + 0.5f);

#if SERVO_PWM_INVERT
    /* 经反相光耦输出：PWM1 下 CNT<CCR 为高，CCR=周期-脉宽 使引脚在每周期
     * 末尾输出 pulse 个计数的低电平，光耦反相后舵机端即为 pulse 宽的正脉冲 */
    return (SERVO_PWM_ARR + 1u) - pulse;
#else
    return pulse;
#endif
}

/**
 * @brief  输出指定角度：写比较值并确保 PWM 通道开启（立即生效，无插值）
 * @note   CCR 预装载开启（MX_TIM2_Init 配置通道时置位），占空比在下一个
 *         更新事件生效，不会产生毛刺；快速运动=直接跳变，舵机以自身
 *         最大机械速度跟随（与上位机 jumpToAngle 一致）
 */
static void Servo_WriteAngle(float angle_deg)
{
    const SMD_TimerMap *t = &smd_timer_map[SERVO_CH];
    __HAL_TIM_SET_COMPARE(t->handle, t->tim_channel, Servo_AngleToCCR(angle_deg));
    SMD_PWM_Start(SERVO_CH);
    g_servo_cur_angle = angle_deg;
}

/**
 * @brief  从 EEPROM 载入配置参数：全0/全0xFF（未初始化）时写入默认值
 * @note   全新 24C02 擦除态读出 0xFF，旧板可能残留 0x00，两种都按
 *         "未初始化"处理并写默认值保存；读入后角度限幅0~360
 */
static void Servo_LoadParams(void)
{
    uint16_t buf[4] = {0, 0, 0, 0};
    uint8_t all_zero = 1, all_ff = 1;

    EEPROM_ReadData(AT24Cxx_SERVO_PARAMS_ADDR, (uint8_t *)buf, sizeof(buf));
    for (int i = 0; i < 4; i++)
    {
        if (buf[i] != 0u)      all_zero = 0;
        if (buf[i] != 0xFFFFu) all_ff   = 0;
    }
    if (all_zero || all_ff)
    {
        for (int i = 0; i < 4; i++) buf[i] = servo_default_params[i];
        EEPROM_WriteData(AT24Cxx_SERVO_PARAMS_ADDR, (uint8_t *)buf, sizeof(buf));
    }

    g_servo_cfg.initial_angle = (buf[0] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : buf[0];
    g_servo_cfg.angle_a       = (buf[1] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : buf[1];
    g_servo_cfg.angle_b       = (buf[2] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : buf[2];
    /* 行程时长不持久化：开机取默认值，由上位机每次启动前随启动帧下发；
     * EEPROM 第4个字保留不用（维持原4×uint16布局，兼容已写入的旧数据） */
    g_servo_cfg.duration_ms   = SERVO_DEFAULT_DURATION_MS;
}

/**
 * @brief  配置参数写入 EEPROM（AM/DR/ACC 寄存器写入时调用；第4个字保留）
 */
static void Servo_SaveParams(void)
{
    const uint16_t buf[4] = { g_servo_cfg.initial_angle, g_servo_cfg.angle_a,
                              g_servo_cfg.angle_b,       SERVO_DEFAULT_DURATION_MS };
    EEPROM_WriteData(AT24Cxx_SERVO_PARAMS_ADDR, (uint8_t *)buf, sizeof(buf));
}

/**
 * @brief  舵机初始化：50Hz 基时 + 载入EEPROM参数 + 直接输出初始角度PWM
 * @note   开机即按配置的初始角度输出（不再等待上位机指令）。
 *         EN6(PB0) 已硬件飞线接地（舵机使能恒有效），DR6(PC4) 固定输出
 *         低电平，作为舵机地线（PU 接信号线、5V 接正极）。
 *         本函数须在 main.c 的 ARPE 重新初始化块之后调用，此时
 *         htim2.Init.AutoReloadPreload 已置 ENABLE，HAL_TIM_PWM_Init()
 *         会把 50Hz 的 PSC/ARR 连同 ARPE=1 一起写入寄存器（与其余
 *         5路步进定时器一致）。占空比只在 CCR 预装载开启的前提下才能
 *         无毛刺切换，ARPE 对 CCR 无影响，CCR 预装载由 MX_TIM2_Init()
 *         配置通道时置位，这里不再重复配置通道。
 */
void SMD_Servo_Init(void)
{
    const SMD_TimerMap *t = &smd_timer_map[SERVO_CH];

    /* 50Hz 基时：TIM2 挂 APB1 84MHz，PSC=83 得 1MHz 计数，ARR=19999 得 20ms 周期。
     * HAL_TIM_PWM_Init() 只重写 PSC/ARR/CR1 基时寄存器（内部走 TIM_Base_SetConfig），
     * 通道配置（CCMR/CCER 等）保持 MX_TIM2_Init() 的原样不动 */
    t->handle->Init.Prescaler = SERVO_PWM_PSC;
    t->handle->Init.Period    = SERVO_PWM_ARR;
    HAL_TIM_PWM_Init(t->handle);

    /* DR6 拉低作舵机地线（本通道已从步进调度中排除，引脚不会被翻电平） */
    SMD_DR(SERVO_CH, 0);

    /* 载入EEPROM参数（未初始化则写默认值），并按初始角度直接输出 PWM */
    Servo_LoadParams();
    g_servo_state = SERVO_IDLE;
    Servo_WriteAngle((float)g_servo_cfg.initial_angle);
}

/**
 * @brief  启动一次完整流程：初始角→a（快速跳变）→a到b（线性插值）→在b停留
 *         SERVO_HOLD_AT_B_MS →初始角（快速跳变）
 * @note   触发条件（mb_hook.c）保证仅空闲时调用，运行中重复触发会被忽略，
 *         避免周期重发的触发帧让舵机反复跳回角度a。插值参数（角度a/b/时长）
 *         在启动时快照，运行期间写入的新配置参数不影响本次流程，下次触发生效
 */
void SMD_Servo_StartSequence(void)
{
    /* 快速跳变到角度a（舵机以自身最大机械速度运动） */
    Servo_WriteAngle((float)g_servo_cfg.angle_a);

    g_servo_elapsed_ms   = 0;
    g_servo_duration_ms  = g_servo_cfg.duration_ms;
    g_servo_start_angle  = (float)g_servo_cfg.angle_a;
    g_servo_target_angle = (float)g_servo_cfg.angle_b;
    g_servo_state        = SERVO_A_TO_B;
}

/**
 * @brief  停止自动化流程并快速回到初始角度（SP=0 控制口语义）
 * @note   空闲时调用只会跳到初始角度，无副作用
 */
void SMD_Servo_StopReturnInitial(void)
{
    g_servo_state = SERVO_IDLE;
    Servo_WriteAngle((float)g_servo_cfg.initial_angle);
}

/**
 * @brief  急停：中止当前流程，保持当前角度输出不动
 * @note   急停按钮（TIM10 1ms 检测）与"全部步进电机急停"寄存器共用；
 *         保持当前角度 = 机构原地冻结，不做任何未预期的运动
 */
void SMD_Servo_AbortHold(void)
{
    g_servo_state = SERVO_IDLE;
}

/**
 * @brief  手动直接跳到指定角度：中止自动化流程，不保存EEPROM
 */
void SMD_Servo_JumpToAngle(uint16_t angle)
{
    g_servo_state = SERVO_IDLE;
    Servo_WriteAngle((float)angle);
}

/**
 * @brief  舵机 1ms 调度（TIM10 中断）：推进 a→b 线性插值及到达b后的停留计时
 * @note   线性插值与上位机 servoworker 一致（上位机 tick 5ms，此处 1ms 更细）。
 *         行程时长=0 时首个 tick 直接跳到b并开始停留；停留期间仍算运行中
 *         （重复启动被忽略），写SP=0/急停/手动跳角度可提前中止
 */
void SMD_Servo_Tick(void)
{
#if SERVO_AUTO_SWEEP_TEST
    /* 【临时测试】自动往复：三角波，0°→180°→0° 匀速来回，覆盖正常流程 */
    static uint32_t sweep_ms = 0;
    if (!IN_READ(Incom_Estop)) return;   /* 急停按下：暂停在当前角度 */

    sweep_ms = (sweep_ms + 1u) % (2u * SERVO_AUTO_SWEEP_HALF_MS);
    const uint32_t up_ms = (sweep_ms <= SERVO_AUTO_SWEEP_HALF_MS)
                         ? sweep_ms : (2u * SERVO_AUTO_SWEEP_HALF_MS - sweep_ms);
    Servo_WriteAngle((float)SERVO_ANGLE_PWM_MAX * (float)up_ms / (float)SERVO_AUTO_SWEEP_HALF_MS);
#else
    if (g_servo_state == SERVO_HOLD_B)
    {
        /* 在b停留计时到：快速回到初始角度，结束本次流程 */
        if (++g_servo_hold_ms >= SERVO_HOLD_AT_B_MS)
        {
            g_servo_state = SERVO_IDLE;
            Servo_WriteAngle((float)g_servo_cfg.initial_angle);
        }
        return;
    }

    if (g_servo_state != SERVO_A_TO_B) return;

    if (g_servo_elapsed_ms >= g_servo_duration_ms)
    {
        /* a→b 完成：确保停在b，开始停留计时 */
        Servo_WriteAngle(g_servo_target_angle);
        g_servo_hold_ms = 0;
        g_servo_state   = SERVO_HOLD_B;
        return;
    }

    g_servo_elapsed_ms++;
    const float ratio = (float)g_servo_elapsed_ms / (float)g_servo_duration_ms;
    Servo_WriteAngle(g_servo_start_angle +
                     (g_servo_target_angle - g_servo_start_angle) * ratio);
#endif /* SERVO_AUTO_SWEEP_TEST */
}

/* 配置参数设置/获取（角度设置时立即写入EEPROM保存，供下次开机使用；行程时长不保存） */
void SMD_Servo_SetInitialAngle(uint16_t angle)
{
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    if (g_servo_cfg.initial_angle != angle)
    {
        g_servo_cfg.initial_angle = angle;
        Servo_SaveParams();
    }
}

void SMD_Servo_SetAngleA(uint16_t angle)
{
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    if (g_servo_cfg.angle_a != angle)
    {
        g_servo_cfg.angle_a = angle;
        Servo_SaveParams();
    }
}

void SMD_Servo_SetAngleB(uint16_t angle)
{
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    if (g_servo_cfg.angle_b != angle)
    {
        g_servo_cfg.angle_b = angle;
        Servo_SaveParams();
    }
}

void SMD_Servo_SetDurationMs(uint16_t ms)
{
    /* 仅内存生效，不写EEPROM：时长由上位机按尾段剩余时间实时计算，每次都可能
     * 不同，持久化会磨损EEPROM并拖慢本帧应答 */
    g_servo_cfg.duration_ms = ms;
}

uint16_t SMD_Servo_GetInitialAngle(void) { return g_servo_cfg.initial_angle; }
uint16_t SMD_Servo_GetAngleA(void)       { return g_servo_cfg.angle_a; }
uint16_t SMD_Servo_GetAngleB(void)       { return g_servo_cfg.angle_b; }
uint16_t SMD_Servo_GetDurationMs(void)   { return g_servo_cfg.duration_ms; }

/**
 * @brief  获取舵机当前实际角度（按0~180限幅后取整，PU寄存器回读用）
 */
uint16_t SMD_Servo_GetCurrentAngle(void)
{
    float a = g_servo_cur_angle;
    if (a < 0.0f) a = 0.0f;
    if (a > (float)SERVO_ANGLE_PWM_MAX) a = (float)SERVO_ANGLE_PWM_MAX;
    return (uint16_t)(a + 0.5f);
}

/**
 * @brief  舵机流程是否运行中（SP寄存器回读用）
 */
uint8_t SMD_Servo_IsRunning(void)
{
    return (g_servo_state != SERVO_IDLE) ? 1u : 0u;
}
#endif /* M6TOSERVO_ENABLE */

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

    /* 无需在此生成更新事件：ARR 影子寄存器里不可能再有超长周期——
     * main.c 初始化时已把 20ms 占位周期（Init.Period=19999）锁入影子寄存器
     * （HAL_TIM_PWM_Init 内部生成更新事件），且 SMD_Calc_PSC_ARR 把硬件
     * 周期钳位在 ≤20ms（最低 50Hz）。因此任何一次启动后，第一个更新事件
     * 最迟 20ms 到来，S曲线随后写入的新 ARR 随即生效。运行中不产生 UG，
     * 避免周期中途强制更新产生毛刺（与旧板注释的做法一致）。 */
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
 * @note   判定逻辑沿用旧项目 slj_stm32f407（同一台机械结构，仅下位机板卡不同），
 *         Incom_ID 别名定义见 main.h，需与实际线束核对一致：
 *           进退电机(MOTOR_FBack) ：Incom_FBackRetreatLimit/Incom_FBackWorkLimit 双向限位
 *           夹爪电机(MOTOR_GripperMove)：Incom_GripperHomeLimit/Incom_GripperGripLimit 双向限位，
 *             Incom_FeedHair 联锁触发、RELAY_1 联锁触发
 *           急停：Incom_Estop，作用于全部通道
 *           升降电机已取消（原 Incom_LiftLimit 输入已改作 Incom_Ch4 排发皮带零点标记，
 *             仅上位机读取，不参与限位逻辑）；
 *           舵机通道（SERVO_CH）不进入本函数（见 TIM10 中断调度）
 */
static uint8_t SMD_IsLimited(int ch, uint8_t cur_dir, SMD_Freq_Gradient *m)
{
    uint8_t hit = 0;
    switch (ch)
    {
        case MOTOR_FBack:
            hit = ((!IN_READ(Incom_FBackRetreatLimit) && !cur_dir) ||
                   (!IN_READ(Incom_FBackWorkLimit) && cur_dir));
            break;
        case MOTOR_GripperMove:
            hit = ((!IN_READ(Incom_GripperHomeLimit) && !cur_dir) ||
                   (!IN_READ(Incom_GripperGripLimit) && cur_dir)  ||
                   (IN_READ(Incom_FeedHair)) ||
                   (!OUT_READ(RELAY_1)));
            break;
        default:
            break;
    }
    if (!IN_READ(Incom_Estop)) hit = 1;  // 急停按钮

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
        if (ch == MOTOR_GripperMove && !IN_READ(Incom_GripperHomeLimit))
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
 * @note   仅 MOTOR_FBack / MOTOR_GripperMove 两个通道支持步数控制，
 *         其余通道（PaiFa/Trans/送发辅助）仅支持连续S曲线调速（PU/ACC/JRK/SP），
 *         与旧项目一致。舵机通道（SERVO_CH）不进入本函数（见 TIM10 中断调度）。
 */
static void SMD_MotorStepsCtl(SMD_Channel ch)
{
    if (!g_motorStepsCtl[ch].is_running) return;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];
    int32_t step_remain = (int32_t)g_motorStepsCtl[ch].targetSteps - (int32_t)MotorCurStepsU[ch];

    /* 限位直达模式：targetSteps==0 向原点限位，targetSteps==0xFFFF 向远端限位
     * 夹爪电机  cur_dir=0->Incom_GripperHomeLimit  cur_dir=1->Incom_GripperGripLimit
     * 进退电机  targetSteps==0 走正常步数控制回零，不走限位直达 */
    if ((g_motorStepsCtl[ch].targetSteps == 0 && ch != MOTOR_FBack) ||
        (g_motorStepsCtl[ch].targetSteps == 0xFFFFu))
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
            if ((!IN_READ(Incom_FeedHair) && !IN_READ(Incom_PressHair) && IN_READ(Incom_Estop) /* 急停 */)
                || !IN_READ(Incom_GripperHomeLimit))
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
            if (!IN_READ(Incom_GripperHomeLimit))
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

/* ========================= 内部：料车继电器电机限位/急停（已停用） ========================= */
/**
 * @brief  【已停用】料车升降继电器电机限位/急停保护，原每 1ms 调用一次
 * @note   M1/M2 PWM+MOS桥硬件已调通，料车电机改为直接由 API_MOTOR_CheckLimit()
 *         保护（速度符号代替本函数的"继电器电平"判方向），不再使用 OUT11(上升)/
 *         OUT12(下降) 两路继电器驱动，本函数整体注释保留存档，不再编译进固件：
 *
 * static void SMD_CheckRelayMotorLimit(void)
 * {
 *     if (!OUT_READ(RELAY_CartUp) && !OUT_READ(RELAY_CartDown)) return;
 *
 *     uint8_t cur_dir = OUT_READ(RELAY_CartUp);  // OUT11=1→上升, OUT12=1→下降
 *     uint8_t hit = (((!IN_READ(Incom_CartGripLimit) || !IN_READ(Incom_CartUpperLimit)) && cur_dir) ||
 *                    (!IN_READ(Incom_CartLowerLimit) && !cur_dir));
 *     if (!IN_READ(Incom_Estop)) hit = 1;  // 急停按钮
 *
 *     if (hit)
 *     {
 *         OUT(RELAY_CartUp, 0);
 *         OUT(RELAY_CartDown, 0);
 *     }
 * }
 */

/* ========================= 排发皮带零点 / 一圈步数（ch4 黑点） ========================= */
#define BELT_CH4_DEBOUNCE_MS      3u   /* ch4 电平连续保持 3ms 才算翻转；边沿脉冲取第一次变化那一 ms */
#define BELT_CALIB_TIMEOUT_REVS   (BELT_CALIB_REVS + 2u)   /* 走了这么多圈仍未数满：停皮带放弃（ch4 未接/脏污/漏检） */

typedef enum { BELT_CALIB_IDLE = 0, BELT_CALIB_WAIT_FIRST, BELT_CALIB_COUNTING } BeltCalibState;

static volatile uint32_t s_beltCycleSteps   = BELT_CYCLE_STEPS_DEFAULT;   /* 一圈步数 */
static volatile uint32_t s_beltZeroPulse    = 0;    /* 最近一次 ch4 上升沿时的皮带脉冲总数 */
static volatile uint8_t  s_beltZeroValid    = 0;    /* 开机后是否已经过黑点 */
static uint8_t           s_ch4Stable        = 0;    /* ch4 消抖后电平 */
static uint8_t           s_ch4Cnt           = 0;    /* 与稳定电平不同的连续 ms 数 */
static uint32_t          s_ch4EdgePulse     = 0;    /* 本次电平变化第一 ms 的脉冲总数 */
static volatile uint8_t  s_beltCalibState   = BELT_CALIB_IDLE;
static uint32_t          s_beltCalibStart   = 0;    /* 进入标定时的脉冲总数（超时判定） */
static uint32_t          s_beltCalibFirst   = 0;    /* 标定第一次上升沿的脉冲总数 */
static uint8_t           s_beltCalibRevs    = 0;    /* 第一次上升沿之后已数到的整圈数 */
static volatile uint8_t  s_beltSavePending  = 0;
static volatile uint8_t  s_beltSeeking      = 0;    /* 开机找零进行中 */
static uint32_t          s_beltSeekStart    = 0;    /* 开始找零时的脉冲总数（超时判定） */

void SMD_Belt_Init(void)
{
    uint32_t steps = 0;
    EEPROM_ReadData(AT24Cxx_BELT_CYCLE_ADDR, (uint8_t *)&steps, 4);
    s_beltCycleSteps = (steps == 0u || steps == 0xFFFFFFFFu) ? BELT_CYCLE_STEPS_DEFAULT : steps;
    /* 以开机时电平为初始稳定值：上电时黑点正好在光电下，不能当成一次上升沿 */
    s_ch4Stable = IN_READ(Incom_Ch4) ? 1u : 0u;
}

uint8_t SMD_Belt_CalibStart(void)
{
    const SMD_Channel ch = MOTOR_PaiFa;
    if (s_beltCalibState != BELT_CALIB_IDLE || SMD_PWM_IsOutputOn(ch)) return 0;
    s_beltCalibStart = MotorPulseTotal[ch];
    s_beltCalibState = BELT_CALIB_WAIT_FIRST;

    /* 皮带已停：直接设方向并按 S 曲线升到标定频率（不依赖寄存器镜像差异触发，
     * 镜像由调用方 mb_hook 同步为同样的 DR/PU，槽位循环不会再动它） */
    smd_freq_gradient[ch].dir_change = 0;
    smd_freq_gradient[ch].dir_state  = SMD_DIR_NORMAL;
    SMD_DR(ch, BELT_CALIB_DR);
    SMD_PU_DATA[ch] = BELT_CALIB_FREQ;
    SMD_PWM_SetFreqGradient(ch, BELT_CALIB_FREQ, SMD_ACC_DATA[ch]);
    return 1;
}

void SMD_Belt_AutoCancel(void)
{
    s_beltCalibState = BELT_CALIB_IDLE;
    s_beltSeeking    = 0;
}

void SMD_Belt_SeekZeroStart(void)
{
    const SMD_Channel ch = MOTOR_PaiFa;
    if (s_beltSeeking || s_beltCalibState != BELT_CALIB_IDLE || SMD_PWM_IsOutputOn(ch)) return;
    s_beltSeekStart = MotorPulseTotal[ch];
    s_beltSeeking   = 1;
    smd_freq_gradient[ch].dir_change = 0;
    smd_freq_gradient[ch].dir_state  = SMD_DIR_NORMAL;
    SMD_DR(ch, BELT_CALIB_DR);
    SMD_PU_DATA[ch] = BELT_ZERO_FREQ;
    SMD_PWM_SetFreqGradient(ch, BELT_ZERO_FREQ, SMD_ACC_DATA[ch]);
    /* 寄存器镜像同步成同样的 DR/PU：之后上位机写别的寄存器时槽位循环不会把皮带改停/改速 */
    mbs_hook_belt_mirror(BELT_CALIB_DR, BELT_ZERO_FREQ);
}

uint16_t SMD_Belt_GetPosition(void)
{
    const uint32_t cycle = s_beltCycleSteps;
    if (!s_beltZeroValid || cycle == 0u) return BELT_POSITION_INVALID;
    const uint32_t cur = MotorPulseTotal[MOTOR_PaiFa] - s_beltZeroPulse;   /* 无符号取模，回绕无妨 */
    const uint32_t q   = (uint32_t)(((uint64_t)cur << 16) / cycle);
    return (q >= BELT_POSITION_INVALID) ? BELT_POSITION_INVALID : (uint16_t)q;
}

void SMD_Belt_SaveIfPending(void)
{
    if (!s_beltSavePending) return;
    s_beltSavePending = 0;
    uint32_t steps = s_beltCycleSteps;
    EEPROM_WriteData(AT24Cxx_BELT_CYCLE_ADDR, (uint8_t *)&steps, 4);
}

/* 标定结束：直接停皮带（不减速），清运动状态，同 SP=0 急停；寄存器镜像由 mb_hook 同步 */
static void SMD_Belt_StopNow(void)
{
    const SMD_Channel ch = MOTOR_PaiFa;
    SMD_PWM_Stop(ch);
    SMD_PU_DATA[ch]                        = SMD_PWM_FREQ_MIN;
    smd_freq_gradient[ch].v_c              = 0.0f;
    smd_freq_gradient[ch].v_n              = 0.0f;
    smd_freq_gradient[ch].a_c              = 0.0f;
    smd_freq_gradient[ch].current_freq_int = SMD_PWM_FREQ_MIN;
    smd_freq_gradient[ch].is_running       = 0;
    smd_freq_gradient[ch].dir_change       = 0;
    smd_freq_gradient[ch].dir_state        = SMD_DIR_NORMAL;
    mbs_hook_belt_mirror(BELT_MIRROR_KEEP, SMD_PWM_FREQ_MIN);
}

/* 开机找零结束：S 曲线减速到 SMD_PWM_FREQ_MIN 即停（SMD_RunSCurve 到达后自动关 PWM） */
static void SMD_Belt_DecelStop(void)
{
    const SMD_Channel ch = MOTOR_PaiFa;
    SMD_PU_DATA[ch] = SMD_PWM_FREQ_MIN;
    SMD_PWM_SetFreqGradient(ch, SMD_PWM_FREQ_MIN, SMD_ACC_DATA[ch]);
    mbs_hook_belt_mirror(BELT_MIRROR_KEEP, SMD_PWM_FREQ_MIN);
}

static void SMD_Belt_OnRise(uint32_t pulse)
{
    s_beltZeroPulse = pulse;
    s_beltZeroValid = 1;

    if (s_beltSeeking)
    {
        s_beltSeeking = 0;
        SMD_Belt_DecelStop();
    }

    if (s_beltCalibState == BELT_CALIB_WAIT_FIRST)
    {
        s_beltCalibFirst = pulse;
        s_beltCalibRevs  = 0;
        s_beltCalibState = BELT_CALIB_COUNTING;
    }
    else if (s_beltCalibState == BELT_CALIB_COUNTING && ++s_beltCalibRevs >= BELT_CALIB_REVS)
    {
        /* BELT_CALIB_REVS 圈总步数 / 圈数，四舍五入 */
        const uint32_t steps = (pulse - s_beltCalibFirst + BELT_CALIB_REVS / 2u) / BELT_CALIB_REVS;
        s_beltCalibState = BELT_CALIB_IDLE;
        SMD_Belt_StopNow();
        if (steps > 0u)
        {
            s_beltCycleSteps  = steps;
            s_beltSavePending = 1;
        }
    }
}

/* 每 1ms：ch4 消抖 + 上升沿置零点 + 标定超时（须在 SMD_PulseCountTick 之后调用） */
static void SMD_Belt_Tick(void)
{
    const uint8_t raw = IN_READ(Incom_Ch4) ? 1u : 0u;
    if (raw != s_ch4Stable)
    {
        if (s_ch4Cnt == 0u) s_ch4EdgePulse = MotorPulseTotal[MOTOR_PaiFa];
        if (++s_ch4Cnt >= BELT_CH4_DEBOUNCE_MS)
        {
            s_ch4Stable = raw;
            s_ch4Cnt    = 0;
            if (raw) SMD_Belt_OnRise(s_ch4EdgePulse);
        }
    }
    else
    {
        s_ch4Cnt = 0;
    }

    if (s_beltCalibState != BELT_CALIB_IDLE &&
        (MotorPulseTotal[MOTOR_PaiFa] - s_beltCalibStart) > BELT_CALIB_TIMEOUT_REVS * s_beltCycleSteps)
    {
        s_beltCalibState = BELT_CALIB_IDLE;
        SMD_Belt_StopNow();
    }

    /* 开机找零超时：走了 1.5 圈仍未见黑点（ch4 未接/脏污/黑点磨掉），减速停放弃，137 保持无效 */
    if (s_beltSeeking &&
        (MotorPulseTotal[MOTOR_PaiFa] - s_beltSeekStart) > s_beltCycleSteps + s_beltCycleSteps / 2u)
    {
        s_beltSeeking = 0;
        SMD_Belt_DecelStop();
    }

    /* 标定/找零期间皮带被别的原因停掉（急停按钮触发限位停、上位机写停止等）：放弃判定，
     * 否则之后生产中皮带经过黑点时会被误当成标定/找零完成而停皮带 */
    if ((s_beltSeeking || s_beltCalibState != BELT_CALIB_IDLE) && !SMD_PWM_IsOutputOn(MOTOR_PaiFa))
        SMD_Belt_AutoCancel();
}

/* ========================= TIM10 1ms中断服务函数（S曲线调度） ========================= */
/**
 * @brief  TIM1更新/TIM10全局中断，1ms周期。
 *         TIM10 本身的初始化（PSC/ARR/NVIC）由 tim.c 的 MX_TIM10_Init() 完成，
 *         中断使能由 main.c 的 HAL_TIM_Base_Start_IT(&htim10) 完成。
 * 每1ms依次：
 *   1. 对每个通道调用 SMD_ProcessChannel()（换向/限位检测 + S曲线一步）
 *   2. 对支持步数控制的通道做步数控制决策 SMD_MotorStepsCtl()
 *   2.5 累计各通道输出脉冲总数 SMD_PulseCountTick()（皮带位置，见 SMD_GetPulseTotal）
 *   2.6 排发皮带零点 SMD_Belt_Tick()：ch4 上升沿置零点、手动标定判定
 *   3. 推进"回原点"状态机 SMD_SysToOrigin()
 *   4. 推杆电机（M1/M2双路冗余）的限位/急停保护 API_MOTOR_CheckLimit()
 *   （OUT11/12继电器料车电机方案已停用，SMD_CheckRelayMotorLimit() 不再调用）
 *
 * M6TOSERVO_ENABLE=1 时：物理6号通道（SERVO_CH）不参与第1/2/2.5步
 * （舵机不走步进逻辑），改由第3.5步调度：SMD_Servo_Tick() 推进 a→b
 * 线性插值，急停按钮触发时 SMD_Servo_AbortHold() 中止流程并保持当前角度。
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
#if M6TOSERVO_ENABLE
        if (ch == SERVO_CH) continue;  /* 舵机通道：不走步进S曲线/换向/限位逻辑 */
#endif
        SMD_ProcessChannel((SMD_Channel)ch);
    }

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
#if M6TOSERVO_ENABLE
        if (ch == SERVO_CH) continue;  /* 舵机通道：不支持步数控制 */
#endif
        SMD_MotorStepsCtl((SMD_Channel)ch);
    }

    SMD_PulseCountTick();   /* 必须在本 ms 的频率更新之后：按刚设好的 ARR 累计本 ms 输出的脉冲 */
    SMD_Belt_Tick();        /* 用本 ms 累计后的脉冲总数记 ch4 边沿 */

#if M6TOSERVO_ENABLE
    /* 舵机 a→b 线性插值推进（1ms）；急停按钮触发时中止流程并保持
     * 当前角度（与其他电机通道"急停=立即停止运动"语义一致），
     * 空闲时保持输出不变 */
    SMD_Servo_Tick();
    if (!IN_READ(Incom_Estop))
        SMD_Servo_AbortHold();
#endif

    SMD_SysToOrigin();

    API_MOTOR_CheckLimit();

    /* SMD_CheckRelayMotorLimit(); -- 已停用，见上方函数定义处注释 */
}

/*----------------------------- End of file -------------------------------*/
