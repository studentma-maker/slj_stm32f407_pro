/**
  * @file    api_smd.h
  * @brief   SMD API 接口头文件
  * @details 6 路步进电机 S 曲线加减速 + 步数控制。
  *          移植自 slj_stm32f407 项目 Core/Src/smd.c（8路版本），
  *          裁剪为本板 6 路通道，核心 S 曲线算法与步数控制逻辑保持一致，
  *          上位机仅需更换 Modbus 寄存器地址映射即可复用原有控制协议。
  * @version V2.0.0
  * @date    09-Sep-2026
  */
#ifndef __API_SMD_H__
#define __API_SMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 导入标准库及 HAL 库头文件 ------------------------------------------------*/
#include "main.h"

/* 导出常量定义 ------------------------------------------------------------*/

/**
 * @brief PWM频率范围
 * @note  SMD_PWM_FREQ_MIN 必须为 1，不可改为 0，同时具有两层含义：
 *
 *  【硬件约束】SMD_Calc_PSC_ARR() 和 SMD_PWM_SetDuty() 内部有除法，
 *        freq=0 会导致除零硬件异常（HardFault）。
 *
 *  【停止语义】当目标频率 v_n 被设为 SMD_PWM_FREQ_MIN（1Hz）时，
 *        中断在 v_c 减速到达 1Hz 后会自动调用 SMD_PWM_Stop() 并置
 *        is_running=0，电机真正停止输出，不会出现"1步/秒蠕动"现象。
 *        因此，"S曲线减速停止"的正确用法是将目标频率写为 1（即 PU=1），
 *        而不是写为 0。直接急停请写 SP=0。
 */
#define SMD_PWM_FREQ_MIN        1u      // 最低频率 Hz（禁止改为0，见上方说明）
#define SMD_PWM_FREQ_MAX        8000u   // 最高频率 Hz
#define SMD_COUNTER_FREQ        1000000u // 定时器计数频率 1MHz

/**
 * @brief S曲线加速度（ACC）参数范围
 * @note  ACC 含义为"加速度的上限值"（Hz/s）。
 *        S曲线过程：a_c 以 JERK 速率从 0 增大到 ACC_MAX，
 *        到达 ACC_MAX 后进入匀加速段，接近目标时再对称减速。
 */
#define SMD_ACC_MAX_MIN         1u
#define SMD_ACC_MAX_MAX         8000u
#define SMD_ACC_MAX_DEFAULT     4000u

/**
 * @brief S曲线 Jerk 参数范围
 * @note  Jerk 含义为"加速度的变化率"（Hz/s²），决定S曲线的"柔和程度"。
 */
#define SMD_JERK_MIN            1u
#define SMD_JERK_MAX            15000u
#define SMD_JERK_DEFAULT        8000u

/* TIM10 中断周期 */
#define SMD_UPDATE_DT_s         0.001f  // 1ms = 0.001s
#define SMD_UPDATE_DT_ms        1u      // 1ms

/* 步数控制刹车参数 */
#define BRAKE_TARGET_HZ         100u    // 刹车目标速度 Hz
#define BRAKE_BUFFER            100u    // 巡航缓冲步数（100Hz x 1s）

/* 导出类型定义 ------------------------------------------------------------*/

/**
 * @brief 6路PWM通道枚举
 * @note  每个通道对应固定的GPIO口和定时器通道（不可随意修改），
 *        同时与 main.h 中 SMD_DR(i)/SMD_EN(i)/SMD_AM(i) 的索引 i 一一对应。
 *        通道<->具体机械功能的绑定为本次移植时的假设，需与实际线束核对：
 *        与旧项目（slj_stm32f407）保持同样的顺序：排发/送发/进退/升降/夹爪/上料。
 */
typedef enum {
    SMD_CH0 = 0,  // PA7 - TIM14_CH1 (普通PWM通道) / SMD_DR_1,EN_1,AM_1
    MOTOR_PaiFa = SMD_CH0,          // 排发电机
    SMD_CH1 = 1,  // PA6 - TIM13_CH1 (普通PWM通道) / SMD_DR_2,EN_2,AM_2
    MOTOR_Trans = SMD_CH1,          // 送发（输送）电机
    SMD_CH2 = 2,  // PA5 - TIM8_CH1N (高级定时器互补通道) / SMD_DR_3,EN_3,AM_3
    MOTOR_FBack = SMD_CH2,          // 进退电机（步数控制）
    SMD_CH3 = 3,  // PA3 - TIM5_CH4 (普通PWM通道) / SMD_DR_4,EN_4,AM_4
    MOTOR_UpDown = SMD_CH3,         // 升降电机（步数控制）
    SMD_CH4 = 4,  // PA2 - TIM9_CH1 (普通PWM通道) / SMD_DR_5,EN_5,AM_5
    MOTOR_GripperMove = SMD_CH4,    // 夹爪移动电机（步数控制）
    SMD_CH5 = 5,  // PA1 - TIM2_CH2 (普通PWM通道) / SMD_DR_6,EN_6,AM_6
    MOTOR_Feed = SMD_CH5,           // 上料电机
    SMD_CH_MAX
} SMD_Channel;

/* ========================= 方向切换状态 ========================= */
typedef enum
{
    SMD_DIR_NORMAL = 0,  // 正常运行
    SMD_DIR_DECEL  = 1,  // 正在减速到0（换向中）
    SMD_DIR_WAIT   = 2,  // 已到0，等待GPIO切换完成
} SMD_DirState;

/**
 * @brief 单个PWM通道的S曲线运动状态
 * @note  v_c/v_n 单位 Hz（浮点），a_c 单位 Hz/s，jerk 为运行时动态值。
 *        所有计算/判断使用浮点型变量，仅在更新PWM硬件时转换为整型。
 */
typedef struct {
    float    v_c;               // 当前速度（浮点）
    float    a_c;               // 当前加速度
    float    v_n;               // 目标速度
    float    jerk;              // 当前jerk值（运行时动态值）
    float    acc_max;           // 本次运动的加速度上限（Hz/s），由调用方传入

    uint8_t      dir_change;    // 1=需要换向
    SMD_DirState dir_state;     // 换向子状态机

    uint32_t current_freq_int;  // 当前整型频率（给CCR/PSC/ARR计算用）
    uint8_t  is_running;        // 渐变是否激活（1=激活）
} SMD_Freq_Gradient;

/**
 * @brief 单通道步数控制状态
 */
typedef struct
{
    uint8_t  is_running;       /* 0=idle, 1=步数控制运行中 */
    uint16_t targetSteps;      /* 目标步数（绝对位置） */
    uint8_t  braking;          /* 1=正在S曲线刹车中，禁止重复决策 */
} SMD_StepsCtl_t;

/**
 * @brief 系统上电回原点状态机
 */
typedef enum
{
    defaultset = 0,         // 初始化继电器默认状态
    gripperSetDR,           // 设置夹爪电机的方向
    waitPressUP,            // 等待下压和送发气缸升起
    gripperSettoOriginPU,   // 设置夹爪电机脉冲
    waitToOrigin,           // 等待夹爪到达原点
    toOriginSuccess,        // 复位完成
    GrippertoOrigin_P_MAX
} GrippertoOrigin_P;

/* 导出变量声明 ------------------------------------------------------------*/
extern SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX];
extern uint16_t SMD_PU_DATA[SMD_CH_MAX];    // 目标频率 Hz / 步数模式最大脉冲频率
extern uint16_t SMD_ACC_DATA[SMD_CH_MAX];   // 最大加速度 Hz/s
extern uint16_t SMD_JERK_DATA[SMD_CH_MAX];  // Jerk Hz/s²，用户可通过Modbus自定义
extern uint16_t MotorCurStepsU[SMD_CH_MAX]; // 各通道当前步数（整数部分）
extern SMD_StepsCtl_t g_motorStepsCtl[SMD_CH_MAX];
extern GrippertoOrigin_P g_sysToOrigin;

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  设置指定通道的目标频率，启动S曲线渐变
  * @param  ch: 目标通道
  * @param  target_freq: 目标频率 Hz，范围 SMD_PWM_FREQ_MIN ~ SMD_PWM_FREQ_MAX
  * @param  accel: 本次运动的最大加速度 Hz/s，范围 SMD_ACC_MAX_MIN ~ SMD_ACC_MAX_MAX
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t accel);

/**
  * @brief  停止指定通道的频率渐变（不停PWM硬件，仅清运动状态）
  */
void SMD_PWM_StopFreqGradient(SMD_Channel ch);

/**
  * @brief  启动/停止指定通道的PWM硬件输出
  */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch);
HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch);

/**
  * @brief  直接设置频率（阻塞，不经过S曲线），用于SP寄存器直接跳变
  */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq);

/**
  * @brief  根据当前速度/加速度反推减速到目标速度所需的步数
  * @note   移植自旧项目 SMD_CalcAccNeedSteps()，用于步数控制的提前刹车决策
  */
int32_t SMD_CalcAccNeedSteps(float v_current, float a_current,
                              float v_target, float acc_max, float jerk);

#ifdef __cplusplus
}
#endif

#endif /* __API_SMD_H__ */

/*----------------------------- End of file -------------------------------*/
