/**
  * @file    api_smd.h
  * @brief   SMD API 接口头文件
  * @details 6 路步进电机 S 曲线加减速 + 步数控制。
  *          移植自 slj_stm32f407 项目 Core/Src/smd.c（8路版本），
  *          裁剪为本板 6 路通道，核心 S 曲线算法与步数控制逻辑保持一致，
  *          上位机仅需更换 Modbus 寄存器地址映射即可复用原有控制协议。
  *          另支持 M6TOSERVO_ENABLE=1 时把物理6号通道（SMD_CH5，PA1/TIM2_CH2）
  *          改造为舵机控制：Modbus 槽位6寄存器重定义为 初始角度/角度a/角度b/
  *          行程时长/启动触发/回读角度/控制口，详见本文件 M6TOSERVO_ENABLE
  *          宏定义处的说明。
  * @version V2.1.0
  * @date    22-Sep-2026
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

/* ==================== 6号步进电机舵机化开关（M6TOSERVO_ENABLE） ==================== */
/**
 * @brief 6号步进电机改造为舵机控制的编译开关
 * @note  1 = 物理6号通道（SMD_CH5，PA1/TIM2_CH2）按舵机控制。
 *        接线：PU(PA1) 输出 50Hz 舵机信号，EN6(PB0) 已硬件飞线接地（舵机
 *        使能恒有效），DR6(PC4) 由固件拉低作舵机地线，舵机电源接 5V。
 *        Modbus 槽位6（SMD_6_*_ADDR，地址50~56，SMD_SLOT_6_PHYS_CH=6）
 *        寄存器含义重新定义：
 *          +0 AM (50)  ：初始角度（0~360，输出限幅0~180），写入即保存EEPROM
 *          +1 DR (51)  ：角度a，写入即保存EEPROM（DR6引脚始终拉低作舵机地线）
 *          +2 ACC (52) ：角度b，写入即保存EEPROM
 *          +3 JRK (53) ：行程时长ms（a→b耗时，上限65535），写入即保存EEPROM
 *          +4 STEP (54)：一次性启动触发：写1执行一次 初始角→a（快速跳变）→
 *                        a到b（按行程时长线性插值）→初始角（快速跳变）
 *          +5 PU (55)  ：只读，回读当前实际角度
 *          +6 SP (56)  ：控制口：写1=启动（同STEP=1），写0=停止自动化流程并
 *                        回到初始角；回读 1=流程运行中 / 0=空闲
 *        角度-脉宽映射与旧板 slj_stm32f407 舵机测试一致（正极性输出）：
 *        0°=0.5ms、180°=2.5ms 高电平脉宽，超过180°按180°输出。
 *        开机从EEPROM读参数（全0/全0xFF视为未初始化→写默认值：
 *        初始角120/角度a 120/角度b 50/行程时长5500ms），并直接输出初始角度PWM。
 *        0 = 维持原步进电机控制逻辑，与改动前完全一致。
 */
#define M6TOSERVO_ENABLE        1

#if M6TOSERVO_ENABLE
#define SERVO_CH                SMD_CH5      /* 舵机占用的物理通道（6号接口） */
#define SERVO_PWM_FREQ_HZ       50u          /* 舵机信号频率 50Hz */
#define SERVO_TIM_CNT_HZ        SMD_COUNTER_FREQ            /* 计数频率 1MHz（复用SMD参数） */
#define SERVO_PWM_PSC           (84000000u / SERVO_TIM_CNT_HZ - 1u)  /* 83：TIM2挂APB1 84MHz */
#define SERVO_PWM_ARR           (SERVO_TIM_CNT_HZ / SERVO_PWM_FREQ_HZ - 1u) /* 19999：20ms周期 */
/* 舵机信号反相开关：1 = PA1 经反相光耦（TLP2362，LED亮→输出低，输出侧上拉到5V）
 * 再到舵机信号线，MCU 引脚须输出反相波形（低电平宽度=0.5~2.5ms），经光耦再反一次
 * 后舵机端才是正脉冲；0 = PA1 直连舵机信号线（旧接法），MCU 直接输出正脉冲 */
#define SERVO_PWM_INVERT        1
/* 【临时测试】舵机自动往复开关：1 = 上电后无需上位机指令，舵机端脉宽自动在
 * 0.5ms(0°)~2.5ms(180°) 之间匀速来回扫（单程 SERVO_AUTO_SWEEP_HALF_MS），
 * 此时 Modbus 的启动/停止指令对舵机无效；急停按钮按下时暂停。
 * 测试完务必改回 0，恢复正常的 初始角→a→b→初始角 流程 */
#define SERVO_AUTO_SWEEP_TEST       1
#define SERVO_AUTO_SWEEP_HALF_MS    2000u    /* 单程（0°→180° 或 180°→0°）耗时 ms */
#define SERVO_ANGLE_MAX         360u         /* 寄存器允许写入的角度上限 */
#define SERVO_ANGLE_PWM_MAX     180u         /* 脉宽映射上限（与上位机 clampAngle 一致） */
#define SERVO_DEFAULT_INIT_ANGLE   120u      /* 默认：初始角度 */
#define SERVO_DEFAULT_ANGLE_A      120u      /* 默认：角度a */
#define SERVO_DEFAULT_ANGLE_B      50u       /* 默认：角度b */
#define SERVO_DEFAULT_DURATION_MS   5500u    /* 默认：行程时长ms */
#endif

/* 步数控制刹车参数 */
#define BRAKE_TARGET_HZ         100u    // 刹车目标速度 Hz
#define BRAKE_BUFFER            100u    // 巡航缓冲步数（100Hz x 1s）

/* 导出类型定义 ------------------------------------------------------------*/

/**
 * @brief 6路PWM通道枚举
 * @note  每个通道对应固定的GPIO口和定时器通道（不可随意修改），
 *        同时与 main.h 中 SMD_DR(i)/SMD_EN(i)/SMD_AM(i) 的索引 i 一一对应。
 *        通道<->具体机械功能的绑定（2026-09 与上位机核对后的最新接线）：
 *        排发/送发/进退/夹爪移动/送发辅助/舵机。
 */
typedef enum {
    SMD_CH0 = 0,  // PA7 - TIM14_CH1 (普通PWM通道) / SMD_DR_1,EN_1,AM_1
    MOTOR_PaiFa = SMD_CH0,          // 排发电机
    SMD_CH1 = 1,  // PA6 - TIM13_CH1 (普通PWM通道) / SMD_DR_2,EN_2,AM_2
    MOTOR_Trans = SMD_CH1,          // 送发（输送）电机
    SMD_CH2 = 2,  // PA5 - TIM8_CH1N (高级定时器互补通道) / SMD_DR_3,EN_3,AM_3
    MOTOR_FBack = SMD_CH2,          // 进退电机（步数控制）
    SMD_CH3 = 3,  // PA3 - TIM5_CH4 (普通PWM通道) / SMD_DR_4,EN_4,AM_4
    MOTOR_GripperMove = SMD_CH3,    // 夹爪移动电机（步数控制），原升降电机位置，升降电机已取消
    SMD_CH4 = 4,  // PA2 - TIM9_CH1 (普通PWM通道) / SMD_DR_5,EN_5,AM_5
                                     // 送发辅助电机：上位机槽位8（MOTOR_8）转接到本通道，
                                     // 详见 mb_hook.h 的 SMD_SLOT_x_PHYS_CH 系列宏
    SMD_CH5 = 5,  // PA1 - TIM2_CH2 (普通PWM通道) / SMD_DR_6,EN_6,AM_6
                                     // 舵机：M6TOSERVO_ENABLE=1 时本通道为舵机控制，
                                     // PU 输出 50Hz 舵机信号，EN 脚硬件飞线接地
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

#if M6TOSERVO_ENABLE
/* 以下舵机接口仅 M6TOSERVO_ENABLE=1 时编译，寄存器语义见本文件
 * M6TOSERVO_ENABLE 宏定义处的说明 */

/**
  * @brief  舵机初始化
  * @note   TIM2_CH2 配置为 50Hz PWM 基时（PSC=83/ARR=19999）；从 EEPROM 载入
  *         参数（全0/全0xFF 视为未初始化→写入默认值并保存）；DR6(PC4) 固定
  *         输出低电平作为舵机地线（EN6 已硬件飞线接地）；并直接按初始角度
  *         输出 PWM。
  *         须在 EEPROM_Init() 之后调用。
  */
void SMD_Servo_Init(void);

/**
  * @brief  舵机 1ms 调度（TIM10 中断中调用）：推进 a→b 线性插值
  */
void SMD_Servo_Tick(void);

/**
  * @brief  启动一次完整流程：初始角→a（快速跳变）→a到b（按行程时长线性
  *         插值）→初始角（快速跳变）。流程运行中调用会被忽略（由 mb_hook.c
  *         的触发条件保证，需先 SP=0 停止后才能重新触发）。
  */
void SMD_Servo_StartSequence(void);

/**
  * @brief  停止自动化流程并快速回到初始角度（SP=0 控制口语义）
  * @note   空闲时调用也只会跳到初始角度，无副作用
  */
void SMD_Servo_StopReturnInitial(void);

/**
  * @brief  急停：中止当前流程，保持当前角度输出不动（急停按钮/全局急停寄存器）
  */
void SMD_Servo_AbortHold(void);

/**
  * @brief  设置/获取配置参数（设置函数内部限幅并立即写入EEPROM保存）
  * @param  angle  角度值（0~360，超过按360；输出时按0~180限幅映射脉宽）
  * @param  ms     行程时长（a→b耗时，毫秒）
  */
void     SMD_Servo_SetInitialAngle(uint16_t angle);
void     SMD_Servo_SetAngleA(uint16_t angle);
void     SMD_Servo_SetAngleB(uint16_t angle);
void     SMD_Servo_SetDurationMs(uint16_t ms);
uint16_t SMD_Servo_GetInitialAngle(void);
uint16_t SMD_Servo_GetAngleA(void);
uint16_t SMD_Servo_GetAngleB(void);
uint16_t SMD_Servo_GetDurationMs(void);

/**
  * @brief  获取舵机当前实际角度（已按0~180限幅，PU寄存器回读用）
  */
uint16_t SMD_Servo_GetCurrentAngle(void);

/**
  * @brief  舵机流程是否运行中（SP寄存器回读用）
  * @retval 1=运行中，0=空闲
  */
uint8_t SMD_Servo_IsRunning(void);
#endif

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
  * @brief  读取指定通道"已输出脉冲总数"（自由运行的 32 位无符号计数，只增不清，不分方向）
  * @note   一个脉冲 = 驱动器收到的一个 PU 脉冲（与上位机 feedMmPerStep 的"步"同义）。
  *         回绕无妨：使用方应做无符号取模差分 (uint32_t)(now - prev)。
  *         皮带（排发电机，MOTOR_PaiFa）的该值通过 Modbus 135/136 只读回读。
  */
uint32_t SMD_GetPulseTotal(SMD_Channel ch);

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
