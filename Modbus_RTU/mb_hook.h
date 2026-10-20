/**
  * @file    mb_hook.h
  * @brief   modbus 主机/从机回调函数接口头文件
  * @details 声明 modbus 主机接收回调与从机寄存器读写钩子函数，
  *          并定义 modbus 寄存器地址映射。
  * @version V1.0.0
  * @date    20-Jun-2025
  */
#ifndef _MB_HOOK_H
#define _MB_HOOK_H

#include "main.h"
#include "mb_slave.h"
#include "mb_fwupd.h"

/* 值<->功能码：WRITE_HOLDING_V 保持不变，上位机批量写寄存器时用该值标记
 * "此寄存器保持不变"，下位机收到该值直接跳过，避免误改（沿用旧项目约定） */
#define WRITE_HOLDING_V            2

/* modbus 寄存器地址定义 ----------------------------------------------------*
 * 每路电机占用 10 个连续寄存器（基址 = 电机序号*10），字段顺序与地址
 * 均与旧板 slj_stm32f407 完全一致：
 *   +0 AM  - 报警信号（只读）
 *   +1 DR  - 方向控制（读写，非步数模式下写入触发S曲线换向）
 *   +2 ACC - 最大加速度 Hz/s（读写，所有模式下均可更新）
 *   +3 JRK - Jerk Hz/s²（读写，S曲线加加速度，写0恢复默认值）
 *   +4 STEP- 步数控制目标步数（写入触发步数控制模式；须在 PU 之前写入，
 *            多寄存器连续写入时先触发步数模式，PU 随后按"仅更新最大频率"处理）
 *   +5 PU  - 目标频率 / 步数模式最大脉冲频率 Hz（非步数模式写入触发S曲线渐变）
 *   +6 SP  - 实时速度（读=当前Hz；写0=急停，写N>1=直接跳变）
 *   +7~9 — - 预留
 *
 * 未移植 EN（驱动使能）/ CS（当前步数）两个字段：
 *   - EN：不是所有电机都需要通过 Modbus 控制驱动器使能，暂不在寄存器表中
 *     暴露；SMD_EN/SMD_EN_READ 宏仍保留在 main.h，硬件层面按 GPIO 复位后的
 *     默认电平工作，如后续确认驱动器使能极性，可再决定是否需要固件主动置位。
 *   - CS：并非所有电机都支持步数控制，因此不作为逐路通用字段。仅
 *     MOTOR_FBack / MOTOR_GripperMove 两路支持步数控制的电机，各自使用
 *     扩展状态区的专用寄存器回读当前步数（见下方 GRIPPER_CUR_STEPS_ADDR /
 *     FBACK_CUR_STEPS_ADDR，地址与旧板一致）。
 *
 * 上位机把电机寄存器分成 8 个槽位（对应旧板 8 路电机），但新板只有 6 路
 * 物理通道（其中6号接口已改造为舵机，见 api_smd.h 的 M6TOSERVO_ENABLE）。
 * 槽位地址（SMD_1~8_xxx_ADDR / SMD_SLOT_xxx_ADDR(slot)）与
 * 物理通道（SMD_CH0~SMD_CH5）之间的转接关系由本文件下方的
 * SMD_SLOT_x_PHYS_CH 系列宏统一定义，改接线只需要改那几个数字。
 *------------------------------------------------------------------------*/
#define SMD_1_AM_ADDR              0        /* 报警信号 */
#define SMD_1_DR_ADDR              1        /* 方向控制 */
#define SMD_1_ACC_ADDR             2        /* 加速度控制 */
#define SMD_1_JRK_ADDR             3        /* Jerk控制 */
#define SMD_1_STEP_ADDR            4        /* 步数控制 */
#define SMD_1_PU_ADDR              5        /* 频率控制 */
#define SMD_1_SP_ADDR              6        /* 实时速度控制 */

#define SMD_2_AM_ADDR              10       /* 报警信号 */
#define SMD_2_DR_ADDR              11       /* 方向控制 */
#define SMD_2_ACC_ADDR             12       /* 加速度控制 */
#define SMD_2_JRK_ADDR             13       /* Jerk控制 */
#define SMD_2_STEP_ADDR            14       /* 步数控制 */
#define SMD_2_PU_ADDR              15       /* 频率控制 */
#define SMD_2_SP_ADDR              16       /* 实时速度控制 */

#define SMD_3_AM_ADDR              20       /* 报警信号 */
#define SMD_3_DR_ADDR              21       /* 方向控制 */
#define SMD_3_ACC_ADDR             22       /* 加速度控制 */
#define SMD_3_JRK_ADDR             23       /* Jerk控制 */
#define SMD_3_STEP_ADDR            24       /* 步数控制 */
#define SMD_3_PU_ADDR              25       /* 频率控制 */
#define SMD_3_SP_ADDR              26       /* 实时速度控制 */

#define SMD_4_AM_ADDR              30       /* 报警信号 */
#define SMD_4_DR_ADDR              31       /* 方向控制 */
#define SMD_4_ACC_ADDR             32       /* 加速度控制 */
#define SMD_4_JRK_ADDR             33       /* Jerk控制 */
#define SMD_4_STEP_ADDR            34       /* 步数控制 */
#define SMD_4_PU_ADDR              35       /* 频率控制 */
#define SMD_4_SP_ADDR              36       /* 实时速度控制 */

#define SMD_5_AM_ADDR              40       /* 报警信号 */
#define SMD_5_DR_ADDR              41       /* 方向控制 */
#define SMD_5_ACC_ADDR             42       /* 加速度控制 */
#define SMD_5_JRK_ADDR             43       /* Jerk控制 */
#define SMD_5_STEP_ADDR            44       /* 步数控制 */
#define SMD_5_PU_ADDR              45       /* 频率控制 */
#define SMD_5_SP_ADDR              46       /* 实时速度控制 */

#define SMD_6_AM_ADDR              50       /* 报警信号 */
#define SMD_6_DR_ADDR              51       /* 方向控制 */
#define SMD_6_ACC_ADDR             52       /* 加速度控制 */
#define SMD_6_JRK_ADDR             53       /* Jerk控制 */
#define SMD_6_STEP_ADDR            54       /* 步数控制 */
#define SMD_6_PU_ADDR              55       /* 频率控制 */
#define SMD_6_SP_ADDR              56       /* 实时速度控制 */

/* 槽位7、8：新板没有对应的物理通道，但上位机协议（reg_map.h）里这两组地址
 * 依然存在（MOTOR_7 保留未用，MOTOR_8 被 onControlFeedSpeed() 用作"送发辅助
 * 电机"，转接到5号物理接口），必须占住这两段地址空间，写入才能落到正确的
 * 寄存器上 */
#define SMD_7_AM_ADDR              60       /* 报警信号（未使用） */
#define SMD_7_DR_ADDR              61
#define SMD_7_ACC_ADDR             62
#define SMD_7_JRK_ADDR             63
#define SMD_7_STEP_ADDR            64
#define SMD_7_PU_ADDR              65
#define SMD_7_SP_ADDR              66

#define SMD_8_AM_ADDR              70       /* 报警信号 */
#define SMD_8_DR_ADDR              71       /* 方向控制 */
#define SMD_8_ACC_ADDR             72       /* 加速度控制 */
#define SMD_8_JRK_ADDR             73       /* Jerk控制 */
#define SMD_8_STEP_ADDR            74       /* 步数控制 */
#define SMD_8_PU_ADDR              75       /* 频率控制 */
#define SMD_8_SP_ADDR              76       /* 实时速度控制 */

/* 通用槽位地址函数宏：slot = 0~7（对应上位机的电机1~8），
 * 效果和上面 8 组具名宏一一对应，供 mb_hook.c 里按槽位遍历时使用 */
#define SMD_SLOT_REG_BASE(slot)    ((slot) * 10)
#define SMD_SLOT_AM_ADDR(slot)     (SMD_SLOT_REG_BASE(slot) + 0)
#define SMD_SLOT_DR_ADDR(slot)     (SMD_SLOT_REG_BASE(slot) + 1)
#define SMD_SLOT_ACC_ADDR(slot)    (SMD_SLOT_REG_BASE(slot) + 2)
#define SMD_SLOT_JRK_ADDR(slot)    (SMD_SLOT_REG_BASE(slot) + 3)
#define SMD_SLOT_STEP_ADDR(slot)   (SMD_SLOT_REG_BASE(slot) + 4)
#define SMD_SLOT_PU_ADDR(slot)     (SMD_SLOT_REG_BASE(slot) + 5)
#define SMD_SLOT_SP_ADDR(slot)     (SMD_SLOT_REG_BASE(slot) + 6)

/* ==================== 寄存器槽位(1~8) -> 板上物理通道(1~6)接线映射 ====================
 * 上位机 Modbus 协议按旧板 8 路电机划分寄存器槽位（slj_kickpi_qt_pro 的
 * common/reg_map.h），新板只有 6 路物理通道。改接线只需要改下面这 8 个
 * 数字，不用碰 mb_hook.c 里的任何逻辑。取值 1~6 表示接到板上第几路（对应
 * api_smd.h 的 SMD_CH0~SMD_CH5，下标=数值-1）；取值 0 表示该槽位没有实际
 * 接线，下位机忽略对应寄存器的写入。
 *
 * 当前取值来自与你确认过的接线关系（2026-09 核对上位机 mainworker.cpp 源码）：
 *   - 槽位1（MOTOR_PaiFa 自身地址）：上位机 onControlMotor() 里排发电机的命令
 *     被重映射发到了槽位5（MOTOR_Feed）的地址，槽位1这段地址上位机实际不发，
 *     设为 0（不接线），避免和槽位5同时驱动1号通道产生冲突
 *   - 槽位2（MOTOR_Trans 送发）：上位机直接用自己的地址发送，不重映射 → 直连2号
 *   - 槽位3（MOTOR_FBack 进退）：上位机直连，不重映射 → 直连3号
 *   - 槽位4（MOTOR_GripperMove 夹爪移动）：上位机直连，不重映射 → 直连4号
 *     （升降电机已取消，原 MOTOR_UpDown 槽位改由夹爪移动电机使用）
 *   - 槽位5（MOTOR_Feed 上料）：上位机排发电机命令实际发到这里 → 转接到1号
 *   - 槽位6：舵机。6号物理接口已改造为舵机（api_smd.h 的 M6TOSERVO_ENABLE），
 *     本槽位寄存器语义全部重定义（地址50~56）：
 *       AM(50)=初始角度、DR(51)=角度a、ACC(52)=角度b、JRK(53)=行程时长ms
 *       （角度写入即保存EEPROM，行程时长仅内存生效）；STEP(54)=写1启动一次流程（初始角→a快速
 *       跳变→a到b按行程时长线性插值→在b停留1000ms→初始角快速跳变）；PU(55)=回读当前角度/
 *       写入手动跳到该角度；SP(56)=写1启动/写0停止并回初始角
 *   - 槽位7：上位机未使用 → 不接线
 *   - 槽位8（MOTOR_8，onControlFeedSpeed 里的送发辅助电机）→ 转接到5号
 */
#define SMD_SLOT_1_PHYS_CH        0   /* 排发自身地址，上位机实际不发，不接线 */
#define SMD_SLOT_2_PHYS_CH        2   /* 送发 Trans，直连2号 */
#define SMD_SLOT_3_PHYS_CH        3   /* 进退 FBack，直连3号 */
#define SMD_SLOT_4_PHYS_CH        4   /* 夹爪移动 GripperMove，直连4号 */
#define SMD_SLOT_5_PHYS_CH        1   /* 上料 Feed：排发命令实际落在这里，转接到1号 */
#define SMD_SLOT_6_PHYS_CH        6   /* 舵机：转接到6号，寄存器语义见上方 M6TOSERVO_ENABLE 说明 */
#define SMD_SLOT_7_PHYS_CH        0   /* 未使用，不接线 */
#define SMD_SLOT_8_PHYS_CH        5   /* 送发辅助电机，转接到5号 */

#define SMD_SLOT_COUNT             8

#define OUT_1_ADDR                 80      /* 输出信号，地址与旧板 slj_stm32f407 一致 */
#define OUT_2_ADDR                 81      /* 输出信号 */
#define OUT_3_ADDR                 82      /* 输出信号 */
#define OUT_4_ADDR                 83      /* 输出信号 */
#define OUT_5_ADDR                 84      /* 输出信号 */
#define OUT_6_ADDR                 85      /* 输出信号 */
#define OUT_7_ADDR                 86      /* 输出信号 */
#define OUT_8_ADDR                 87      /* 输出信号 */
#define OUT_9_ADDR                 88      /* 输出信号 */
#define OUT_10_ADDR                89      /* 输出信号 */
#define OUT_11_ADDR                90      /* 输出信号 */
#define OUT_12_ADDR                91      /* 输出信号 */

/* 92~95：兼容旧板"继电器控制料车电机"协议，地址与旧板 OUT_13~16_ADDR 一致。
 * 旧板上料车电机由 4 路继电器驱动：13/14 是主继电器对（上升/下降），
 * 15/16 是上位机对同一信号的冗余复制（onControlRelayMotor() 里 CartUp/
 * CartDown 两个方向永远同时设置这4个寄存器，13与15同值、14与16同值，
 * 提高继电器触点的过流能力），四者最终只对应同一台电机：
 *   13=1,14=0（15/16同步）→ 正转；13=0,14=1（15/16同步）→ 反转；
 *   全为0（或13=14=1的非法组合）→ 停止
 * 新板没有这组物理继电器，这 4 个寄存器只是协议兼容层：写入后由
 * mbs_hook_extract_holding() 统一解码（以13/14为准），转换成对 M1 的
 * API_MOTOR_SetSpeed(...,100/-100/0) 调用，不驱动 M2，也不驱动任何
 * 实际 GPIO（OUT_READ()/OUT() 不覆盖这 4 个地址，新板 OUT 只有 1~12）。 */
#define OUT_13_ADDR                92      /* 料车正转继电器位（协议兼容，无实际GPIO） */
#define OUT_14_ADDR                93      /* 料车反转继电器位（协议兼容，无实际GPIO） */
#define OUT_15_ADDR                94      /* 料车正转继电器位冗余复制（协议兼容，无实际GPIO） */
#define OUT_16_ADDR                95      /* 料车反转继电器位冗余复制（协议兼容，无实际GPIO） */

/* M1/M2 直流电机调速：新增的直接调速寄存器（-100~100），与上面 13~16
 * 继电器兼容寄存器共用同一套 API_MOTOR_SetSpeed() 底层实现 */
#define MOTOR_1_TARGET_SP_ADDR     100     /* 电机1目标速度（可读可写） */
#define MOTOR_1_CURRENT_SP_ADDR    101     /* 电机1当前速度（只读） */
#define MOTOR_2_TARGET_SP_ADDR     102     /* 电机2目标速度（可读可写） */
#define MOTOR_2_CURRENT_SP_ADDR    103     /* 电机2当前速度（只读） */

/* 104~111 预留 */

#define IN_1_ADDR                  112      /* 输入信号，地址与旧板 slj_stm32f407 一致 */
#define IN_2_ADDR                  113      /* 输入信号 */
#define IN_3_ADDR                  114      /* 输入信号 */
#define IN_4_ADDR                  115      /* 输入信号 */
#define IN_5_ADDR                  116      /* 输入信号 */
#define IN_6_ADDR                  117      /* 输入信号 */
#define IN_7_ADDR                  118      /* 输入信号 */
#define IN_8_ADDR                  119      /* 输入信号 */
#define IN_9_ADDR                  120      /* 输入信号 */
#define IN_10_ADDR                 121      /* 输入信号 */
#define IN_11_ADDR                 122      /* 输入信号 */
#define IN_12_ADDR                 123      /* 输入信号 */
#define IN_13_ADDR                 124      /* 输入信号 */
#define IN_14_ADDR                 125      /* 输入信号 */
#define IN_15_ADDR                 126      /* 输入信号 */
#define IN_16_ADDR                 127      /* 输入信号 */
#define IN_17_ADDR                 128      /* 输入信号 */
#define IN_18_ADDR                 129      /* 输入信号 */
#define IN_19_ADDR                 130      /* 输入信号 */
#define IN_20_ADDR                 131      /* 输入信号 */

/* 步数只读回读：旧板同样只为夹爪电机、进退电机两路开了专用寄存器，
 * 地址与旧板完全一致 */
#define GRIPPER_CUR_STEPS_ADDR     132     /* 夹爪电机当前步数（只读） */
#define SYS_TO_ORIGIN_ADDR         133     /* 系统上电回原点状态（只读，对应 GrippertoOrigin_P 枚举） */
#define FBACK_CUR_STEPS_ADDR       134     /* 进退电机当前步数（只读） */

/* 皮带（排发电机 MOTOR_PaiFa，物理通道 CH0）已输出脉冲总数，32 位拆成两个 16 位寄存器（只读）：
 *   135 = 高 16 位，136 = 低 16 位（与 W1110/编码器的"先高后低"约定一致）
 * 自由运行、不清零；正转（BELT_CALIB_DR）累加、反转累减；一个脉冲 = 驱动器收到的一个 PU 脉冲。
 * 上位机做取模差分后按有符号数解释（负 = 反转），不需要（也不应该）清零。 */
#define BELT_PULSE_HIGH_ADDR       135
#define BELT_PULSE_LOW_ADDR        136

/* 皮带一圈进度（只读）：16 位定点数，值/65536 ∈ [0,1)，0xFFFF = 无效
 * （开机后尚未经过 ch4 黑点 / 正反向超出一圈未见黑点）。黑点前沿处归零（正转 ch4 上升沿 /
 * 反转 ch4 下降沿），应答轮询前按 (当前脉冲 − 零点脉冲) / 一圈步数 现算，反转越过零点退回上一圈
 * （进度接近 1、圈计数 −1），见 api_smd.h 皮带零点说明 */
#define BELT_POSITION_ADDR         137

/* 皮带圈计数（只读，辊筒偏心补偿用，见 api_smd.h"断电前位置记录 + 圈计数"）：
 *   138 = 圈计数 0~BELT_REV_WRAP-1（每接受一个零点加经过的整圈数，反转时减；开机零点与断电前停稳位置
 *         吻合时接着断电前计数），BELT_REV_COUNT_INVALID = 尚无零点
 *   139 = 状态位 BELT_ST_*（零点有效 / 零点已与断电前位置核对 / 核对中 / 核对失败 / 开机无记录）
 *   140 = 圈计数纪元：计数链断开（开机无记录、核对失败、核对中反转）一次 +1，EEPROM 持久。
 *         上位机标定辊筒相位时记下纪元，纪元变了说明圈号不再连续，须重测相位 */
#define BELT_REV_COUNT_ADDR        138
#define BELT_STATUS_ADDR           139
#define BELT_REV_EPOCH_ADDR        140

/* 皮带一圈步数手动标定（写1触发，处理后自动清零）：仅排发皮带停止时接受，
 * 皮带以 BELT_CALIB_FREQ、BELT_CALIB_DR 连续转 BELT_CALIB_REVS 整圈，一圈步数取多圈平均，
 * 数满后在 ch4 上升沿处直接停并写 EEPROM；
 * 标定期间上位机改写排发电机寄存器或写全部急停 → 中止标定 */
#define BELT_PULSE_CYCLE_ADDR      198

#define STOP_ALL_MOTOR_ADDR        (REG_HOLDING_NREGS - 1) /* 全部步进电机急停（写1触发） */

/* 导出函数声明 ------------------------------------------------------------*/

/**
  * @brief  主机模式下接收到从机不同功能码回复的回调处理
  * @param  add      从机地址
  * @param  data     接收到的从机数据指针
  * @param  datalen  接收到的数据长度
  * @retval None
  */
/**
  * @brief  下位机自己启停皮带（开机找零/标定）时由 api_smd.c 调用（可在中断里）：把排发电机所在
  *         槽位的 DR/PU 寄存器镜像同步为实际值。镜像与实际不一致时，上位机下一帧任意写入会让槽位
  *         循环按镜像把皮带改速/拉起/停下（停止时 PU 镜像要置 SMD_PWM_FREQ_MIN，同 SP=0）
  * @param  dr  方向镜像；BELT_MIRROR_KEEP 表示不改
  * @param  pu  频率镜像
  */
#define BELT_MIRROR_KEEP  0xFFFFu
void mbs_hook_belt_mirror(uint16_t dr, uint16_t pu);

/* 应用程序固件信息（版本见 fw_version.h），定义在 mb_hook.c */
extern const fwu_info_t g_fwuInfo;

void mbh_hook_rec01(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec02(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec03(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec04(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec05(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec06(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec15(uint8_t add, uint8_t *data, uint8_t datalen);
void mbh_hook_rec16(uint8_t add, uint8_t *data, uint8_t datalen);

/**
  * @brief  主机读写从机超过最大错误次数回调
  * @param  add   从机地址
  * @param  cmd   功能码
  * @param  data  异常码
  * @retval None
  */
void mbh_hook_timesErr(uint8_t add, uint8_t cmd, uint8_t data);

/**
  * @brief  从机更新保持寄存器值
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_holding(mbs *_mbs);

/**
  * @brief  从机提取保持寄存器值
  * @param  _mbs  从机结构体
  * @param  _reg  寄存器地址
  * @param  _val  寄存器值
  * @retval None
  */
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val);

/**
  * @brief  从机更新线圈状态
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_coils(mbs *_mbs);

/**
  * @brief  从机提取线圈状态
  * @param  _mbs  从机结构体
  * @param  _reg  线圈地址
  * @param  _val  线圈值
  * @retval None
  */
void mbs_hook_extract_coils(mbs *_mbs, uint16_t _reg, uint8_t _val);

#endif

/*----------------------------- End of file -------------------------------*/
