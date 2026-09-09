/**
  * @file    mb_hook.c
  * @brief   modbus 主机/从机回调函数实现文件
  * @details 实现 modbus 主机接收回调与从机寄存器读写钩子函数。
  *          从机寄存器读写语义移植自 slj_stm32f407 项目 Modbus_RTU/mb_hook.c，
  *          按本板 6 路电机 + 新增 EN/AM 信号、JRK/STEP/CS 寄存器重新适配。
  * @version V2.0.0
  * @date    09-Sep-2026
  */

#include "mb_hook.h"
#include "mb_host.h"
#include "api_smd.h"
#include "api_motor_control.h"

/* 私有函数实现 ------------------------------------------------------------*/

/**
  * @brief  将"正转继电器位+反转继电器位"解码为目标速度，兼容旧板继电器协议
  * @param  fwd  正转继电器寄存器值（非0视为1）
  * @param  rev  反转继电器寄存器值（非0视为1）
  * @retval 100=正转，-100=反转，0=停止（两位同时为1的非法组合按停止处理）
  */
static int8_t mb_hook_decode_relay_pair(uint16_t fwd, uint16_t rev)
{
    uint8_t f = (fwd != 0);
    uint8_t r = (rev != 0);
    if (f && !r) return 100;
    if (!f && r) return -100;
    return 0;
}

/**
  * @brief  同时设置 M1、M2 目标速度
  * @note   新板 M1/M2 两路 MOS 桥硬件上是同一台电机的双路冗余接线（哪路桥
  *         烧了就把电机接线换到另一路，程序不用改），所以两路必须收到完全
  *         相同的控制指令，不能只驱动其中一路。
  */
static void mb_hook_set_cart_speed(int8_t speed)
{
    API_MOTOR_SetSpeed(API_MOTOR_1, speed);
    API_MOTOR_SetSpeed(API_MOTOR_2, speed);
}

/**
  * @brief  寄存器槽位(0~7) -> 物理步进通道(SMD_Channel, 0~5) 查找表
  * @note   由 mb_hook.h 里的 SMD_SLOT_x_PHYS_CH 宏（1~6=物理通道号，0=未接线）
  *         转换而来，-1 表示该槽位没有对应的物理通道。改接线只需要改
  *         mb_hook.h 里的宏，这里不用动。
  */
static const int8_t smd_slot_phys_ch[SMD_SLOT_COUNT] = {
    SMD_SLOT_1_PHYS_CH - 1,
    SMD_SLOT_2_PHYS_CH - 1,
    SMD_SLOT_3_PHYS_CH - 1,
    SMD_SLOT_4_PHYS_CH - 1,
    SMD_SLOT_5_PHYS_CH - 1,
    SMD_SLOT_6_PHYS_CH - 1,
    SMD_SLOT_7_PHYS_CH - 1,
    SMD_SLOT_8_PHYS_CH - 1,
};

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  主机模式下接收到从机不同功能码回复的回调处理
  * @param  add      从机地址
  * @param  data     接收到的从机数据指针
  * @param  datalen  接收到的数据长度
  * @retval None
  * @note   rec01\02\03 等数字代表功能码
  */
void mbh_hook_rec01(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec02(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec03(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec04(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec05(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec06(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec15(uint8_t add, uint8_t *data, uint8_t datalen)
{

}
void mbh_hook_rec16(uint8_t add, uint8_t *data, uint8_t datalen)
{

}

/**
  * @brief  主机读写从机超过最大错误次数回调
  * @param  add   从机地址
  * @param  cmd   功能码
  * @param  data  异常码
  * @retval None
  */
void mbh_hook_timesErr(uint8_t add, uint8_t cmd, uint8_t data)
{

}

/**
  * @brief  从机更新保持寄存器值（读取时刷新寄存器镜像）
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_holding(mbs *_mbs)
{
    for (int slot = 0; slot < SMD_SLOT_COUNT; slot++)
    {
        int8_t ch = smd_slot_phys_ch[slot];
        if (ch < 0) continue;  /* 该槽位未接线，寄存器保持原值，不驱动硬件 */

        _mbs->regHoldingBuf[SMD_SLOT_AM_ADDR(slot)]  = SMD_AM_READ(ch);
        _mbs->regHoldingBuf[SMD_SLOT_DR_ADDR(slot)]  = SMD_DR_READ(ch);
        _mbs->regHoldingBuf[SMD_SLOT_ACC_ADDR(slot)] = SMD_ACC_DATA[ch];
        _mbs->regHoldingBuf[SMD_SLOT_JRK_ADDR(slot)] = (uint16_t)(SMD_JERK_DATA[ch] > 65535u ? 65535u : SMD_JERK_DATA[ch]);
        /* STEP：步数控制触发寄存器（只写，回读始终为 0） */
        _mbs->regHoldingBuf[SMD_SLOT_STEP_ADDR(slot)] = 0;
        _mbs->regHoldingBuf[SMD_SLOT_PU_ADDR(slot)]  = SMD_PU_DATA[ch];
        /* SP：实时速度，主机可读取当前运行频率 */
        _mbs->regHoldingBuf[SMD_SLOT_SP_ADDR(slot)]  = (uint16_t)smd_freq_gradient[ch].current_freq_int;
    }

    for (int i = 0; i < 20; i++)
    {
        _mbs->regHoldingBuf[IN_1_ADDR + i] = IN_READ(i);
    }

    for (int i = 0; i < 12; i++)
    {
        _mbs->regHoldingBuf[OUT_1_ADDR + i] = OUT_READ(i);
    }

    _mbs->regHoldingBuf[MOTOR_1_CURRENT_SP_ADDR] = API_MOTOR_GetCurrentSpeed(API_MOTOR_1);
    _mbs->regHoldingBuf[MOTOR_2_CURRENT_SP_ADDR] = API_MOTOR_GetCurrentSpeed(API_MOTOR_2);

    _mbs->regHoldingBuf[SYS_TO_ORIGIN_ADDR]     = g_sysToOrigin;
    /* 仅夹爪、进退两路支持步数控制的电机开放当前步数只读回读，与旧板一致 */
    _mbs->regHoldingBuf[GRIPPER_CUR_STEPS_ADDR] = MotorCurStepsU[MOTOR_GripperMove];
    _mbs->regHoldingBuf[FBACK_CUR_STEPS_ADDR]   = MotorCurStepsU[MOTOR_FBack];
}

/**
  * @brief  从机提取保持寄存器值（写入时触发执行）
  * @param  _mbs  从机结构体
  * @param  _reg  写入的寄存器地址
  * @param  _val  写入的值
  * @retval None
  *
  * 寄存器写入行为说明：
  *  DR   ：设置 dir_change 标志（S曲线换向保护），步数模式下忽略
  *  ACC  ：更新最大加速度，所有模式下均可写入，限幅 SMD_ACC_MAX_MIN~MAX
  *  JRK  ：更新 jerk 值（Hz/s²），写 0 自动恢复 SMD_JERK_DEFAULT
  *  STEP ：写入目标步数，进入步数控制模式（须在 PU 之前写入）
  *  PU   ：非步数模式→启动S曲线；步数模式→更新最大脉冲频率
  *  SP   ：写 0=急停；写 N>1=直接跳变到 N Hz
  *  OUT13/14/15/16：兼容旧板"继电器料车电机"协议，13/14为主继电器对、
  *                   15/16为其冗余复制，四者统一解码后同步驱动 M1+M2
  *                   （M1/M2 是同一台电机的双路冗余接线，详见
  *                   mb_hook_decode_relay_pair() / mb_hook_set_cart_speed()）
  */
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    /* WRITE_HOLDING_V（=2）保护：上位机批量写寄存器时用该值标记"保持不变"；
     * 系统上电回原点尚未完成时，也不接受任何控制写入 */
    if ((_val == WRITE_HOLDING_V) || (g_sysToOrigin != toOriginSuccess))
        return;

    _mbs->regHoldingBuf[_reg] = _val;

    for (int slot = 0; slot < SMD_SLOT_COUNT; slot++)
    {
        int8_t ch = smd_slot_phys_ch[slot];
        if (ch < 0) continue;  /* 该槽位未接线，跳过，不驱动任何硬件 */

        /* --- 方向控制（步数模式下忽略，方向由步数差自动决定） --- */
        if (!g_motorStepsCtl[ch].is_running)
        {
            if (_mbs->regHoldingBuf[SMD_SLOT_DR_ADDR(slot)] != SMD_DR_READ(ch))
            {
                smd_freq_gradient[ch].dir_change = 1;
                smd_freq_gradient[ch].dir_state  = SMD_DIR_NORMAL;
            }
        }

        /* --- JRK写入：所有模式下均可更新 --- */
        if (_mbs->regHoldingBuf[SMD_SLOT_JRK_ADDR(slot)] != SMD_JERK_DATA[ch])
        {
            uint32_t jrk_val = (uint32_t)_mbs->regHoldingBuf[SMD_SLOT_JRK_ADDR(slot)];
            if (jrk_val == 0)           jrk_val = SMD_JERK_DEFAULT;
            if (jrk_val < SMD_JERK_MIN) jrk_val = SMD_JERK_MIN;
            if (jrk_val > SMD_JERK_MAX) jrk_val = SMD_JERK_MAX;
            SMD_JERK_DATA[ch] = jrk_val;
            _mbs->regHoldingBuf[SMD_SLOT_JRK_ADDR(slot)] = (uint16_t)(jrk_val > 65535u ? 65535u : jrk_val);
        }

        /* --- ACC_MAX写入：所有模式下均可更新 --- */
        if (_mbs->regHoldingBuf[SMD_SLOT_ACC_ADDR(slot)] != SMD_ACC_DATA[ch])
        {
            uint16_t acc_val = _mbs->regHoldingBuf[SMD_SLOT_ACC_ADDR(slot)];
            if (acc_val < SMD_ACC_MAX_MIN) acc_val = SMD_ACC_MAX_MIN;
            if (acc_val > SMD_ACC_MAX_MAX) acc_val = (uint16_t)SMD_ACC_MAX_MAX;
            SMD_ACC_DATA[ch] = acc_val;
            _mbs->regHoldingBuf[SMD_SLOT_ACC_ADDR(slot)] = acc_val;
        }

        /* --- STEP写入：进入步数控制模式 ---
         * 在 PU 之前处理：多寄存器连续写入时先触发步数模式，
         * 后续 PU 写入自动走"仅更新最大频率"分支。 */
        if (_reg == SMD_SLOT_STEP_ADDR(slot))
        {
            uint16_t target_steps = _mbs->regHoldingBuf[SMD_SLOT_STEP_ADDR(slot)];
            g_motorStepsCtl[ch].targetSteps = target_steps;
            g_motorStepsCtl[ch].is_running  = 1;
            g_motorStepsCtl[ch].braking     = 0;
        }

        /* --- PU写入：步数模式仅更新最大频率，非步数模式启动S曲线 --- */
        if (_mbs->regHoldingBuf[SMD_SLOT_PU_ADDR(slot)] != SMD_PU_DATA[ch])
        {
            uint16_t pu_freq = _mbs->regHoldingBuf[SMD_SLOT_PU_ADDR(slot)];
            if (pu_freq < SMD_PWM_FREQ_MIN) pu_freq = SMD_PWM_FREQ_MIN;
            if (pu_freq > SMD_PWM_FREQ_MAX) pu_freq = (uint16_t)SMD_PWM_FREQ_MAX;
            SMD_PU_DATA[ch] = pu_freq;
            _mbs->regHoldingBuf[SMD_SLOT_PU_ADDR(slot)] = pu_freq;

            if (!g_motorStepsCtl[ch].is_running)
            {
                /* 非步数模式：启动S曲线渐变 */
                SMD_PWM_SetFreqGradient((SMD_Channel)ch, SMD_PU_DATA[ch], SMD_ACC_DATA[ch]);
            }
            /* 步数模式：仅更新 SMD_PU_DATA[ch]，SMD_MotorStepsCtl 在下个中断取用 */
        }

        /* --- SP写入：直接跳变 / 急停 --- */
        if (_reg == SMD_SLOT_SP_ADDR(slot))
        {
            uint16_t sp_cmd = _mbs->regHoldingBuf[SMD_SLOT_SP_ADDR(slot)];

            if (sp_cmd == 0)
            {
                /* 急停：硬件停止，清除运动状态 */
                SMD_PWM_Stop((SMD_Channel)ch);
                SMD_PU_DATA[ch] = SMD_PWM_FREQ_MIN;
                smd_freq_gradient[ch].v_c              = 0.0f;
                smd_freq_gradient[ch].v_n              = 0.0f;
                smd_freq_gradient[ch].a_c              = 0.0f;
                smd_freq_gradient[ch].current_freq_int = SMD_PWM_FREQ_MIN;
                smd_freq_gradient[ch].is_running       = 0;
                g_motorStepsCtl[ch].is_running         = 0;
                g_motorStepsCtl[ch].braking            = 0;
            }
            else if (sp_cmd > 1)
            {
                /* 直接跳变到指定频率（限幅），不经过S曲线 */
                uint16_t target = sp_cmd;
                if (target < (uint16_t)SMD_PWM_FREQ_MIN) target = (uint16_t)SMD_PWM_FREQ_MIN;
                if (target > (uint16_t)SMD_PWM_FREQ_MAX) target = (uint16_t)SMD_PWM_FREQ_MAX;
                SMD_PWM_SetFreq((SMD_Channel)ch, target);
                smd_freq_gradient[ch].v_c  = (float)target;
                smd_freq_gradient[ch].v_n  = (float)target;
                smd_freq_gradient[ch].a_c  = 0.0f;
                smd_freq_gradient[ch].is_running = 0;
                SMD_PU_DATA[ch] = target;
                g_motorStepsCtl[ch].is_running = 0;
                g_motorStepsCtl[ch].braking = 0;
            }
        }

        /* --- 全部步进电机急停寄存器 --- */
        if (_reg == STOP_ALL_MOTOR_ADDR && _mbs->regHoldingBuf[STOP_ALL_MOTOR_ADDR] == 1)
        {
            SMD_PWM_Stop((SMD_Channel)ch);
            SMD_PU_DATA[ch] = SMD_PWM_FREQ_MIN;
            smd_freq_gradient[ch].v_c              = 0.0f;
            smd_freq_gradient[ch].v_n              = 0.0f;
            smd_freq_gradient[ch].a_c              = 0.0f;
            smd_freq_gradient[ch].current_freq_int = SMD_PWM_FREQ_MIN;
            smd_freq_gradient[ch].is_running       = 0;
            g_motorStepsCtl[ch].is_running         = 0;
            g_motorStepsCtl[ch].braking            = 0;
        }
    }

    /* --- 输出控制 --- */
    for (int i = 0; i < 12; i++)
    {
        if (_mbs->regHoldingBuf[OUT_1_ADDR + i] != OUT_READ(i))
        {
            OUT(i, _mbs->regHoldingBuf[OUT_1_ADDR + i]);
        }
    }

    /* M1/M2 是同一台电机的双路冗余接线（哪路 MOS 桥烧了就换接到另一路，
     * 程序不用改），所以无论上位机通过哪一组寄存器下发速度，都必须同时
     * 驱动 M1 和 M2：
     *   - MOTOR_1_TARGET_SP_ADDR / MOTOR_2_TARGET_SP_ADDR：新增的直接调速
     *     寄存器，两个地址等价，写任意一个都会同步到两路
     *   - OUT13/14/15/16：兼容旧板"继电器控制料车电机"协议，13/14 是主
     *     继电器对，15/16 是上位机对同一信号的冗余复制（onControlRelayMotor()
     *     里 CartUp/CartDown 永远同时设置这4个寄存器，13与15同值、14与16
     *     同值），四者统一以13/14解码后同步驱动两路 */
    switch (_reg)
    {
        case MOTOR_1_TARGET_SP_ADDR:
        case MOTOR_2_TARGET_SP_ADDR:
            mb_hook_set_cart_speed((int8_t)_val);
            break;

        case OUT_13_ADDR:
        case OUT_14_ADDR:
        case OUT_15_ADDR:
        case OUT_16_ADDR:
            mb_hook_set_cart_speed(mb_hook_decode_relay_pair(
                _mbs->regHoldingBuf[OUT_13_ADDR], _mbs->regHoldingBuf[OUT_14_ADDR]));
            break;

        default: break;
    }
}

/**
  * @brief  从机更新线圈状态
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_coils(mbs *_mbs)
{
    _mbs->regCoilsBuf[0] |= 0x01;
}

/**
  * @brief  从机提取线圈状态
  * @param  _mbs  从机结构体
  * @param  _reg  线圈地址
  * @param  _val  线圈值
  * @retval None
  */
void mbs_hook_extract_coils(mbs *_mbs, uint16_t _reg, uint8_t _val)
{
    if (_val)
    {
        _mbs->regCoilsBuf[_reg / 8] |= 1 << (_reg % 8);
    }
    else
    {
        _mbs->regCoilsBuf[_reg / 8] &= ~(1 << (_reg % 8));
    }
}

/*----------------------------- End of file -------------------------------*/
