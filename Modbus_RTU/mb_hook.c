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
    for (int i = 0; i < SMD_CH_MAX; i++)
    {
        _mbs->regHoldingBuf[SMD_1_AM_ADDR  + i * 10] = SMD_AM_READ(i);
        _mbs->regHoldingBuf[SMD_1_DR_ADDR  + i * 10] = SMD_DR_READ(i);
        _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i * 10] = SMD_ACC_DATA[i];
        _mbs->regHoldingBuf[SMD_1_JRK_ADDR + i * 10] = (uint16_t)(SMD_JERK_DATA[i] > 65535u ? 65535u : SMD_JERK_DATA[i]);
        /* STEP：步数控制触发寄存器（只写，回读始终为 0） */
        _mbs->regHoldingBuf[SMD_1_STEP_ADDR + i * 10] = 0;
        _mbs->regHoldingBuf[SMD_1_PU_ADDR  + i * 10] = SMD_PU_DATA[i];
        /* SP：实时速度，主机可读取当前运行频率 */
        _mbs->regHoldingBuf[SMD_1_SP_ADDR  + i * 10] = (uint16_t)smd_freq_gradient[i].current_freq_int;
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
  */
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    /* WRITE_HOLDING_V（=2）保护：上位机批量写寄存器时用该值标记"保持不变"；
     * 系统上电回原点尚未完成时，也不接受任何控制写入 */
    if ((_val == WRITE_HOLDING_V) || (g_sysToOrigin != toOriginSuccess))
        return;

    _mbs->regHoldingBuf[_reg] = _val;

    for (int i = 0; i < SMD_CH_MAX; i++)
    {
        /* --- 方向控制（步数模式下忽略，方向由步数差自动决定） --- */
        if (!g_motorStepsCtl[i].is_running)
        {
            if (_mbs->regHoldingBuf[SMD_1_DR_ADDR + i * 10] != SMD_DR_READ(i))
            {
                smd_freq_gradient[i].dir_change = 1;
                smd_freq_gradient[i].dir_state  = SMD_DIR_NORMAL;
            }
        }

        /* --- JRK写入：所有模式下均可更新 --- */
        if (_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i * 10] != SMD_JERK_DATA[i])
        {
            uint32_t jrk_val = (uint32_t)_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i * 10];
            if (jrk_val == 0)           jrk_val = SMD_JERK_DEFAULT;
            if (jrk_val < SMD_JERK_MIN) jrk_val = SMD_JERK_MIN;
            if (jrk_val > SMD_JERK_MAX) jrk_val = SMD_JERK_MAX;
            SMD_JERK_DATA[i] = jrk_val;
            _mbs->regHoldingBuf[SMD_1_JRK_ADDR + i * 10] = (uint16_t)(jrk_val > 65535u ? 65535u : jrk_val);
        }

        /* --- ACC_MAX写入：所有模式下均可更新 --- */
        if (_mbs->regHoldingBuf[SMD_1_ACC_ADDR + i * 10] != SMD_ACC_DATA[i])
        {
            uint16_t acc_val = _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i * 10];
            if (acc_val < SMD_ACC_MAX_MIN) acc_val = SMD_ACC_MAX_MIN;
            if (acc_val > SMD_ACC_MAX_MAX) acc_val = (uint16_t)SMD_ACC_MAX_MAX;
            SMD_ACC_DATA[i] = acc_val;
            _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i * 10] = acc_val;
        }

        /* --- STEP写入：进入步数控制模式 ---
         * 在 PU 之前处理：多寄存器连续写入时先触发步数模式，
         * 后续 PU 写入自动走"仅更新最大频率"分支。 */
        if (_reg == (SMD_1_STEP_ADDR + i * 10))
        {
            uint16_t target_steps = _mbs->regHoldingBuf[SMD_1_STEP_ADDR + i * 10];
            g_motorStepsCtl[i].targetSteps = target_steps;
            g_motorStepsCtl[i].is_running  = 1;
            g_motorStepsCtl[i].braking     = 0;
        }

        /* --- PU写入：步数模式仅更新最大频率，非步数模式启动S曲线 --- */
        if (_mbs->regHoldingBuf[SMD_1_PU_ADDR + i * 10] != SMD_PU_DATA[i])
        {
            uint16_t pu_freq = _mbs->regHoldingBuf[SMD_1_PU_ADDR + i * 10];
            if (pu_freq < SMD_PWM_FREQ_MIN) pu_freq = SMD_PWM_FREQ_MIN;
            if (pu_freq > SMD_PWM_FREQ_MAX) pu_freq = (uint16_t)SMD_PWM_FREQ_MAX;
            SMD_PU_DATA[i] = pu_freq;
            _mbs->regHoldingBuf[SMD_1_PU_ADDR + i * 10] = pu_freq;

            if (!g_motorStepsCtl[i].is_running)
            {
                /* 非步数模式：启动S曲线渐变 */
                SMD_PWM_SetFreqGradient((SMD_Channel)i, SMD_PU_DATA[i], SMD_ACC_DATA[i]);
            }
            /* 步数模式：仅更新 SMD_PU_DATA[i]，SMD_MotorStepsCtl 在下个中断取用 */
        }

        /* --- SP写入：直接跳变 / 急停 --- */
        if (_reg == (SMD_1_SP_ADDR + i * 10))
        {
            uint16_t sp_cmd = _mbs->regHoldingBuf[SMD_1_SP_ADDR + i * 10];

            if (sp_cmd == 0)
            {
                /* 急停：硬件停止，清除运动状态 */
                SMD_PWM_Stop((SMD_Channel)i);
                SMD_PU_DATA[i] = SMD_PWM_FREQ_MIN;
                smd_freq_gradient[i].v_c              = 0.0f;
                smd_freq_gradient[i].v_n              = 0.0f;
                smd_freq_gradient[i].a_c              = 0.0f;
                smd_freq_gradient[i].current_freq_int = SMD_PWM_FREQ_MIN;
                smd_freq_gradient[i].is_running       = 0;
                g_motorStepsCtl[i].is_running         = 0;
                g_motorStepsCtl[i].braking            = 0;
            }
            else if (sp_cmd > 1)
            {
                /* 直接跳变到指定频率（限幅），不经过S曲线 */
                uint16_t target = sp_cmd;
                if (target < (uint16_t)SMD_PWM_FREQ_MIN) target = (uint16_t)SMD_PWM_FREQ_MIN;
                if (target > (uint16_t)SMD_PWM_FREQ_MAX) target = (uint16_t)SMD_PWM_FREQ_MAX;
                SMD_PWM_SetFreq((SMD_Channel)i, target);
                smd_freq_gradient[i].v_c  = (float)target;
                smd_freq_gradient[i].v_n  = (float)target;
                smd_freq_gradient[i].a_c  = 0.0f;
                smd_freq_gradient[i].is_running = 0;
                SMD_PU_DATA[i] = target;
                g_motorStepsCtl[i].is_running = 0;
                g_motorStepsCtl[i].braking = 0;
            }
        }

        /* --- 全部步进电机急停寄存器 --- */
        if (_reg == STOP_ALL_MOTOR_ADDR && _mbs->regHoldingBuf[STOP_ALL_MOTOR_ADDR] == 1)
        {
            SMD_PWM_Stop((SMD_Channel)i);
            SMD_PU_DATA[i] = SMD_PWM_FREQ_MIN;
            smd_freq_gradient[i].v_c              = 0.0f;
            smd_freq_gradient[i].v_n              = 0.0f;
            smd_freq_gradient[i].a_c              = 0.0f;
            smd_freq_gradient[i].current_freq_int = SMD_PWM_FREQ_MIN;
            smd_freq_gradient[i].is_running       = 0;
            g_motorStepsCtl[i].is_running         = 0;
            g_motorStepsCtl[i].braking            = 0;
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

    switch (_reg)
    {
        case MOTOR_1_TARGET_SP_ADDR:
            API_MOTOR_SetSpeed(API_MOTOR_1, (int8_t)_val);
            break;
        case MOTOR_2_TARGET_SP_ADDR:
            API_MOTOR_SetSpeed(API_MOTOR_2, (int8_t)_val);
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
