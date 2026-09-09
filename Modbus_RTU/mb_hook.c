/**
  * @file    mb_hook.c
  * @brief   modbus 主机/从机回调函数实现文件
  * @details 实现 modbus 主机接收回调与从机寄存器读写钩子函数。
  * @version V1.0.0
  * @date    20-Jun-2025
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
  * @brief  从机更新保持寄存器值
  * @param  _mbs  从机结构体
  * @retval None
  */
void mbs_hook_updata_holding(mbs *_mbs)
{
    for(int i = 0; i < SMD_CH_MAX; i++)
    {
        _mbs->regHoldingBuf[SMD_1_AM_ADDR + i*10]  = SMD_AM_READ(i);
        _mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10]  = SMD_EN_READ(i);
        _mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10]  = SMD_DR_READ(i);
        _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10]  = SMD_PU_DATA[i];
        _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] = SMD_ACC_DATA[i];
        _mbs->regHoldingBuf[SMD_1_SP_ADDR + i*10]  = (uint16_t) smd_freq_gradient[i].current_freq_int;
    }

    for(int i = 0; i < 20; i++)
    {
        _mbs->regHoldingBuf[IN_1_ADDR + i] = IN_READ(i);
    }

    for(int i = 0; i < 12; i++)
    {
        _mbs->regHoldingBuf[OUT_1_ADDR + i] = OUT_READ(i);
    }
    
    _mbs->regHoldingBuf[MOTOR_1_CURRENT_SP_ADDR] = API_MOTOR_GetCurrentSpeed(API_MOTOR_1);
    _mbs->regHoldingBuf[MOTOR_2_CURRENT_SP_ADDR] = API_MOTOR_GetCurrentSpeed(API_MOTOR_2);
}

/**
  * @brief  从机提取保持寄存器值
  * @param  _mbs  从机结构体
  * @param  _reg  寄存器地址
  * @param  _val  寄存器值
  * @retval None
  */
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    _mbs->regHoldingBuf[_reg] = _val;

    for(int i = 0; i < SMD_CH_MAX; i++)
    {
        if(_mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10] != SMD_EN_READ(i))
        {
            SMD_EN(i, _mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10]);
        }

        if(_mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10] != SMD_DR_READ(i))
        {
            SMD_DR(i, _mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10]);
        }
        if(_mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10] != SMD_PU_DATA[i] ||\
            _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] != SMD_ACC_DATA[i])
        {
            SMD_PU_DATA[i] =  _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10];
            SMD_ACC_DATA[i] =  _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10];
            SMD_PWM_SetFreqGradient((SMD_Channel)i,SMD_PU_DATA[i],SMD_ACC_DATA[i]);
        }
    }

    for(int i = 0; i < 12; i++)
    {
        if(_mbs->regHoldingBuf[OUT_1_ADDR + i] != OUT_READ(i))
        {
            OUT(i, _mbs->regHoldingBuf[OUT_1_ADDR + i]);
        }
    }

    switch(_reg)
    {
        case MOTOR_1_TARGET_SP_ADDR:
        {
            API_MOTOR_SetSpeed(API_MOTOR_1, _val);
            break;
        }
        case MOTOR_2_TARGET_SP_ADDR:
        {
            API_MOTOR_SetSpeed(API_MOTOR_2, _val);
            break;
        }
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
    if(_val)
    {
        _mbs->regCoilsBuf[_reg / 8] |= 1 << (_reg % 8);  /* 设置状态寄存器 */
    }
    else
    {
        _mbs->regCoilsBuf[_reg / 8] &= ~(1 << (_reg % 8));  /* 清除状态寄存器 */
    }
}

/*----------------------------- End of file -------------------------------*/
