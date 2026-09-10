/**
  * @file    api_eeprom.c
  * @brief   EEPROM API 接口源文件
  * @details 实现 EEPROM 初始化及多字节读写等函数。
  * @version V1.0.0
  * @date    22-Aug-2026
  */

#include "api_eeprom.h"
#include "api_i2c.h"
#include "api_delay.h"
#include "parameter.h"
#include "mb_slave.h"
#include "mb_host.h"

/* 私有宏定义 --------------------------------------------------------------*/
/* AT24C02存储地址定义 */
enum
{
    /* 日期信息存储地址 */
    AT24Cxx_DATE_INFO_ADDR      = 0,
    /* USB通信modbus从机地址存储地址 */
    AT24Cxx_USB_SLAVE_ADDR      = AT24Cxx_DATE_INFO_ADDR + 4,
    /* USB通信modbus波特率存储地址 */
    AT24Cxx_USB_BAUD_ADDR       = AT24Cxx_USB_SLAVE_ADDR + 1,
    /* USB通信modbus校验位存储地址 */
    AT24CXX_USB_PARITY_ADDR     = AT24Cxx_USB_BAUD_ADDR + 4,
    /* MCU通信modbus从机地址存储地址 */
    AT24Cxx_MCU_SLAVE_ADDR      = AT24CXX_USB_PARITY_ADDR + 1,
    /* MCU通信modbus波特率存储地址 */
    AT24Cxx_MCU_BAUD_ADDR       = AT24Cxx_MCU_SLAVE_ADDR + 1,
    /* MCU通信modbus校验位存储地址 */
    AT24CXX_MCU_PARITY_ADDR     = AT24Cxx_MCU_BAUD_ADDR + 4
};

/* EEPROM读写引脚定义 */
#define EEPROM_WRITE_DISABLE()  HAL_GPIO_WritePin(I2C_WR_GPIO_Port, I2C_WR_Pin, GPIO_PIN_SET)
#define EEPROM_WRITE_ENABLE()   HAL_GPIO_WritePin(I2C_WR_GPIO_Port, I2C_WR_Pin, GPIO_PIN_RESET)

/* EEPROM参数配置定义 */
#define AT24Cxx_Address             0xA0

#define AT24Cxx_OK                  0
#define AT24Cxx_FAIL                1

/* 私有变量定义 ------------------------------------------------------------*/
uint8_t ucEEPROMFaultFlag = AT24Cxx_OK;  /* EEPROM故障标志位 */

/* 导出函数实现 ------------------------------------------------------------*/

/**
  * @brief  从EEPROM读取双字数据
  * @param  readAddr  读取地址
  * @param  data      读取数据
  * @param  len       读取长度
  * @retval None
  */
void EEPROM_ReadData(uint16_t readAddr, uint8_t *data, uint16_t len)
{
    /* 读取计数 */
    uint8_t cnt = 0;

    /* 第1步：发起I2C总线启动信号 */
    I2C_Start();
    /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    I2C_SendByte(AT24Cxx_Address | ((readAddr >> 7) & 0xFE) | I2C_WR);

    /* 第3步：等待器件应答 */
    if (I2C_WaitAck() != 0)
    {
        ucEEPROMFaultFlag = AT24Cxx_FAIL;
        goto cmd_fail;
    }

    /* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
    I2C_SendByte(readAddr);

    /* 第5步：等待器件应答 */
    if (I2C_WaitAck() != 0)
    {
        ucEEPROMFaultFlag = AT24Cxx_FAIL;
        goto cmd_fail;
    }

    /* 第6步：重新启动I2C总线。前面的代码的目的向EEPROM传送地址，下面开始读取数据 */
    I2C_Start();
    /* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    I2C_SendByte(AT24Cxx_Address | ((readAddr >> 7) & 0xFE) | I2C_RD);

    /* 第8步：等待器件应答 */
    if (I2C_WaitAck() != 0)
    {
        ucEEPROMFaultFlag = AT24Cxx_FAIL;
        goto cmd_fail;
    }

    /* 第9步：循环读取数据 */
    for (cnt = 0; cnt < len - 1; cnt++)
    {
        data[cnt] = I2C_ReadByte();
        /* 发送应答信号 */
        I2C_YAck();
    }

    /* 第10步：读取最后1个字节数据 */
    data[cnt] = I2C_ReadByte();
    /* 第11步: 最后1个字节读完后，CPU产生NACK信号 */
    I2C_NAck();
    /* 第12步: 发送I2C总线停止信号 */
    I2C_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
    /* 发送I2C总线停止信号 */
    I2C_Stop();
}

/**
  * @brief  写入双字数据到EEPROM
  * @param  writeAddr  写入地址
  * @param  data       写入数据
  * @param  len        写入长度
  * @retval None
  */
void EEPROM_WriteData(uint16_t writeAddr, uint8_t *data, uint16_t len)
{
    uint8_t cnt = 0; /* 写入计数 */

    EEPROM_WRITE_ENABLE(); /* 启用写入模式 */

    /* 第1步：发起I2C总线启动信号 */
    I2C_Start();
    /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    I2C_SendByte(AT24Cxx_Address | ((writeAddr >> 7) & 0xFE) | I2C_WR);

    /* 第3步：等待器件应答 */
    if (I2C_WaitAck() != 0)
    {
        ucEEPROMFaultFlag = AT24Cxx_FAIL;
        goto cmd_fail;
    }

    /* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
    I2C_SendByte(writeAddr);

    /* 第5步：等待器件应答 */
    if (I2C_WaitAck() != 0)
    {
        ucEEPROMFaultFlag = AT24Cxx_FAIL;
        goto cmd_fail;
    }

    /* 第6步：开始写入数据 */
    for (cnt = 0; cnt < len; cnt++)
    {
        I2C_SendByte(data[cnt]); /* 发送数据 */

        if (I2C_WaitAck() != 0)
        {
            ucEEPROMFaultFlag = AT24Cxx_FAIL;
            goto cmd_fail;
        }

        writeAddr++;

        if (((writeAddr % 8) == 0) && (cnt < (len - 1)))
        {
            /* 发送I2C总线停止信号 */
            I2C_Stop();
            /* 延时等待器件存储完成 */
            delay_ms(6);
            /* 第1步：发起I2C总线启动信号 */
            I2C_Start();
            /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
            I2C_SendByte(AT24Cxx_Address | I2C_WR);

            /* 第3步：等待器件应答 */
            if (I2C_WaitAck() != 0)
            {
                ucEEPROMFaultFlag = AT24Cxx_FAIL;
                goto cmd_fail;
            }

            /* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
            I2C_SendByte(writeAddr);

            /* 第5步：等待器件应答 */
            if (I2C_WaitAck() != 0)
            {
                ucEEPROMFaultFlag = AT24Cxx_FAIL;
                goto cmd_fail;
            }
        }
    }

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
    /* 发送I2C总线停止信号 */
    I2C_Stop();

    delay_ms(6);

    EEPROM_WRITE_DISABLE(); /* 禁用写模式 */
}

/**
  * @brief  EEPROM初始化
  * @param  None
  * @retval None
  */
void EEPROM_Init(void)
{
    #if AT24Cxx_DATA_INIT

    uint8_t slaveAddr = 0;
    uint8_t parity = 0;
    uint32_t baudRate = 0;
    uint32_t dateInfo = 0;

    EEPROM_ReadData(AT24Cxx_DATE_INFO_ADDR, (uint8_t *)&dateInfo, 4);

    /* 如果24Cxx的日期信息和定义的日期信息不同,则初始化24Cxx数据 */
    if(dateInfo != DATE_INFO)
    {
        dateInfo = DATE_INFO;

        slaveAddr = RS485_SLAVE_ADDR;
        baudRate = RS485_BAUD_RATE;
        parity = RS485_PARITY;

        EEPROM_WriteData(AT24Cxx_DATE_INFO_ADDR, (uint8_t *)&dateInfo, 4);

        EEPROM_WriteData(AT24Cxx_USB_SLAVE_ADDR, (uint8_t *)&slaveAddr, 1);
        EEPROM_WriteData(AT24Cxx_USB_BAUD_ADDR,  (uint8_t *)&baudRate, 4);
        EEPROM_WriteData(AT24CXX_USB_PARITY_ADDR,  (uint8_t *)&parity, 1);

        EEPROM_WriteData(AT24Cxx_MCU_SLAVE_ADDR, (uint8_t *)&slaveAddr, 1);
        EEPROM_WriteData(AT24Cxx_MCU_BAUD_ADDR, (uint8_t *)&baudRate, 4);
        EEPROM_WriteData(AT24CXX_MCU_PARITY_ADDR, (uint8_t *)&parity, 1);
    }

    #endif

    /* 初始化完AT24Cxx后,读取存储数据 */
    EEPROM_ReadData(AT24Cxx_USB_SLAVE_ADDR, (uint8_t *)&mbsUSB.slaveAddr, 1);
    EEPROM_ReadData(AT24Cxx_USB_BAUD_ADDR, (uint8_t *)&mbsUSB.baudRate, 4);
    EEPROM_ReadData(AT24CXX_USB_PARITY_ADDR, (uint8_t *)&mbsUSB.parity, 1);

    EEPROM_ReadData(AT24Cxx_MCU_SLAVE_ADDR, (uint8_t *)&mbhMCU.slaveAddr, 1);
    EEPROM_ReadData(AT24Cxx_MCU_BAUD_ADDR, (uint8_t *)&mbhMCU.baudRate, 4);
    EEPROM_ReadData(AT24CXX_MCU_PARITY_ADDR, (uint8_t *)&mbhMCU.parity, 1);

    /* 兜底：I2C 读失败（ucEEPROMFaultFlag==FAIL）或读回明显非法值（从机地址为0/
       波特率为0，例如全新未初始化的EEPROM或I2C总线故障导致读取直接返回上电
       残留值）时，退回编译期默认参数，避免下位机在通信参数非法的情况下完全
       哑掉且无任何现象可查（原逻辑仅在dateInfo不匹配时才写默认值，读失败时
       不会有任何提示） */
    if (ucEEPROMFaultFlag == AT24Cxx_FAIL || mbsUSB.slaveAddr == 0 || mbsUSB.baudRate == 0)
    {
        mbsUSB.slaveAddr = RS485_SLAVE_ADDR;
        mbsUSB.baudRate  = RS485_BAUD_RATE;
        mbsUSB.parity    = RS485_PARITY;
    }

    if (ucEEPROMFaultFlag == AT24Cxx_FAIL || mbhMCU.slaveAddr == 0 || mbhMCU.baudRate == 0)
    {
        mbhMCU.slaveAddr = RS485_SLAVE_ADDR;
        mbhMCU.baudRate  = RS485_BAUD_RATE;
        mbhMCU.parity    = RS485_PARITY;
    }
}

/*----------------------------- End of file -------------------------------*/
