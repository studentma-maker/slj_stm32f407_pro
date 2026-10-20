/**
  * @file    mb_fwupd.h
  * @brief   上位机经 Modbus 在线升级下位机固件：Flash 分区、升级寄存器、协议处理接口
  * @details 应用程序与 bootloader 共用本模块（bootloader 编译时定义 FWU_BOOTLOADER=1），
  *          两边对上位机呈现同一套升级寄存器，上位机不用区分对端是谁。
  *          本模块只用 CMSIS 寄存器操作 Flash，不依赖 HAL。
  *
  * ==================== Flash 分区（STM32F407ZE，512KB）====================
  *   扇区 0~1  0x08000000  32KB   bootloader（升级永不擦写）
  *   扇区 2    0x08008000  16KB   升级记录（待安装固件的长度/CRC/版本，见 fwu_meta_t）
  *   扇区 3    0x0800C000  16KB   预留
  *   扇区 4~5  0x08010000  192KB  应用程序（Keil IROM1 = 0x08010000 / 0x30000）
  *   扇区 6~7  0x08040000  256KB  下载区（新固件先整包写到这里，校验通过后由 bootloader 搬到应用区）
  *
  * ==================== 升级流程 ====================
  *   1. 上位机读信息区（FWU_REG_INFO），确认协议、对端模式、当前版本
  *   2. 写 BEGIN（长度/CRC32/版本）：擦除下载区所需扇区（CPU 取指停顿约 1~2s，电机须全部停止）
  *   3. 逐块写 DATA（偏移 + 数据），写入下载区
  *   4. 写 FINISH：对下载区整包算 CRC32 并检查向量表，通过后写升级记录（待安装）
  *   5. 写 REBOOT：应答后复位；bootloader 见到待安装记录 → 校验下载区 → 擦应用区 → 拷贝
  *      （向量表最后写）→ 校验应用区 → 记录标为已处理 → 跳转新程序
  *   传输中途失败不影响正在运行的旧程序；拷贝中途断电，下次上电 bootloader 会重新拷贝。
  *   应用区向量表无效（未烧录/拷贝未完成）时 bootloader 停在 Modbus 升级模式等上位机。
  *
  * ==================== 升级寄存器（不在 regHoldingBuf 里，地址在普通寄存器表之外）====================
  *   0x1000~0x100F 信息区（只读，03 功能码）：
  *     +0  FWU_ID_VALUE（0x4657 'FW'）       +1  协议版本 FWU_PROTO_VER
  *     +2  模式 FWU_MODE_APP / FWU_MODE_BOOT  +3~+5 运行中程序的版本 主/次/修订
  *     +6  升级状态 FWU_STATE_*               +7  最近错误码 FWU_ERR_*
  *     +8~+9   最大固件长度（字节，高/低16位）
  *     +10~+11 已接收到的最远位置（字节，高/低16位）
  *     +12 板型 FWU_BOARD_ID                  +13~+15 预留(0)
  *   0x1010 命令（06/10 功能码写）：
  *     BEGIN    10 功能码连写 8 个：[1, 长度高, 长度低, CRC高, CRC低, 主版本, 次版本, 修订]
  *     FINISH   06 写 2；REBOOT 06 写 3；ABORT 06 写 4；ENTER_BOOT 06 写 5（复位后停在 bootloader）
  *   0x1100 数据（10 功能码）：[偏移高, 偏移低, 数据...]，数据为固件字节流（每寄存器高字节在前），
  *     每帧至多 FWU_DATA_MAX_BYTES 字节，偏移与字节数都须为 4 的倍数
  *   出错时返回 Modbus 异常应答（3=参数错，4=执行失败，6=电机运行中），具体原因读信息区 +7
  */
#ifndef _MB_FWUPD_H
#define _MB_FWUPD_H

#include <stdint.h>

/* ---- Flash 分区 ---- */
#define FWU_BOOT_BASE          0x08000000u
#define FWU_META_BASE          0x08008000u   /* 扇区 2 */
#define FWU_META_SECTOR        2u
#define FWU_APP_BASE           0x08010000u   /* 扇区 4 起 */
#define FWU_APP_SIZE           0x00030000u   /* 192KB：扇区 4~5 */
#define FWU_DL_BASE            0x08040000u   /* 扇区 6 起 */
#define FWU_DL_SIZE            0x00040000u   /* 256KB：扇区 6~7（固件长度上限仍按 FWU_APP_SIZE） */
#define FWU_SRAM_BASE          0x20000000u
#define FWU_SRAM_END           0x20020000u   /* 128KB SRAM（不含 CCM） */

/* 应用程序请求"复位后停在 bootloader"：CCM RAM 末尾两个字（应用和 bootloader 都不用 CCM，
 * 软件复位不清 RAM），值为 FWU_BOOT_HOLD_MAGIC 及其取反 */
#define FWU_BOOT_HOLD_ADDR     0x1000FFF0u
#define FWU_BOOT_HOLD_MAGIC    0x484F4C44u   /* 'HOLD' */

/* ---- 升级寄存器 ---- */
#define FWU_REG_BASE           0x1000u
#define FWU_REG_INFO           0x1000u
#define FWU_REG_INFO_COUNT     16u
#define FWU_REG_CMD            0x1010u
#define FWU_REG_DATA           0x1100u
#define FWU_REG_END            0x1200u
#define FWU_IS_REG(reg)        ((reg) >= FWU_REG_BASE && (reg) < FWU_REG_END)

#define FWU_ID_VALUE           0x4657u       /* 'FW' */
#define FWU_PROTO_VER          1u
#define FWU_BOARD_ID           0x0407u
#define FWU_DATA_MAX_BYTES     128u

#define FWU_MODE_APP           1u
#define FWU_MODE_BOOT          2u

#define FWU_CMD_BEGIN          1u
#define FWU_CMD_FINISH         2u
#define FWU_CMD_REBOOT         3u
#define FWU_CMD_ABORT          4u
#define FWU_CMD_ENTER_BOOT     5u

#define FWU_STATE_IDLE         0u
#define FWU_STATE_RECEIVING    1u
#define FWU_STATE_READY        2u            /* 整包校验通过，已写升级记录，等复位安装 */
#define FWU_STATE_ERROR        3u

#define FWU_ERR_NONE           0u
#define FWU_ERR_BUSY           1u            /* 电机运行中，拒绝擦写 Flash */
#define FWU_ERR_SIZE           2u            /* 固件长度为 0/超上限/不是 4 的倍数 */
#define FWU_ERR_ERASE          3u
#define FWU_ERR_STATE          4u            /* 当前状态不接受该命令（如未 BEGIN 就写数据） */
#define FWU_ERR_OFFSET         5u            /* 偏移/长度未对齐或越界 */
#define FWU_ERR_PROGRAM        6u
#define FWU_ERR_CONFLICT       7u            /* 该位置已写入不同的数据（需重新 BEGIN） */
#define FWU_ERR_CRC            8u
#define FWU_ERR_IMAGE          9u            /* 向量表不像本分区的程序（链接地址不是 0x08010000？） */
#define FWU_ERR_META           10u
#define FWU_ERR_CMD            11u

/* Modbus 异常码（与 mb_slave.h 的 mb_slave_rsp 数值一致，6 为标准"从站忙"） */
#define FWU_RSP_OK             0u
#define FWU_RSP_ERR_REG_ADDR   2u
#define FWU_RSP_ERR_VALUE      3u
#define FWU_RSP_ERR_WRITE      4u
#define FWU_RSP_BUSY           6u

/* ---- 升级记录（扇区 2 起始处）---- */
typedef struct
{
    uint32_t magic;      /* FWU_META_MAGIC */
    uint32_t size;       /* 固件字节数 */
    uint32_t crc;        /* 固件 CRC32 */
    uint32_t version;    /* 主<<16 | 次<<8 | 修订 */
    uint32_t check;      /* ~(magic ^ size ^ crc ^ version) */
    uint32_t state;      /* FWU_META_PENDING = 待安装；写 0 = 已处理（安装完成或放弃），不用擦扇区 */
} fwu_meta_t;

#define FWU_META_MAGIC         0x4D455441u   /* 'META' */
#define FWU_META_PENDING       0xFFFFFFFFu

/* ---- 应用程序固件信息（只在应用程序里定义，上位机按 magic 在 .bin 里定位它，读版本/板型）---- */
#define FWU_INFO_MAGIC         "SLJ_MCU_FWINFO1"   /* 15 字符 + '\0' 正好 16 字节 */
typedef struct
{
    char     magic[16];
    uint16_t board_id;
    uint16_t ver_major;
    uint16_t ver_minor;
    uint16_t ver_patch;
    char     build_date[12];   /* __DATE__ */
    char     build_time[12];   /* __TIME__ */
} fwu_info_t;

/* ---- 接口 ---- */

/**
  * @brief  初始化：记下本端模式与运行中程序的版本，读取升级记录状态
  */
void     fwu_init(uint8_t mode, uint16_t verMajor, uint16_t verMinor, uint16_t verPatch);

/**
  * @brief  读升级寄存器（03 功能码），out 按寄存器值填写
  * @retval Modbus 异常码，FWU_RSP_OK 表示成功
  */
uint8_t  fwu_read_regs(uint16_t reg, uint16_t num, uint16_t *out);

/**
  * @brief  写升级寄存器（06/10 功能码），data 为帧里的寄存器数据（每寄存器高字节在前）
  * @note   BEGIN/FINISH 会擦写 Flash，阻塞约 0.3~2s，须在主循环（而不是中断）里调用
  * @retval Modbus 异常码，FWU_RSP_OK 表示成功
  */
uint8_t  fwu_write_regs(uint16_t reg, uint16_t num, const uint8_t *data);

/**
  * @brief  主循环调用：收到 REBOOT/ENTER_BOOT 且应答已发出（延时 FWU_REBOOT_DELAY_MS）后返回 1，
  *         由调用方执行 NVIC_SystemReset()
  */
#define FWU_REBOOT_DELAY_MS    50u
uint8_t  fwu_poll(uint32_t nowMs);

/**
  * @brief  各工程自己实现：是否有电机在运行（擦写 Flash 期间 CPU 停顿，运行中的电机会丢步/计数）
  */
uint8_t  fwu_port_busy(void);

/* ---- 供 bootloader 安装固件使用的底层函数 ---- */
uint32_t fwu_crc32(const uint8_t *data, uint32_t len);
uint8_t  fwu_flash_erase(uint32_t addr, uint32_t len);                     /* 擦除覆盖 [addr, addr+len) 的扇区 */
uint8_t  fwu_flash_program(uint32_t addr, const uint8_t *src, uint32_t len); /* 4 字节对齐，0=成功 */
uint8_t  fwu_image_valid(uint32_t base, uint32_t size);                    /* 向量表：SP 在 SRAM、复位向量落在应用区 */
const fwu_meta_t *fwu_meta_get(void);                                      /* 记录有效返回指针，否则 NULL */
uint8_t  fwu_meta_mark_done(void);                                          /* 待安装记录标为已处理，0=成功 */

#endif /* _MB_FWUPD_H */

/*----------------------------- End of file -------------------------------*/
