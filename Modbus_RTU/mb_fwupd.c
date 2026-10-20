/**
  * @file    mb_fwupd.c
  * @brief   上位机经 Modbus 在线升级下位机固件：协议处理 + 片内 Flash 擦写 + CRC32
  * @details 分区、寄存器与流程说明见 mb_fwupd.h。应用程序与 bootloader 共用本文件
  *          （bootloader 定义 FWU_BOOTLOADER=1），只用 CMSIS 寄存器操作，不依赖 HAL。
  *          F407 只有一个 Flash bank：擦写期间 CPU 从 Flash 取指会停顿（擦一个 128KB 扇区约 1~2s，
  *          中断也一并停顿），所以 BEGIN/FINISH 要求电机全部停止（fwu_port_busy）。
  */

#include "mb_fwupd.h"
#include "stm32f4xx.h"
#include <stddef.h>
#include <string.h>

#ifndef FWU_BOOTLOADER
#define FWU_BOOTLOADER 0
#endif

/* 私有变量 ----------------------------------------------------------------*/
static uint8_t  s_mode     = FWU_MODE_APP;
static uint16_t s_ver[3]   = {0, 0, 0};
static uint8_t  s_state    = FWU_STATE_IDLE;
static uint8_t  s_err      = FWU_ERR_NONE;
static uint32_t s_size     = 0;      /* BEGIN 给出的固件字节数（4 的倍数） */
static uint32_t s_crc      = 0;
static uint32_t s_version  = 0;
static uint32_t s_received = 0;      /* 已写入的最远位置 */
static uint8_t  s_rebootReq    = 0;
static uint8_t  s_rebootStamped = 0;
static uint32_t s_rebootStampMs = 0;

/* CRC32（与 zlib crc32 相同：多项式 0xEDB88320，初值/结果异或 0xFFFFFFFF），半字节查表 */
static const uint32_t s_crcNibble[16] =
{
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

/* 私有函数 ----------------------------------------------------------------*/

static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
static uint32_t be32(const uint8_t *p) { return ((uint32_t)be16(p) << 16) | be16(p + 2); }

/* F407ZE：扇区 0~3 各 16KB，4 为 64KB，5~7 各 128KB */
static uint32_t flash_sector_of(uint32_t addr)
{
    const uint32_t off = addr - FLASH_BASE;
    if (off < 0x10000u) return off / 0x4000u;
    if (off < 0x20000u) return 4u;
    return 4u + off / 0x20000u;
}

/* 擦写后让 ART 的指令/数据缓存失效，之后读到的才是 Flash 里的新内容（同 HAL 的 FLASH_FlushCaches） */
static void flash_flush_caches(void)
{
    if (FLASH->ACR & FLASH_ACR_ICEN)
    {
        FLASH->ACR &= ~FLASH_ACR_ICEN;
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        FLASH->ACR |= FLASH_ACR_ICEN;
    }
    if (FLASH->ACR & FLASH_ACR_DCEN)
    {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        FLASH->ACR |= FLASH_ACR_DCEN;
    }
}

#define FLASH_SR_ERR_MASK  (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_ERR_MASK;   /* 写 1 清除残留标志 */
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/* 等待操作完成，返回 0=成功 */
static uint8_t flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {}
    if (FLASH->SR & FLASH_SR_ERR_MASK)
    {
        FLASH->SR = FLASH_SR_ERR_MASK;
        return 1;
    }
    return 0;
}

/* 允许擦写的范围：bootloader 扇区永不擦写；应用程序运行时不能擦写自己所在的应用区 */
static uint8_t flash_range_allowed(uint32_t addr, uint32_t len)
{
    const uint32_t end = addr + len;
    if (len == 0u || addr < FWU_META_BASE || end > FWU_DL_BASE + FWU_DL_SIZE || end < addr) return 0;
#if !FWU_BOOTLOADER
    if (addr < FWU_APP_BASE + FWU_APP_SIZE && end > FWU_APP_BASE) return 0;
#endif
    return 1;
}

static uint8_t meta_write(uint32_t size, uint32_t crc, uint32_t version)
{
    uint32_t rec[5];
    rec[0] = FWU_META_MAGIC;
    rec[1] = size;
    rec[2] = crc;
    rec[3] = version;
    rec[4] = ~(rec[0] ^ rec[1] ^ rec[2] ^ rec[3]);
    /* state 字留在擦除值 0xFFFFFFFF = 待安装 */
    if (fwu_flash_erase(FWU_META_BASE, sizeof(fwu_meta_t))) return 1;
    return fwu_flash_program(FWU_META_BASE, (const uint8_t *)rec, sizeof(rec));
}

static uint8_t fail(uint8_t err, uint8_t rsp)
{
    s_err = err;
    return rsp;
}

static uint8_t cmd_begin(const uint8_t *data, uint16_t num)
{
    if (num != 8u) return fail(FWU_ERR_CMD, FWU_RSP_ERR_VALUE);
    if (fwu_port_busy()) return fail(FWU_ERR_BUSY, FWU_RSP_BUSY);

    const uint32_t size = be32(data + 2);
    if (size == 0u || size > FWU_APP_SIZE || (size & 3u)) return fail(FWU_ERR_SIZE, FWU_RSP_ERR_VALUE);

    /* 之前写过、还没安装的记录作废：下载区马上要被擦掉 */
    const fwu_meta_t *m = fwu_meta_get();
    if (m && m->state == FWU_META_PENDING) fwu_meta_mark_done();

    s_state = FWU_STATE_IDLE;
    if (fwu_flash_erase(FWU_DL_BASE, size))
    {
        s_state = FWU_STATE_ERROR;
        return fail(FWU_ERR_ERASE, FWU_RSP_ERR_WRITE);
    }
    s_size     = size;
    s_crc      = be32(data + 6);
    s_version  = ((uint32_t)be16(data + 10) << 16) | ((uint32_t)(be16(data + 12) & 0xFFu) << 8) | (be16(data + 14) & 0xFFu);
    s_received = 0;
    s_state    = FWU_STATE_RECEIVING;
    s_err      = FWU_ERR_NONE;
    return FWU_RSP_OK;
}

static uint8_t cmd_finish(void)
{
    if (s_state == FWU_STATE_READY) return FWU_RSP_OK;   /* 上次应答丢了，上位机重发 */
    if (s_state != FWU_STATE_RECEIVING) return fail(FWU_ERR_STATE, FWU_RSP_ERR_VALUE);
    if (fwu_port_busy()) return fail(FWU_ERR_BUSY, FWU_RSP_BUSY);

    if (fwu_crc32((const uint8_t *)FWU_DL_BASE, s_size) != s_crc)
    {
        s_state = FWU_STATE_ERROR;
        return fail(FWU_ERR_CRC, FWU_RSP_ERR_WRITE);
    }
    if (!fwu_image_valid(FWU_DL_BASE, s_size))
    {
        s_state = FWU_STATE_ERROR;
        return fail(FWU_ERR_IMAGE, FWU_RSP_ERR_WRITE);
    }
    if (meta_write(s_size, s_crc, s_version))
    {
        s_state = FWU_STATE_ERROR;
        return fail(FWU_ERR_META, FWU_RSP_ERR_WRITE);
    }
    s_state = FWU_STATE_READY;
    s_err   = FWU_ERR_NONE;
    return FWU_RSP_OK;
}

static uint8_t write_data(const uint8_t *data, uint16_t num)
{
    if (s_state != FWU_STATE_RECEIVING) return fail(FWU_ERR_STATE, FWU_RSP_ERR_VALUE);
    if (num < 3u) return fail(FWU_ERR_OFFSET, FWU_RSP_ERR_VALUE);

    const uint32_t offset = be32(data);
    const uint32_t len    = (uint32_t)(num - 2u) * 2u;
    if ((offset & 3u) || (len & 3u) || len > FWU_DATA_MAX_BYTES || offset > s_size || len > s_size - offset)
        return fail(FWU_ERR_OFFSET, FWU_RSP_ERR_VALUE);

    const uint8_t r = fwu_flash_program(FWU_DL_BASE + offset, data + 4, len);
    if (r)
    {
        s_state = FWU_STATE_ERROR;
        return fail(r == 2u ? FWU_ERR_CONFLICT : FWU_ERR_PROGRAM, FWU_RSP_ERR_WRITE);
    }
    if (offset + len > s_received) s_received = offset + len;
    return FWU_RSP_OK;
}

/* 导出函数 ----------------------------------------------------------------*/

void fwu_init(uint8_t mode, uint16_t verMajor, uint16_t verMinor, uint16_t verPatch)
{
    s_mode   = mode;
    s_ver[0] = verMajor;
    s_ver[1] = verMinor;
    s_ver[2] = verPatch;
    s_state  = FWU_STATE_IDLE;
    s_err    = FWU_ERR_NONE;
}

uint8_t fwu_read_regs(uint16_t reg, uint16_t num, uint16_t *out)
{
    if (num == 0u || reg < FWU_REG_INFO || reg + num > FWU_REG_INFO + FWU_REG_INFO_COUNT)
        return FWU_RSP_ERR_REG_ADDR;

    uint16_t info[FWU_REG_INFO_COUNT];
    memset(info, 0, sizeof(info));
    info[0]  = FWU_ID_VALUE;
    info[1]  = FWU_PROTO_VER;
    info[2]  = s_mode;
    info[3]  = s_ver[0];
    info[4]  = s_ver[1];
    info[5]  = s_ver[2];
    info[6]  = s_state;
    info[7]  = s_err;
    info[8]  = (uint16_t)(FWU_APP_SIZE >> 16);
    info[9]  = (uint16_t)(FWU_APP_SIZE & 0xFFFFu);
    info[10] = (uint16_t)(s_received >> 16);
    info[11] = (uint16_t)(s_received & 0xFFFFu);
    info[12] = FWU_BOARD_ID;

    memcpy(out, &info[reg - FWU_REG_INFO], (uint32_t)num * 2u);
    return FWU_RSP_OK;
}

uint8_t fwu_write_regs(uint16_t reg, uint16_t num, const uint8_t *data)
{
    if (num == 0u) return FWU_RSP_ERR_VALUE;

    if (reg == FWU_REG_DATA) return write_data(data, num);
    if (reg != FWU_REG_CMD)  return FWU_RSP_ERR_REG_ADDR;

    switch (be16(data))
    {
        case FWU_CMD_BEGIN:
            return cmd_begin(data, num);
        case FWU_CMD_FINISH:
            return cmd_finish();
        case FWU_CMD_REBOOT:
            s_rebootReq = 1;
            return FWU_RSP_OK;
        case FWU_CMD_ABORT:
            s_state = FWU_STATE_IDLE;
            s_err   = FWU_ERR_NONE;
            return FWU_RSP_OK;
        case FWU_CMD_ENTER_BOOT:
            *(volatile uint32_t *)FWU_BOOT_HOLD_ADDR       = FWU_BOOT_HOLD_MAGIC;
            *(volatile uint32_t *)(FWU_BOOT_HOLD_ADDR + 4) = ~FWU_BOOT_HOLD_MAGIC;
            s_rebootReq = 1;
            return FWU_RSP_OK;
        default:
            return fail(FWU_ERR_CMD, FWU_RSP_ERR_VALUE);
    }
}

uint8_t fwu_poll(uint32_t nowMs)
{
    if (!s_rebootReq) return 0;
    if (!s_rebootStamped)
    {
        s_rebootStamped = 1;
        s_rebootStampMs = nowMs;
    }
    return (nowMs - s_rebootStampMs >= FWU_REBOOT_DELAY_MS) ? 1u : 0u;
}

uint32_t fwu_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (len--)
    {
        const uint8_t b = *data++;
        crc = (crc >> 4) ^ s_crcNibble[(crc ^ b) & 0x0Fu];
        crc = (crc >> 4) ^ s_crcNibble[(crc ^ (b >> 4)) & 0x0Fu];
    }
    return ~crc;
}

uint8_t fwu_flash_erase(uint32_t addr, uint32_t len)
{
    if (!flash_range_allowed(addr, len)) return 1;

    const uint32_t first = flash_sector_of(addr);
    const uint32_t last  = flash_sector_of(addr + len - 1u);
    uint8_t err = 0;

    flash_unlock();
    for (uint32_t s = first; s <= last && !err; s++)
    {
        while (FLASH->SR & FLASH_SR_BSY) {}
        FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
        FLASH->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_SER | (s << FLASH_CR_SNB_Pos);   /* 按字擦除，要求 VDD 2.7~3.6V */
        FLASH->CR |= FLASH_CR_STRT;
        err = flash_wait();
        FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    }
    flash_lock();
    flash_flush_caches();
    return err;
}

uint8_t fwu_flash_program(uint32_t addr, const uint8_t *src, uint32_t len)
{
    if ((addr & 3u) || (len & 3u) || !flash_range_allowed(addr, len)) return 1;

    /* 先查冲突：已经是目标值的字跳过（上位机重发同一块不出错），不是擦除值又不同则不能再写 */
    for (uint32_t i = 0; i < len; i += 4u)
    {
        const uint32_t w   = (uint32_t)src[i] | ((uint32_t)src[i + 1] << 8) | ((uint32_t)src[i + 2] << 16) | ((uint32_t)src[i + 3] << 24);
        const uint32_t cur = *(volatile const uint32_t *)(addr + i);
        if (cur != w && cur != 0xFFFFFFFFu) return 2;
    }

    uint8_t err = 0;
    flash_unlock();
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;
    for (uint32_t i = 0; i < len && !err; i += 4u)
    {
        const uint32_t w = (uint32_t)src[i] | ((uint32_t)src[i + 1] << 8) | ((uint32_t)src[i + 2] << 16) | ((uint32_t)src[i + 3] << 24);
        if (*(volatile const uint32_t *)(addr + i) == w) continue;
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(addr + i) = w;
        __DSB();
        err = flash_wait();
        FLASH->CR &= ~FLASH_CR_PG;
    }
    flash_lock();
    flash_flush_caches();
    if (err) return 1;

    for (uint32_t i = 0; i < len; i += 4u)
    {
        const uint32_t w = (uint32_t)src[i] | ((uint32_t)src[i + 1] << 8) | ((uint32_t)src[i + 2] << 16) | ((uint32_t)src[i + 3] << 24);
        if (*(volatile const uint32_t *)(addr + i) != w) return 1;
    }
    return 0;
}

uint8_t fwu_image_valid(uint32_t base, uint32_t size)
{
    const uint32_t sp = *(volatile const uint32_t *)base;
    const uint32_t pc = *(volatile const uint32_t *)(base + 4u);
    if (size < 8u || size > FWU_APP_SIZE) return 0;
    if (sp < FWU_SRAM_BASE || sp > FWU_SRAM_END || (sp & 3u)) return 0;
    if (!(pc & 1u)) return 0;                                   /* Thumb 位 */
    if (pc < FWU_APP_BASE + 8u || pc >= FWU_APP_BASE + size) return 0;
    return 1;
}

const fwu_meta_t *fwu_meta_get(void)
{
    const fwu_meta_t *m = (const fwu_meta_t *)FWU_META_BASE;
    if (m->magic != FWU_META_MAGIC) return 0;
    if (m->check != ~(m->magic ^ m->size ^ m->crc ^ m->version)) return 0;
    if (m->size == 0u || m->size > FWU_APP_SIZE || (m->size & 3u)) return 0;
    return m;
}

uint8_t fwu_meta_mark_done(void)
{
    const uint32_t zero = 0;
    return fwu_flash_program(FWU_META_BASE + offsetof(fwu_meta_t, state), (const uint8_t *)&zero, 4u);
}

/*----------------------------- End of file -------------------------------*/
