/**
  * @file    boot_main.c
  * @brief   下位机 bootloader（扇区 0~1，32KB）：安装上位机下发的新固件、跳转应用程序，
  *          应用程序无效时停在 Modbus 升级模式
  * @details 分区与升级协议见 Modbus_RTU/mb_fwupd.h。上电流程：
  *            1. 应用程序请求"停在 bootloader"（CCM 标志，ENTER_BOOT 命令）→ 记下并清除标志
  *            2. 升级记录为"待安装" → 校验下载区 → 擦应用区 → 拷贝（向量表最后写）→ 校验 →
  *               记录标为已处理；拷贝中途断电，下次上电重来
  *            3. 未请求停留且应用区向量表有效 → 跳转应用程序
  *            4. 否则在 USART1（PA9/PA10，115200 8N1，从机地址 1）上跑精简 Modbus 从机，
  *               只响应升级寄存器，上位机按与应用程序相同的流程下发固件，REBOOT 后回到第 2 步
  *          只用 CMSIS 寄存器、HSI 16MHz、不开任何中断（轮询收发），跳转前无需逐个关外设中断。
  */

#include "stm32f4xx.h"
#include "mb_fwupd.h"

#define BOOT_VER_MAJOR       1u
#define BOOT_VER_MINOR       0u
#define BOOT_VER_PATCH       0u

#define BOOT_SLAVE_ADDR      1u          /* 与应用程序默认从机地址 RS485_SLAVE_ADDR 一致 */
#define BOOT_BAUD            115200u     /* 与应用程序默认波特率 RS485_BAUD_RATE 一致 */
#define BOOT_HSI_HZ          16000000u   /* 复位后默认时钟，APB2 不分频 */
#define BOOT_FRAME_GAP_MS    3u          /* 超过这么久没收到字节即认为一帧结束 */
#define BOOT_INSTALL_TRIES   3u
#define BOOT_RX_MAX          256u
#define BOOT_TX_MAX          64u

static uint32_t s_ms;
static uint8_t  s_rx[BOOT_RX_MAX];
static uint16_t s_rxLen;
static uint32_t s_lastRxMs;
static uint8_t  s_tx[BOOT_TX_MAX];
static uint32_t s_appSp;                 /* 跳转用：放全局变量而不是栈上，换 MSP 之后仍可用 */
static uint32_t s_appPc;

/* bootloader 运行时不驱动任何电机 */
uint8_t fwu_port_busy(void)
{
    return 0;
}

/* ---------------- 时间：SysTick 不开中断，轮询 COUNTFLAG 计毫秒（擦 Flash 期间少计无妨） ---------------- */
static void tick_init(void)
{
    SysTick->LOAD = BOOT_HSI_HZ / 1000u - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t millis(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) s_ms++;
    return s_ms;
}

/* ---------------- USART1：PA9 TX / PA10 RX，AF7 ---------------- */
static void uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    GPIOA->AFR[1]   = (GPIOA->AFR[1] & ~(0xFFu << 4)) | (7u << 4) | (7u << 8);
    GPIOA->PUPDR    = (GPIOA->PUPDR & ~(3u << 20)) | (1u << 20);          /* RX 上拉 */
    GPIOA->OSPEEDR  = (GPIOA->OSPEEDR & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);
    GPIOA->MODER    = (GPIOA->MODER & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);

    USART1->BRR = (BOOT_HSI_HZ + BOOT_BAUD / 2u) / BOOT_BAUD;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_send(const uint8_t *buf, uint16_t len)
{
    while (len--)
    {
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = *buf++;
    }
    while (!(USART1->SR & USART_SR_TC)) {}
}

/* ---------------- Modbus RTU 从机（只响应升级寄存器） ---------------- */
static uint16_t crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--)
    {
        crc ^= *p++;
        for (uint8_t i = 0; i < 8u; i++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static void reply(uint16_t len)
{
    const uint16_t crc = crc16(s_tx, len);
    s_tx[len++] = (uint8_t)(crc & 0xFFu);
    s_tx[len++] = (uint8_t)(crc >> 8);
    uart_send(s_tx, len);
}

static void reply_exception(uint8_t fc, uint8_t code)
{
    s_tx[0] = BOOT_SLAVE_ADDR;
    s_tx[1] = fc | 0x80u;
    s_tx[2] = code;
    reply(3);
}

static void reply_echo(const uint8_t *f)
{
    for (uint8_t i = 0; i < 6u; i++) s_tx[i] = f[i];
    reply(6);
}

static void handle_frame(const uint8_t *f, uint16_t len)
{
    if (len < 4u || crc16(f, len) != 0u) return;       /* 含 CRC 整帧算出 0 才正确 */
    if (f[0] != BOOT_SLAVE_ADDR) return;

    const uint8_t  fc  = f[1];
    const uint16_t reg = (uint16_t)((f[2] << 8) | f[3]);
    uint8_t rsp;

    switch (fc)
    {
        case 3:
        case 4:
        {
            const uint16_t num = (uint16_t)((f[4] << 8) | f[5]);
            uint16_t regs[FWU_REG_INFO_COUNT];
            if (len != 8u) { reply_exception(fc, FWU_RSP_ERR_VALUE); return; }
            rsp = (FWU_IS_REG(reg) && num <= FWU_REG_INFO_COUNT) ? fwu_read_regs(reg, num, regs) : FWU_RSP_ERR_REG_ADDR;
            if (rsp != FWU_RSP_OK) { reply_exception(fc, rsp); return; }
            s_tx[0] = BOOT_SLAVE_ADDR;
            s_tx[1] = fc;
            s_tx[2] = (uint8_t)(num * 2u);
            for (uint16_t i = 0; i < num; i++)
            {
                s_tx[3 + 2 * i] = (uint8_t)(regs[i] >> 8);
                s_tx[4 + 2 * i] = (uint8_t)(regs[i] & 0xFFu);
            }
            reply((uint16_t)(3u + num * 2u));
            return;
        }
        case 6:
            if (len != 8u) { reply_exception(fc, FWU_RSP_ERR_VALUE); return; }
            rsp = FWU_IS_REG(reg) ? fwu_write_regs(reg, 1, &f[4]) : FWU_RSP_ERR_REG_ADDR;
            if (rsp != FWU_RSP_OK) { reply_exception(fc, rsp); return; }
            reply_echo(f);
            return;
        case 16:
        {
            const uint16_t num = (uint16_t)((f[4] << 8) | f[5]);
            if (len < 9u || f[6] != num * 2u || len != 9u + f[6]) { reply_exception(fc, FWU_RSP_ERR_VALUE); return; }
            rsp = FWU_IS_REG(reg) ? fwu_write_regs(reg, num, &f[7]) : FWU_RSP_ERR_REG_ADDR;
            if (rsp != FWU_RSP_OK) { reply_exception(fc, rsp); return; }
            reply_echo(f);
            return;
        }
        default:
            reply_exception(fc, 1u);
            return;
    }
}

static void modbus_poll(void)
{
    const uint32_t now = millis();
    const uint32_t sr  = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        const uint8_t b = (uint8_t)USART1->DR;              /* 先读 SR 再读 DR，同时清掉错误标志 */
        if ((sr & USART_SR_RXNE) && s_rxLen < BOOT_RX_MAX) s_rx[s_rxLen++] = b;
        s_lastRxMs = now;
    }
    else if (s_rxLen && now - s_lastRxMs >= BOOT_FRAME_GAP_MS)
    {
        handle_frame(s_rx, s_rxLen);
        s_rxLen = 0;
    }
}

/* ---------------- 启动流程 ---------------- */

/* 应用程序 ENTER_BOOT 命令在 CCM 留下的标志：读一次即清除 */
static uint8_t boot_hold_requested(void)
{
    volatile uint32_t *p = (volatile uint32_t *)FWU_BOOT_HOLD_ADDR;
    const uint8_t hold = (p[0] == FWU_BOOT_HOLD_MAGIC && p[1] == ~FWU_BOOT_HOLD_MAGIC) ? 1u : 0u;
    p[0] = 0;
    p[1] = 0;
    return hold;
}

/* 有待安装记录就把下载区拷到应用区；失败则让应用区向量表无效，停在升级模式 */
static void install_pending(void)
{
    const fwu_meta_t *m = fwu_meta_get();
    if (!m || m->state != FWU_META_PENDING) return;

    const uint32_t size = m->size;
    const uint32_t crc  = m->crc;
    if (fwu_crc32((const uint8_t *)FWU_DL_BASE, size) != crc || !fwu_image_valid(FWU_DL_BASE, size))
    {
        fwu_meta_mark_done();   /* 下载区已不是记录里的那份固件，放弃，旧程序照常运行 */
        return;
    }

    for (uint8_t t = 0; t < BOOT_INSTALL_TRIES; t++)
    {
        if (fwu_flash_erase(FWU_APP_BASE, size)) continue;
        /* 向量表（前 8 字节）最后写：中途断电时应用区向量表仍是擦除值，不会跳进半截程序 */
        if (fwu_flash_program(FWU_APP_BASE + 8u, (const uint8_t *)(FWU_DL_BASE + 8u), size - 8u)) continue;
        if (fwu_flash_program(FWU_APP_BASE, (const uint8_t *)FWU_DL_BASE, 8u)) continue;
        if (fwu_crc32((const uint8_t *)FWU_APP_BASE, size) != crc) continue;
        fwu_meta_mark_done();
        return;
    }
    fwu_flash_erase(FWU_APP_BASE, 8u);
}

static void jump_to_app(void)
{
    s_appSp = *(volatile uint32_t *)FWU_APP_BASE;
    s_appPc = *(volatile uint32_t *)(FWU_APP_BASE + 4u);

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (uint8_t i = 0; i < 8u; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SCB->VTOR = FWU_APP_BASE;
    __DSB();
    __ISB();
    __set_MSP(s_appSp);
    __enable_irq();              /* 复位后 PRIMASK=0，应用程序（HAL）按这个前提运行 */
    ((void (*)(void))s_appPc)();
}

int main(void)
{
    const uint8_t hold = boot_hold_requested();

    install_pending();
    if (!hold && fwu_image_valid(FWU_APP_BASE, FWU_APP_SIZE))
        jump_to_app();

    tick_init();
    uart_init();
    fwu_init(FWU_MODE_BOOT, BOOT_VER_MAJOR, BOOT_VER_MINOR, BOOT_VER_PATCH);

    for (;;)
    {
        modbus_poll();
        if (fwu_poll(millis()))
            NVIC_SystemReset();
    }
}
