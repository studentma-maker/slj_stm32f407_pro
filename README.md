# slj_stm32f407_pro

三联机嵌入式主控板（新板）下位机固件，STM32F407ZET6，Keil MDK-ARM 工程。

## 项目背景

新板与旧项目 [`slj_stm32f407`](../slj_stm32f407) 是同一台机械设备（进退/升降/夹爪/排发/送发/上料）
换用新 PCB 布局后的下位机固件。核心的 Modbus 通信协议与电机 S 曲线控制逻辑从旧项目移植而来，
主体行为保持一致，上位机只需更换寄存器地址映射即可继续使用。

与旧板的主要差异：

- 步进电机通道由 8 路减少为 6 路（`SMD_CH0~5`），去掉旧板 CH6/CH7 两路预留通道
  （原本就未分配具体机械功能）
- 每路步进电机新增 **EN**（驱动使能）、**AM**（驱动报警）信号，旧板没有这两路
- 原继电器驱动的普通电机（OUT12/13 + IN7/8/9 限位）已改为独立 PWM + H 桥驱动
  （见 `Middleware/api_motor_control.c`，M1/M2），因此未移植旧项目的
  `SMD_CheckRelayMotorLimit()` 继电器电机限位逻辑
- 旧板的 ESP32 从机链路（`mbsESP`）已取消；新板改为预留一路对等 MCU 的 Modbus 主机链路
  （`mbhMCU`，`Modbus_RTU/mb_host.c`），具体用途待定，当前仅搭好协议栈骨架

## 已移植内容

### 1. `Middleware/api_smd.c/h` — 6 路步进电机 S 曲线 + 步数控制

移植自旧项目 `Core/Src/smd.c`（8 路版本），保留：

- **S 曲线加减速**：`SMD_UpdateVelocity()` / `SMD_RunSCurve()`，jerk 限幅的加速度连续变化
- **步数控制**：`SMD_MotorStepsCtl()`，含刹车距离预判 `SMD_CalcAccNeedSteps()`、
  子步定点累加（消除浮点误差）、限位直达模式（`targetSteps=0` 回原点 / `0xFFFF` 到远端限位）
- **换向保护**：写方向不直接切 GPIO，而是先减速到 0 再切换，避免失步
- **限位急停** `SMD_IsLimited()` 与 **上电回原点状态机** `SMD_SysToOrigin()`：
  按用户确认，这两套逻辑与机械结构绑定、机械结构未变，因此原样移植，
  仅将旧板 IN/OUT 序号套用到新板的 `IN_READ()/OUT()` 宏上

**通道分配**（沿用旧项目顺序，只是从 8 路裁剪到 6 路）：

| 通道 | 别名 | 定时器/引脚 | 步数控制 |
|------|------|------|------|
| CH0 | MOTOR_PaiFa | TIM14_CH1 / PA7 | — |
| CH1 | MOTOR_Trans | TIM13_CH1 / PA6 | — |
| CH2 | MOTOR_FBack | TIM8_CH1N / PA5 | ✅ |
| CH3 | MOTOR_UpDown | TIM5_CH4 / PA3 | ✅ |
| CH4 | MOTOR_GripperMove | TIM9_CH1 / PA2 | ✅ |
| CH5 | MOTOR_Feed | TIM2_CH2 / PA1 | — |

> ⚠️ **待硬件核实**：通道与物理电机的绑定、`SMD_IsLimited`/`SMD_SysToOrigin`
> 中用到的 IN1~20、OUT1~12 具体序号，都是沿用旧板编号搬过来的假设值
> （新板原理图里 IN/OUT 只是通用光耦端子，没有打印功能标签，实际含义由接线决定）。
> 上电通电前请对照实际线束核实，必要时调整 `api_smd.c` 里的索引常量。

### 2. `Modbus_RTU/mb_hook.c/h` — 寄存器读写映射

在新板已有的 6 字段寄存器骨架（AM/EN/DR/PU/ACC/SP）基础上，新增 S 曲线步数控制
所需的 JRK / STEP / 当前步数只读（CS）寄存器，并按讨论确定的顺序重排：

每路电机占用 10 个连续寄存器，基址 = 电机序号 × 10：

| 偏移 | 名称 | 读写 | 说明 |
|------|------|------|------|
| +0 | AM | R | 报警信号（驱动器故障） |
| +1 | EN | R/W | 使能控制，直接透传到 SMD_EN 引脚 |
| +2 | DR | R/W | 方向控制，非步数模式下写入触发 S 曲线换向 |
| +3 | ACC | R/W | 最大加速度 Hz/s |
| +4 | JRK | R/W | Jerk Hz/s²，写 0 恢复默认值 |
| +5 | STEP | W | 目标步数，写入触发步数控制模式（须在 PU 之前写） |
| +6 | PU | R/W | 目标频率 / 步数模式最大脉冲频率 Hz |
| +7 | CS | R | 当前步数（只读） |
| +8 | SP | R/W | 实时速度；写 0=急停，写 N>1=直接跳变 |
| +9 | — | — | 预留 |

IO 映射区（IN/OUT/SYS_TO_ORIGIN 地址已按要求对齐旧板 `slj_stm32f407` 的取值，
M1/M2 直流电机寄存器为新板独有功能，旧板无对应地址，暂沿用原骨架取值）：

| 地址 | 说明 |
|------|------|
| 80~91 | 12 路继电器输出 OUT1~OUT12（与旧板 OUT1~OUT16 起始地址一致） |
| 92~99 | 预留（旧板此区间为编码器清零/编码器值，新板未实现） |
| 100~103 | M1/M2 直流电机目标/当前速度（新板独有，旧板无对应地址） |
| 104~111 | 预留 |
| 112~131 | 20 路数字输入 IN1~IN20（与旧板地址一致） |
| 133 | 系统上电回原点状态，只读，对应 `GrippertoOrigin_P` 枚举（与旧板 SYS_TO_ORIGIN 地址一致） |
| 149（= `REG_HOLDING_NREGS-1`） | 全部步进电机急停（写 1 触发） |

保留旧项目的 `WRITE_HOLDING_V`（=2）批量写保护约定：上位机批量写寄存器时，
某个寄存器写入值 2 表示"保持不变"，下位机跳过该寄存器，避免误改。

### 3. GPIO 配置修复

`Project.ioc` / `Core/Src/gpio.c` 中 `SMD_DR_2`（PE7，2 号步进电机方向脚）原被
CubeMX 误配置为外部中断输入（`GPXTI7`），导致写方向寄存器对该通道不生效。
已改为与其余 DR 引脚一致的推挽输出。

## 已知需要人工确认的假设

1. **IN/OUT/AM/EN 索引对应的具体物理信号**：本次移植沿用旧板编号，需要根据新板
   实际接线核对（尤其是 `SMD_IsLimited`/`SMD_SysToOrigin` 用到的 IN1/2/3/6/7/20
   和继电器 `RELAY_1/RELAY_FeedHair/RELAY_PressHair/RELAY_WarnYELLOW`）。
2. **`RELAY_1`/`RELAY_FeedHair`/`RELAY_PressHair`/`RELAY_WarnYELLOW` 及
   `WarnLED_on/off`、`RELAY_FeedHair_up/RELAY_PressHair_up` 的取值**：旧项目源码里
   使用了这些宏但代码库中并未找到定义（大概率是旧项目某次提交遗漏），当前取值
   是根据旧项目 README 文档记录的继电器映射表还原的，请核实。
3. **上电回原点门禁**：`mbs_hook_extract_holding()` 在 `g_sysToOrigin != toOriginSuccess`
   时会忽略全部 Modbus 写请求（含继电器输出、直流电机调速），这是旧项目的既有设计，
   意味着夹爪原点开关（IN6）在硬件未就位前，下位机对上位机的任何控制指令都不会生效。

## 编译

Keil MDK-ARM 中打开 `MDK-ARM/Project.uvprojx` 编译下载。
