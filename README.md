# slj_stm32f407_pro

三联机嵌入式主控板（新板）下位机固件，STM32F407ZET6，Keil MDK-ARM 工程。

## 项目背景

新板与旧项目 [`slj_stm32f407`](../slj_stm32f407) 是同一台机械设备（进退/升降/夹爪/排发/送发/上料）
换用新 PCB 布局后的下位机固件。核心的 Modbus 通信协议与电机 S 曲线控制逻辑从旧项目移植而来，
主体行为保持一致，上位机只需更换寄存器地址映射即可继续使用。

与旧板的主要差异：

- 步进电机通道由 8 路减少为 6 路（`SMD_CH0~5`），去掉旧板 CH6/CH7 两路预留通道
  （原本就未分配具体机械功能）
- 每路步进电机新增 **EN**（驱动使能）、**AM**（驱动报警）信号，旧板没有这两路。
  AM 仍作为只读寄存器暴露；EN 暂不通过 Modbus 控制（详见下方寄存器表说明）
- 原继电器驱动的推杆电机（旧板 `RELAY_MOTOR_1/2` 一对继电器正反转、
  IN7/8/9 限位）已改为 M1 独立 PWM + MOS 桥驱动（`Middleware/api_motor_control.c`），
  对应的限位急停逻辑已移植为 `API_MOTOR_CheckLimit()`（详见下方"M1/M2 直流电机"一节）
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

每路电机占用 10 个连续寄存器，基址 = 电机序号 × 10，字段顺序与地址**与旧板
完全一致**（AM/DR/ACC/JRK/STEP/PU/SP，偏移 0~6）：

| 偏移 | 名称 | 读写 | 说明 |
|------|------|------|------|
| +0 | AM | R | 报警信号（驱动器故障） |
| +1 | DR | R/W | 方向控制，非步数模式下写入触发 S 曲线换向 |
| +2 | ACC | R/W | 最大加速度 Hz/s |
| +3 | JRK | R/W | Jerk Hz/s²，写 0 恢复默认值 |
| +4 | STEP | W | 目标步数，写入触发步数控制模式（须在 PU 之前写） |
| +5 | PU | R/W | 目标频率 / 步数模式最大脉冲频率 Hz |
| +6 | SP | R/W | 实时速度；写 0=急停，写 N>1=直接跳变 |
| +7~9 | — | — | 预留 |

**EN（驱动使能）和 CS（当前步数）未做成逐路通用字段**：

- EN：不是所有电机都需要通过 Modbus 控制驱动器使能，暂不暴露为寄存器。
  `SMD_EN`/`SMD_EN_READ` 宏仍在 `main.h` 中可用，硬件当前按 GPIO 复位后的
  默认电平工作（未上电主动置位），若后续确认驱动器使能极性、确实需要
  上位机控制使能，再补充对应寄存器。
- CS：并非所有电机都支持步数控制，因此不做通用字段。仅 `MOTOR_FBack`（进退）、
  `MOTOR_GripperMove`（夹爪）两路支持步数控制的电机开放当前步数只读回读，
  与旧板一致（旧板的升降电机同样没有当前步数寄存器）：

| 地址 | 说明 |
|------|------|
| 132 | 夹爪电机当前步数（只读，`GRIPPER_CUR_STEPS_ADDR`），与旧板地址一致 |
| 134 | 进退电机当前步数（只读，`FBACK_CUR_STEPS_ADDR`），与旧板地址一致 |

IO 映射区（IN/OUT/SYS_TO_ORIGIN 地址已按要求对齐旧板 `slj_stm32f407` 的取值，
M1/M2 直流电机寄存器为新板独有功能，旧板无对应地址，暂沿用原骨架取值）：

| 地址 | 说明 |
|------|------|
| 80~91 | 12 路继电器输出 OUT1~OUT12（与旧板 OUT1~OUT16 起始地址一致） |
| 92~95 | 兼容旧板"继电器控制直流电机"协议（OUT13/14→M1，OUT15/16→M2），无实际GPIO，详见下方 |
| 96~99 | 预留（旧板此区间为编码器清零，新板未实现） |
| 100~103 | M1/M2 直流电机目标/当前速度（新板独有，旧板无对应地址） |
| 104~111 | 预留 |
| 112~131 | 20 路数字输入 IN1~IN20（与旧板地址一致） |
| 132 | 夹爪电机当前步数（只读，与旧板地址一致） |
| 133 | 系统上电回原点状态，只读，对应 `GrippertoOrigin_P` 枚举（与旧板地址一致） |
| 134 | 进退电机当前步数（只读，与旧板地址一致） |
| 199（= `REG_HOLDING_NREGS-1`） | 全部步进电机急停（写 1 触发） |

保留旧项目的 `WRITE_HOLDING_V`（=2）批量写保护约定：上位机批量写寄存器时，
某个寄存器写入值 2 表示"保持不变"，下位机跳过该寄存器，避免误改。

**`REG_HOLDING_NREGS` 由 150 改为 200**：核对上位机 `slj_kickpi_qt_pro/common/
reg_map.h` 发现其 `STOP_ALL_MOTOR_ADDR` 硬编码算的是 `200-1=199`，如果固件端
保持 150，地址 199 会越界，"全部电机急停"这个安全功能在上位机现有代码下根本
不会生效。改成 200 后两边完全对齐，多用的寄存器空间（200×2=400 字节/从机
实例）在 F407 上可以忽略不计。

### 3. 电机寄存器槽位 -> 板上物理通道映射（`SMD_SLOT_x_PHYS_CH`）

上位机 `slj_kickpi_qt_pro` 的协议是按旧板 8 路电机设计的（`common/reg_map.h`
里 `MOTOR_1~MOTOR_8`，寄存器地址 0~79，每路 10 个寄存器），但新板只有 6 路
物理步进通道。为了不用改上位机代码，`Modbus_RTU/mb_hook.h` 新增了 8 组具名
寄存器地址（`SMD_1~8_xxx_ADDR`，与上位机 reg_map.h 完全一致）和一套通用函数宏
`SMD_SLOT_xxx_ADDR(slot)`，`mb_hook.c` 按"寄存器槽位"（0~7，对应电机1~8）
遍历寄存器，而不是直接按物理通道遍历；每个槽位驱动哪一路物理通道，由
`mb_hook.h` 里这 8 个宏统一决定：

```c
#define SMD_SLOT_1_PHYS_CH   0   /* 排发自身地址，上位机实际不发，不接线 */
#define SMD_SLOT_2_PHYS_CH   2   /* 送发 Trans，直连2号 */
#define SMD_SLOT_3_PHYS_CH   3   /* 进退 FBack，直连3号 */
#define SMD_SLOT_4_PHYS_CH   4   /* 升降 UpDown，直连4号 */
#define SMD_SLOT_5_PHYS_CH   5   /* 夹爪 GripperMove，直连5号 */
#define SMD_SLOT_6_PHYS_CH   1   /* 上料 Feed：排发命令实际落在这里，转接到1号 */
#define SMD_SLOT_7_PHYS_CH   0   /* 未使用，不接线 */
#define SMD_SLOT_8_PHYS_CH   6   /* 送发辅助电机，转接到6号 */
```

数值 1~6 表示接到板上第几路物理通道（对应 `api_smd.h` 的 `SMD_CH0~SMD_CH5`），
0 表示这个槽位没有实际接线，写入会被忽略、不驱动任何硬件。**改接线只需要改
这 8 个数字，不用碰 `mb_hook.c` 里的任何逻辑**。

这张表是核对上位机 `worker/mainworker.cpp` 源码后确定的，关键依据：

- `onControlMotor()` 里有 `if (id == MOTOR_PaiFa) hwId = MOTOR_Feed;`——
  排发电机的命令实际发到了"槽位6"（MOTOR_Feed）的寄存器地址，不是自己槽位1
  的地址；槽位1这段地址上位机实际从不发送
- `onControlFeedSpeed()` 里 `MOTOR_Trans`、`MOTOR_FBack`、`MOTOR_GripperMove`
  都是用自己的槽位地址直接发送，没有被重映射
- `onControlFeedSpeed()` 额外用 `MOTOR_8`（槽位8）控制"送发辅助电机"

> ⚠️ **待硬件核实**：以上映射关系是根据 2026-09 时点的上位机源码 + 你确认的
> 实际接线（送发→2号、进退→3号、升降→4号、夹爪→5号、排发→1号、送发辅助
> →6号）推出来的。如果之后上位机协议或接线有变化，只需要改
> `mb_hook.h` 里的 8 个 `SMD_SLOT_x_PHYS_CH` 宏。

### 4. `Middleware/api_motor_control.c` — M1/M2 直流电机（MOS 桥）

新板用 4 颗 IRF3205 MOSFET（经 EG2104S 半桥驱动 IC + TLP2362 光耦隔离）搭了两组
独立全桥，分别驱动 M1、M2 两路直流电机（原理图 `SYS_MC` 页），取代了旧板用继电器
正反转驱动推杆电机的方案。每路电机有两个 PWM 输入（各控制桥的一条腿）：

| 电机 | "正端"引脚 | "负端"引脚 | 定时器通道 |
|------|------|------|------|
| M1 | M1_PWM1（PB8） | M1_PWM2（PB9） | TIM4_CH3 / TIM4_CH4 |
| M2 | M2_PWM1（PB7） | M2_PWM2（PB6） | TIM4_CH2 / TIM4_CH1 |

**不是简单的"某引脚置高/置低"，而是 PWM 占空比控制**：每路电机的两个引脚经
EG2104S 各自驱动一条半桥腿，引脚为高电平时对应桥臂上管导通（该输出节点被拉到
+24V），为低电平时下管导通（该节点被拉到 GND）。`Motor_SetPWM()`
（`api_motor_control.c`）的实际做法：

- 正转：`M1_PWM1` 输出与目标速度百分比成正比占空比的 PWM（决定平均电压），
  `M1_PWM2` 恒为 0%占空比（该桥臂输出恒定拉低）
- 反转：两个引脚角色互换，`M1_PWM2` 给 PWM，`M1_PWM1` 恒为 0%
- 停止：两个引脚占空比都是 0%，电机两端都被拉到 GND（动态刹车）

这套驱动本身、以及 Modbus 寄存器接口（`MOTOR_1/2_TARGET_SP_ADDR` 读写、
`MOTOR_1/2_CURRENT_SP_ADDR` 只读，见上方寄存器表）在本次移植开始前就已经
实现好了，不需要再改。

**平滑加减速调用周期修正**：`API_MOTOR_UpdateSpeed()` 每次调用把当前速度向
目标速度靠近 `g_motor_step_percent`（默认 2）个百分点，函数自身的文档注释
写的是"BSP 定时器 10ms 中执行"，但 `main.c` 里实际挂在了
`BSP_TIMER_FLAG_100MS` 下（100ms 一次），导致 0→100% 全程要 5 秒，比设计
意图慢了 10 倍。已改为跟 `mbh_send()` 共用 `BSP_TIMER_FLAG_10MS`（10ms 一次），
配合默认 2% 步进，0→100% 加减速时间变为 0.5 秒。如需更快，可以调大
`g_motor_step_percent`（如 5%→0.2 秒，10%→0.1 秒）。

**本次新移植的是旧板 `SMD_CheckRelayMotorLimit()` 的限位/急停保护逻辑**，
新增 `API_MOTOR_CheckLimit()`，每 1ms 在 `api_smd.c` 的 TIM10 中断里调用一次：

- 仅保护 M1（对应旧板 `RELAY_MOTOR_1`/`RELAY_MOTOR_2` 那一台推杆电机，旧板
  没有对第二路电机做限位检测，M2 同样不做）
- 用 M1 当前速度的正负号代替旧板"读继电器方向"来判断运行方向
- 限位/急停 IN 序号沿用旧板编号：**IN8(索引7)/IN10(索引9)** 为正转方向限位，
  **IN9(索引8)** 为反转方向限位，**IN20(索引19)** 为全局急停，触发后立即调用
  `API_MOTOR_Stop(API_MOTOR_1)` 停止该电机（待你核实新板实际接线）；这是立即
  停止（两路PWM占空比直接置0），不走平滑减速斜坡，跟旧板继电器断电的效果一致

**兼容旧板"继电器控制直流电机"协议**：旧板上位机是通过写继电器寄存器
`OUT_13_ADDR(92)`/`OUT_14_ADDR(93)` 一组控制一台电机、`OUT_15_ADDR(94)`/
`OUT_16_ADDR(95)` 一组控制另一台电机的通断来实现启停+换向的（13=1,14=0→
正转；13=0,14=1→反转；都为0→停止）。新板没有对应的物理继电器，但为了让
旧上位机代码不用改，在 `mbs_hook_extract_holding()` 里保留了这 4 个寄存器
地址作为纯协议兼容层：写入后由 `mb_hook_decode_relay_pair()` 解码，
分别转换成对 M1（13/14）、M2（15/16）的 `API_MOTOR_SetSpeed(..., 100/-100/0)`
调用——效果等价于直接写 `MOTOR_1/2_TARGET_SP_ADDR`，同样会经过平滑加减速。
上位机原有发 13/14/15/16 的代码完全不用改，新旧两套寄存器可以任选其一，
互相兼容。

### 5. GPIO 配置修复

`Project.ioc` / `Core/Src/gpio.c` 中 `SMD_DR_2`（PE7，2 号步进电机方向脚）原被
CubeMX 误配置为外部中断输入（`GPXTI7`），导致写方向寄存器对该通道不生效。
已改为与其余 DR 引脚一致的推挽输出。

## 已知需要人工确认的假设

1. **IN/OUT/AM 索引对应的具体物理信号**：本次移植沿用旧板编号，需要根据新板
   实际接线核对（尤其是 `SMD_IsLimited`/`SMD_SysToOrigin` 用到的 IN1/2/3/6/7/20、
   `API_MOTOR_CheckLimit()` 用到的 IN8/9/10/20，以及继电器
   `RELAY_1/RELAY_FeedHair/RELAY_PressHair/RELAY_WarnYELLOW`）。
2. **`RELAY_1`/`RELAY_FeedHair`/`RELAY_PressHair`/`RELAY_WarnYELLOW` 及
   `WarnLED_on/off`、`RELAY_FeedHair_up/RELAY_PressHair_up` 的取值**：旧项目源码里
   使用了这些宏但代码库中并未找到定义（大概率是旧项目某次提交遗漏），当前取值
   是根据旧项目 README 文档记录的继电器映射表还原的，请核实。
3. **上电回原点门禁**：`mbs_hook_extract_holding()` 在 `g_sysToOrigin != toOriginSuccess`
   时会忽略全部 Modbus 写请求（含继电器输出、直流电机调速），这是旧项目的既有设计，
   意味着夹爪原点开关（IN6）在硬件未就位前，下位机对上位机的任何控制指令都不会生效。
4. **M1 是否确实对应旧板 `RELAY_MOTOR_1` 那台推杆电机**：按"电机1↔电机1"的顺序
   假设一致，若实际接线是 M2 对应旧板的限位推杆电机，需要把
   `API_MOTOR_CheckLimit()` 里的 `API_MOTOR_1` 改成 `API_MOTOR_2`。
5. **SMD_EN 驱动使能引脚的有效电平未知**：目前固件完全不主动控制 EN（既不通过
   Modbus 暴露，也不在初始化时置位），停留在 `gpio.c` 配置的 GPIO 复位电平
   （低电平）。如果驱动器是"EN 拉高才使能"，步进电机上电后可能不会转动，
   需要确认极性后决定是否要在初始化代码里主动置位。

## 编译

Keil MDK-ARM 中打开 `MDK-ARM/Project.uvprojx` 编译下载。
