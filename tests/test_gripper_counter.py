"""Run the production counter/start/stop functions with register-backed host stubs.

python3 tests/test_gripper_counter.py
The timer event model tests arithmetic and stop sequencing, not STM32 electrical
or internal-trigger timing; those still require a logic analyzer on the board.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Middleware/api_smd.c").read_text()
backend = source[source.index("static uint8_t s_gripperCounterReady;"):
                 source.index("static void SMD_GripperFinish(void)")]
wrappers = source[source.index("HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch)"):
                  source.index("#if M6TOSERVO_ENABLE\n/* ========================= 舵机控制（仅")]
finish = source[source.index("static void SMD_GripperFinish(void)"):
                source.index("/* SMD_CheckRelayMotorLimit(void);")]
position_tick = source[source.index("static uint8_t SMD_GripperPositionTick(void)"):
                       source.index("/* ========================= 内部：上电回原点状态机")]
prefix = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "api_smd.h"
#include "tim.h"
#include "smd_gripper_profile.h"
#undef TIM3
#undef TIM5
#undef SMD_DR_READ
#undef __HAL_RCC_TIM3_CLK_ENABLE
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
static TIM_TypeDef counter, pwm;
static unsigned direction;
#define TIM3 (&counter)
#define TIM5 (&pwm)
#define SMD_DR_READ(ch) (direction)
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
uint16_t MotorCurStepsU[SMD_CH_MAX];
static volatile uint32_t MotorPulseTotal[SMD_CH_MAX];
SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX];
SMD_StepsCtl_t g_motorStepsCtl[SMD_CH_MAX];
uint16_t SMD_PU_DATA[SMD_CH_MAX];
uint16_t SMD_ACC_DATA[SMD_CH_MAX];
uint16_t SMD_JERK_DATA[SMD_CH_MAX];
typedef struct { TIM_HandleTypeDef *handle; uint32_t tim_channel;
                 uint8_t is_complementary; } SMD_TimerMap;
static TIM_HandleTypeDef test_timer;
static const SMD_TimerMap smd_timer_map[SMD_CH_MAX] = {
    {0}, {0}, {0}, {&test_timer, TIM_CHANNEL_4, 0}, {0}, {0}
};
void HAL_NVIC_SetPriority(IRQn_Type i, uint32_t p, uint32_t s)
{ (void)i; (void)p; (void)s; }
void HAL_NVIC_ClearPendingIRQ(IRQn_Type i) { (void)i; }
void HAL_NVIC_EnableIRQ(IRQn_Type i) { (void)i; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch)
{ (void)h; (void)ch; TIM5->CCER |= TIM_CCER_CC4E;
  TIM5->CR1 |= TIM_CR1_CEN; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch)
{ (void)h; (void)ch; TIM5->CCER &= ~TIM_CCER_CC4E;
  TIM5->CR1 &= ~TIM_CR1_CEN; return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *h, uint32_t ch)
{ return HAL_TIM_PWM_Start(h, ch); }
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *h, uint32_t ch)
{ return HAL_TIM_PWM_Stop(h, ch); }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin;
  return TIM5->CNT < TIM5->CCR4 ? GPIO_PIN_SET : GPIO_PIN_RESET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{ (void)port; (void)pin; direction = value == GPIO_PIN_SET; }
static uint8_t SMD_PWM_IsOutputOn(SMD_Channel ch)
{ (void)ch; return (TIM5->CCER & TIM_CCER_CC4E) != 0; }
static void SMD_ApplyFreqToHW(SMD_Channel ch, uint32_t frequency)
{ (void)ch; if (frequency < 50) frequency = 50;
  TIM5->ARR = 1000000u / frequency - 1u;
  TIM5->CCR4 = TIM5->ARR / 2u; TIM5->CCR1 = TIM5->CCR4; }
static uint8_t SMD_IsLimited(int ch, uint8_t dir, SMD_Freq_Gradient *m)
{ (void)ch; (void)dir; (void)m; return 0; }
'''
tests = r'''
/* One complete PWM high pulse. The internal PWM2 reference rises at its end. */
static void pulse(void)
{
    if (!(TIM3->CR1 & TIM_CR1_CEN)) return; /* Hardware gate closed. */
    TIM5->CNT = TIM5->CCR4;
    if (TIM3->CNT == TIM3->ARR)
    {
        TIM3->CNT = 0;
        TIM3->SR |= TIM_SR_UIF;
        if (TIM3->CR1 & TIM_CR1_OPM) TIM3->CR1 &= ~TIM_CR1_CEN;
    }
    else ++TIM3->CNT;
}
static void reset(unsigned dir)
{
    memset(&counter, 0, sizeof(counter));
    memset(&pwm, 0, sizeof(pwm));
    memset(MotorCurStepsU, 0, sizeof(MotorCurStepsU));
    memset(smd_freq_gradient, 0, sizeof(smd_freq_gradient));
    memset(g_motorStepsCtl, 0, sizeof(g_motorStepsCtl));
    SMD_PU_DATA[MOTOR_GripperMove] = 1600;
    SMD_ACC_DATA[MOTOR_GripperMove] = 4000;
    SMD_JERK_DATA[MOTOR_GripperMove] = 8000;
    MotorPulseTotal[MOTOR_GripperMove] = 0;
    s_gripperLastCount = 0;
    s_gripperRequestedPulses = 0;
    s_gripperAcceleration = 0;
    s_gripperArmedTarget = 0;
    direction = dir;
    test_timer.Instance = TIM5;
    TIM5->ARR = 999;
    TIM5->CCR4 = 499;
    SMD_GripperCounterInit();
}
int main(void)
{
    static const unsigned lengths[] = {1, 2, 100, 10000, 65534};
    unsigned i, j, dir;
    for (dir = 0; dir <= 1; ++dir)
        for (i = 0; i < sizeof(lengths)/sizeof(lengths[0]); ++i)
        {
            unsigned n = lengths[i];
            reset(dir);
            if (!dir) MotorCurStepsU[MOTOR_GripperMove] = n;
            s_gripperRequestedPulses = n;
            assert(SMD_PWM_Start(MOTOR_GripperMove) == HAL_OK);
            assert(TIM3->ARR != 0);
            for (j = 0; j < n; ++j)
            {
                pulse();
                if ((j % 7u) == 3u) SMD_GripperCollect();
            }
            assert(!(TIM3->CR1 & TIM_CR1_CEN));
            for (j = 0; j < 20; ++j) pulse(); /* Delay CPU service deliberately. */
            TIM3_IRQHandler();
            assert(s_gripperDone);
            assert(MotorPulseTotal[MOTOR_GripperMove] == n);
            assert(MotorCurStepsU[MOTOR_GripperMove] == (dir ? n : 0));
            SMD_PWM_Stop(MOTOR_GripperMove);
            assert(MotorPulseTotal[MOTOR_GripperMove] == n);
        }
    reset(1);
    SMD_PWM_Start(MOTOR_GripperMove);
    for (i = 0; i < 65536u; ++i) pulse();
    TIM3_IRQHandler();
    assert(MotorPulseTotal[MOTOR_GripperMove] == 65536u);
    for (i = 0; i < 17u; ++i) pulse();
    SMD_GripperCollect();
    assert(MotorPulseTotal[MOTOR_GripperMove] == 65553u);
    /* Emergency stop midway through high: settle that STEP once, before DIR. */
    TIM5->CNT = 100;
    SMD_PWM_Stop(MOTOR_GripperMove);
    assert(MotorPulseTotal[MOTOR_GripperMove] == 65554u);
    SMD_PWM_Stop(MOTOR_GripperMove);
    assert(MotorPulseTotal[MOTOR_GripperMove] == 65554u);
    direction = 0;
    s_gripperRequestedPulses = 2;
    SMD_PWM_Start(MOTOR_GripperMove);
    pulse(); pulse();
    SMD_GripperCollect();
    assert(MotorCurStepsU[MOTOR_GripperMove] == 65533u);
    assert(MotorPulseTotal[MOTOR_GripperMove] == 65556u);
    /* Exercise the production position state machine, not only the counter. */
    reset(1);
    g_motorStepsCtl[MOTOR_GripperMove].is_running = 1;
    g_motorStepsCtl[MOTOR_GripperMove].targetSteps = 3;
    assert(SMD_GripperPositionTick() == 1);
    pulse(); SMD_GripperPositionTick();
    assert(MotorCurStepsU[MOTOR_GripperMove] == 1);
    pulse(); pulse(); SMD_GripperPositionTick();
    assert(MotorCurStepsU[MOTOR_GripperMove] == 3);
    assert(!g_motorStepsCtl[MOTOR_GripperMove].is_running);
    assert(!SMD_PWM_IsOutputOn(MOTOR_GripperMove));

    reset(1);
    g_motorStepsCtl[MOTOR_GripperMove].is_running = 1;
    g_motorStepsCtl[MOTOR_GripperMove].targetSteps = 1000;
    SMD_GripperPositionTick();
    smd_freq_gradient[MOTOR_GripperMove].v_c = 500;
    SMD_PU_DATA[MOTOR_GripperMove] = 1;
    for (i = 0; i < 5000 && SMD_PWM_IsOutputOn(MOTOR_GripperMove); ++i)
        SMD_GripperPositionTick();
    assert(i < 5000);
    assert(g_motorStepsCtl[MOTOR_GripperMove].is_running);
    SMD_PU_DATA[MOTOR_GripperMove] = 1600;
    SMD_GripperPositionTick();
    assert(s_gripperFinite && SMD_PWM_IsOutputOn(MOTOR_GripperMove));
    /* Changing the target decelerates and re-arms from the settled position. */
    smd_freq_gradient[MOTOR_GripperMove].v_c = 500;
    g_motorStepsCtl[MOTOR_GripperMove].targetSteps = 2000;
    for (i = 0; i < 5000 && s_gripperArmedTarget != 2000; ++i)
        SMD_GripperPositionTick();
    assert(i < 5000);
    assert(s_gripperFinite && s_gripperArmedTarget == 2000);
    assert(s_gripperCountPeriod - 1u ==
           2000u - MotorCurStepsU[MOTOR_GripperMove]);
    g_motorStepsCtl[MOTOR_GripperMove].targetSteps = 0;
    assert(SMD_GripperPositionTick() == 0);
    assert(!s_gripperFinite);
    puts("production gripper counter tests passed");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="slj-gripper-test-") as directory:
    c = Path(directory) / "counter.c"
    executable = Path(directory) / "counter"
    c.write_text(prefix + backend + wrappers + finish + position_tick + tests)
    includes = ["Core/Inc", "Middleware", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F4xx/Include"]
    subprocess.run(["gcc", "-std=c99", "-O2", "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
                    "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
                    "-Wno-overflow", *[f"-I{ROOT / p}" for p in includes],
                    str(c), "-lm", "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
