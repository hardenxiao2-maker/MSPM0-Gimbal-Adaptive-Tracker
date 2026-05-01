#include "stepper_x_ctrl.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_timerg.h" // Include for TIMG12
#include "ti/driverlib/m0p/dl_core.h"
#include <stdlib.h> // abs

/* 
 * X轴步进电机控制配置 (交换后)
 * STP (脉冲) -> PA21
 * DIR (方向) -> PB21
 */
#define STP_PORT        GPIOA
#define STP_PIN         DL_GPIO_PIN_21
#define DIR_PORT        GPIOB
#define DIR_PIN         DL_GPIO_PIN_21

// ---------------- 定时器配置 (TIMG12) ----------------
// TIMG12 不在 ti_msp_dl_config中自动生成，需手动使用
#define X_TIMER_INST    TIMG12
#define X_TIMER_IRQ     TIMG12_INT_IRQn

// 全局变量控制电机运动
static volatile int32_t remaining_steps = 0;
static volatile bool is_running = false;
static volatile int8_t current_dir = 0; // 1: CW, -1: CCW, 0: 停止

// DDS 频率生成器参数 (固定 10kHz 时基)
static volatile uint32_t phase_acc = 0;
static volatile uint32_t phase_inc = 0;

// 绝对位置跟踪 (累积角度，单位：步)
static volatile int32_t g_x_position = 0;

// PID 参数
static float s_kp = 1.0f;
static float s_ki = 0.0f;
static float s_kd = 0.0f;

// PID 状态
static float s_prev_error = 0.0f;
static float s_integral = 0.0f;

void StepperX_Init(void)
{
    // 1. 初始化 GPIO
    // 移除原有导致系统崩溃的 DL_GPIO_initDigitalOutput 越界调用
    // SysConfig 中已正确配置 YUNTAI_PIN_19/PA21/PB21 输出，因此仅需开启引脚即可
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_19);
    DL_GPIO_enableOutput(STP_PORT, STP_PIN);
    DL_GPIO_enableOutput(DIR_PORT, DIR_PIN); 

    // 2. 初始化 TIMG12
    DL_TimerG_reset(X_TIMER_INST);
    DL_TimerG_enablePower(X_TIMER_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    // 配置时钟: 使用 BUSCLK, 改用 TimerG 特有的配置结构
    DL_TimerG_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0
    };
    DL_TimerG_setClockConfig(X_TIMER_INST, &clockConfig);

    // 配置定时器模式: 周期性向下计数
    // 固定 10kHz 时基 (32MHz / 3200 = 10kHz)
    DL_TimerG_TimerConfig timerConfig = {
        .period = 3199, 
        .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
        .startTimer = DL_TIMER_START // 直接启动
    };
    DL_TimerG_initTimerMode(X_TIMER_INST, &timerConfig);

    // 关键修正：只有开启了 Clock，定时器才能在后台运转，否则会锁死
    DL_TimerG_enableClock(X_TIMER_INST);

    // 清除积压的中断，再开启中断保护
    NVIC_ClearPendingIRQ(X_TIMER_IRQ);
    DL_TimerG_enableInterrupt(X_TIMER_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(X_TIMER_IRQ);
}

void StepperX_SetPID(float kp, float ki, float kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    s_integral = 0;
    s_prev_error = 0;
    phase_inc = 0;
    phase_acc = 0;
}

int32_t StepperX_GetPosition(void)
{
    return g_x_position;
}

// 设置步进电机运行频率 (Hz, 正数=正向, 负数=反向, 0=停止)
void StepperX_SetFrequency(int32_t freq)
{
    if (freq == 0)
    {
        phase_inc = 0;
        current_dir = 0;
        // 注意：为避免再次卡死，我们不在这里关闭定时器。
        return;
    }

    // 设置方向 (已根据用户反馈反转)
    if (freq > 0)
    {
        DL_GPIO_clearPins(DIR_PORT, DIR_PIN); 
        current_dir = 1;
    }
    else
    {
        DL_GPIO_setPins(DIR_PORT, DIR_PIN); 
        current_dir = -1;
    }

    uint32_t abs_freq = abs(freq);
    
    // 简单保护，限制最大允许生成 2000Hz
    if (abs_freq > 2000) abs_freq = 2000;

    // 计算 DDS 步长 (相加增量)
    // 中断频率 = 10000Hz. 阈值 = 65536 (1<<16)
    phase_inc = (abs_freq << 16) / 10000;

    // 不需要在这里去调用启停函数，由后台 10kHz 持续处理 DDS 即可
}

// PID 控制主函数 (X轴)
// 输出目标频率
void StepperX_PID_Ctrl(float x)
{
    // 1. 计算误差 (目标位置是 0，x 是当前偏离值)
    float target = 0.0f;
    float error = target - x; 

    // 双阈值状态锁定 + 时间防抖机制
    // 过滤视觉噪声跳变
    static bool is_locked = false;
    static uint8_t unlock_counter = 0;

    if (is_locked) {
        // 锁定状态下，只有连续 3 帧超出逃逸阈值才认定目标真实移动
        if (fabs(error) > 6.5f) {
            unlock_counter++;
            if (unlock_counter >= 3) {  // 连续 3 帧确认，解除锁定
                is_locked = false;
                unlock_counter = 0;
            } else {
                StepperX_SetFrequency(0);
                return;
            }
        } else {
            unlock_counter = 0; // 回到稳态圈内，重置防抖计数
            StepperX_SetFrequency(0);
            return;
        }
    } else {
        // 追击状态：误差进入 1.5px 精度内圈后锁定
        if (fabs(error) <= 1.5f) {
            StepperX_SetFrequency(0);
            s_integral = 0; 
            s_prev_error = 0;
            is_locked = true;
            unlock_counter = 0;
            return;
        }
    }

    // 2. PID 计算
    s_integral += error;
    float derivative = error - s_prev_error;
    
    // 积分限幅
    if (s_integral > 1000) s_integral = 1000;
    if (s_integral < -1000) s_integral = -1000;
    
    float output = (s_kp * error) + (s_ki * s_integral) + (s_kd * derivative);
    s_prev_error = error;

    // 3. 输出直接作为频率 Hz
    int32_t target_freq = (int32_t)output;
    
    // 分段限速：X 轴负载重，需要保底频率克服静摩擦力
    int32_t max_allowed_freq = 250;  

    if (fabs(error) <= 18.0f) {
        max_allowed_freq = 60;  // 近场：低速滑行，防止超调 
    } else if (fabs(error) <= 35.0f) {
        max_allowed_freq = 100; 
    }

    // 速度钳制
    if (target_freq > max_allowed_freq) target_freq = max_allowed_freq;
    if (target_freq < -max_allowed_freq) target_freq = -max_allowed_freq;

    // 最小频率门限 40Hz，兼顾克服静摩擦与防止惯性超调
    if (target_freq > 0 && target_freq < 40) target_freq = 40;
    if (target_freq < 0 && target_freq > -40) target_freq = -40;

    // 4. 执行
    StepperX_SetFrequency(target_freq);
}

// TIMG12 中断服务函数
void TIMG12_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(X_TIMER_INST))
    {
        case DL_TIMERG_IIDX_ZERO:
            // 仅在有方向且要求有步进速度时累加相位
            if (current_dir != 0 && phase_inc > 0) {
                phase_acc += phase_inc;
                // DDS 溢出检测（溢出一次代表需要生成一个脉冲）
                if (phase_acc >= (1UL << 16)) {
                    phase_acc -= (1UL << 16);
                    
                    // 生成脉冲
                    DL_GPIO_setPins(STP_PORT, STP_PIN);
                    // 简单延时
                    for(volatile int i=0; i<30; i++); 
                    DL_GPIO_clearPins(STP_PORT, STP_PIN);

                    // 累计位置
                    g_x_position += current_dir;
                }
            }
            break;
        default:
            break;
    }
}
