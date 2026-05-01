#include "stepper_y_ctrl.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_timer.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <stdlib.h> // abs

/* 
 * 步进电机脉冲控制配置
 * PUL (脉冲) -> PA30
 * DIR (方向) -> PB23
 * EN  (使能) -> PB22
 */
#define PUL_PORT        GPIOA
#define PUL_PIN         DL_GPIO_PIN_30
#define DIR_PORT        GPIOB
#define DIR_PIN         DL_GPIO_PIN_23

// 运动参数限制
// 假设 3200 步/圈, 则 1 度 ≈ 8.89 步, 90 度 ≈ 800 步
#define MAX_ANGLE_STEPS  800  // +90 度
#define MIN_ANGLE_STEPS -800  // -90 度

// 全局变量控制电机运动
static volatile int32_t remaining_steps = 0;
static volatile bool is_running = false;
static volatile int8_t current_dir = 0; // 1: CW, -1: CCW, 0: 停止

// DDS 频率生成器参数 (固定 10kHz 时基)
static volatile uint32_t phase_acc = 0;
static volatile uint32_t phase_inc = 0;

// 绝对位置跟踪 (累积角度，单位：步)
static volatile int32_t g_y_position = 0;

// PID 参数
static float s_kp = 1.0f;
static float s_ki = 0.0f;
static float s_kd = 0.0f;

// PID 状态
static float s_prev_error = 0.0f;
static float s_integral = 0.0f;

void StepperY_Init(void)
{
    // 先停止 Timer0，防止初始化期间产生意外脉冲
    DL_Timer_stopCounter(TIMER_0_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN); // 清除积压的中断

    // 初始化 EN (PB22) -> Output
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_22);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_22);

    // 初始化引脚状态
    DL_GPIO_enableOutput(PUL_PORT, PUL_PIN);  // PA30 脉冲引脚使能输出
    DL_GPIO_enableOutput(DIR_PORT, DIR_PIN);  // PB23 方向引脚使能输出
    DL_GPIO_clearPins(PUL_PORT, PUL_PIN);
    DL_GPIO_clearPins(DIR_PORT, DIR_PIN);
    
    // 设置为固定 10kHz 时基 (32MHz / 3200 = 10kHz)
    DL_Timer_setLoadValue(TIMER_0_INST, 3199); 
    
    // 启用 Timer0 中断，让它在后台持续按 10kHz 运行
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void StepperY_SetPID(float kp, float ki, float kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    // 重置积分项
    s_integral = 0;
    s_prev_error = 0;
}

// 重置电机状态 (用于优化器开始新一轮测试)
void StepperY_ResetStatus(void)
{
    g_y_position = 0;
    remaining_steps = 0;
    s_integral = 0;
    s_prev_error = 0;
    current_dir = 0;
    phase_inc = 0;
    phase_acc = 0;
    DL_Timer_stopCounter(TIMER_0_INST);
}

// 仅重置 PID 状态（不重置位置），用于优化器新一轮测试
void StepperY_ResetPIDState(void)
{
    s_integral = 0;
    s_prev_error = 0;
    current_dir = 0;
    phase_inc = 0;
}

int32_t StepperY_GetPosition(void)
{
    return g_y_position;
}

// 辅助函数：数值钳制
static int32_t clamp(int32_t val, int32_t min, int32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// 设置步进电机运行频率 (Hz, 正数=正向, 负数=反向, 0=停止)
void StepperY_SetFrequency(int32_t freq)
{
    if (freq == 0)
    {
        phase_inc = 0;
        current_dir = 0;
        return;
    }

    // 设置方向
    if (freq > 0)
    {
        DL_GPIO_setPins(DIR_PORT, DIR_PIN); // CW
        current_dir = 1;
    }
    else
    {
        DL_GPIO_clearPins(DIR_PORT, DIR_PIN); // CCW
        current_dir = -1;
    }

    uint32_t abs_freq = abs(freq);
    
    // 简单保护，限制最大允许生成 2000Hz (如果太快则失步)
    if (abs_freq > 2000) abs_freq = 2000;

    // 计算 DDS 步长 (相加增量)
    // 中断频率 = 10000Hz. 阈值 = 65536 (1<<16)
    // 也就是当累加满 65536 时发出一个脉冲
    // Increment = (TargetFreq * 65536) / 10000
    phase_inc = (abs_freq << 16) / 10000;

    if (!is_running)
    {
        is_running = true;
        DL_Timer_startCounter(TIMER_0_INST);
    }
}

// PID 控制主函数
// y: 输入值 (偏差，目标是使其趋向于 0)
void StepperY_PID_Ctrl(float y)
{
    // 1. 计算误差 (目标位置是 0，y 是当前偏离值)
    float target = 0.0f;
    float error = target - y; 

    // 双阈值状态锁定 + 时间防抖机制
    // 过滤视觉噪声跳变
    static bool is_locked = false;
    static uint8_t unlock_counter = 0;

    if (is_locked) {
        // 锁定状态下，只有连续 3 帧超出逃逸阈值才认定目标真实移动
        // 
        if (fabs(error) > 6.5f) {
            unlock_counter++;
            if (unlock_counter >= 3) {
                is_locked = false;
                unlock_counter = 0;
            } else {
                StepperY_SetFrequency(0);
                return;
            }
        } else {
            unlock_counter = 0;
            StepperY_SetFrequency(0);
            return;
        }
    } else {
        // 追击状态：误差进入 1.0px 精度内圈后锁定
        if (fabs(error) <= 1.0f) {
            StepperY_SetFrequency(0);
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
    
    // 限制积分项防止积分饱和
    if (s_integral > 1000) s_integral = 1000;
    if (s_integral < -1000) s_integral = -1000;
    
    float output = (s_kp * error) + (s_ki * s_integral) + (s_kd * derivative);
    s_prev_error = error;

    // 3. 输出直接作为频率 Hz
    int32_t target_freq = (int32_t)output;

    // 分段限速：根据误差距离限制最大频率
    int32_t max_allowed_freq = 300;

    if (fabs(error) <= 15.0f) {
        max_allowed_freq = 20;   // 近场：低速滑行 (Y 轴较轻)
    } else if (fabs(error) <= 35.0f) {
        max_allowed_freq = 60;   // 中场：减速
    }

    // 速度钳制
    if (target_freq > max_allowed_freq) target_freq = max_allowed_freq;
    if (target_freq < -max_allowed_freq) target_freq = -max_allowed_freq;

    // 最小频率门限 15Hz
    if (target_freq > 0 && target_freq < 15) target_freq = 15;
    if (target_freq < 0 && target_freq > -15) target_freq = -15;

    // 4. 软限位检查 (以角度为准, 当超过限位时禁止向该方向运动)
    if (g_y_position >= MAX_ANGLE_STEPS && target_freq > 0)
    {
        target_freq = 0; 
    }
    if (g_y_position <= MIN_ANGLE_STEPS && target_freq < 0)
    {
        target_freq = 0;
    }

    // 5. 执行
    StepperY_SetFrequency(target_freq);
}

// Timer0 中断服务函数 - 固定 10kHz 发生
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            // 仅在有方向且要求有步进速度时累加相位
            if (current_dir != 0 && phase_inc > 0) {
                phase_acc += phase_inc;
                // DDS 溢出检测（溢出一次代表需要生成一个脉冲）
                if (phase_acc >= (1UL << 16)) {
                    phase_acc -= (1UL << 16);
                    
                    // 生成脉冲
                    DL_GPIO_setPins(PUL_PORT, PUL_PIN);
                    // 微小延时保持高电平
                    for(volatile int i=0; i<30; i++); 
                    DL_GPIO_clearPins(PUL_PORT, PUL_PIN);
                    
                    // 记录绝对步数
                    g_y_position += current_dir;
                }
            }
            break;
            
        default:
            break;
    }
}
