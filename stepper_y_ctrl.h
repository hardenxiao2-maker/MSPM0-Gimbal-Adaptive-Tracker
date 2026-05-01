#ifndef STEPPER_Y_CTRL_H
#define STEPPER_Y_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 Y 轴步进电机控制模块
 * @details 配置 GPIO (PA30 脉冲, PB23 方向, PB22 使能) 和 TIMER_0
 */
void StepperY_Init(void);

/**
 * @brief 设置 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void StepperY_SetPID(float kp, float ki, float kd);

/**
 * @brief 重置电机状态 (位置、积分、误差等)
 */
void StepperY_ResetStatus(void);

/**
 * @brief 仅重置 PID 状态（不重置位置计数器）
 */
void StepperY_ResetPIDState(void);

/**
 * @brief 根据输入 y 进行 PID 控制，驱动电机转动
 * 
 * @param y 输入值 (偏差值，目标是使其趋向于 0)
 * @details 
 *  - 计算 PID 输出作为目标角度
 *  - 自动限制在 +/- 90 度范围内 (约 +/- 800 步)
 *  - 累积记录当前角度
 */
void StepperY_PID_Ctrl(float y);

/**
 * @brief 获取当前累积角度（步数）
 * @return int32_t 当前位置 (正数正转, 负数反转)
 */
int32_t StepperY_GetPosition(void);

/**
 * @brief 设置步进电机运行频率 (Hz)
 * @param freq 频率 (Hz, 正数=正向, 负数=反向, 0=停止)
 */
void StepperY_SetFrequency(int32_t freq);

#endif // STEPPER_Y_CTRL_H
