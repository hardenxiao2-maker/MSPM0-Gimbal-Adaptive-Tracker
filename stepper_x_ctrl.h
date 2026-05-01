#ifndef STEPPER_X_CTRL_H
#define STEPPER_X_CTRL_H

#include <stdint.h>
#include <stdbool.h>

void StepperX_Init(void);
void StepperX_SetPID(float kp, float ki, float kd);
void StepperX_PID_Ctrl(float x);
int32_t StepperX_GetPosition(void);
// 设置步进电机运行频率 (Hz)
void StepperX_SetFrequency(int32_t freq);

#endif // STEPPER_X_CTRL_H
