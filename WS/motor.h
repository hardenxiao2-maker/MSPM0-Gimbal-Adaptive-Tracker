#include "ti_msp_dl_config.h"

#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);

void MotorL_SetPWM(int PWML);

void MotorR_SetPWM(int PWMR);

void motor_stop(void);
#endif
