#ifndef PID_OPTIMIZER_H
#define PID_OPTIMIZER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params;

/**
 * @brief Initialize the PID Optimizer
 * @param init_params Initial guess for PID parameters
 */
void Optimizer_Init(PID_Params init_params);

/**
 * @brief Start the optimization process
 * @param target_steps Target position for step response test (e.g. 500)
 */
void Optimizer_Start(int32_t target_steps);

/**
 * @brief Main tick function for the optimizer state machine.
 * Should be called periodically in the main loop.
 */
void Optimizer_Tick(void);

/**
 * @brief Check if optimization is running
 */
bool Optimizer_IsRunning(void);

int Optimizer_GetIteration(void);
float Optimizer_GetBestErr(void);

void Optimizer_GetBestParams(float *kp, float *ki, float *kd);
float Optimizer_GetDpSum(void);

#endif // PID_OPTIMIZER_H
