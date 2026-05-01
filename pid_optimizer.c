#include "pid_optimizer.h"
#include "stepper_y_ctrl.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h> // For printf debugging

// Optimization settings
#define TEST_DURATION_TICKS 200 // Assumes Tick called every 10-20ms. Total ~2-4s
#define TOLERANCE           0.3f

// 回退阶段使用的固定保守 PID 参数（避免用测试参数回退导致丢步）
#define RETURN_KP  1.5f
#define RETURN_KI  0.3f
#define RETURN_KD  0.0f

typedef enum {
    OPT_IDLE,
    OPT_RESET_WAIT,
    OPT_RUN_TEST,
    OPT_EVALUATE,
    OPT_RETURN,      // 回退电机到起始位置
    OPT_NEXT_GEN
} OptState;

static OptState current_state = OPT_IDLE;
static int32_t test_target = 0;
static int32_t start_pos = 0;    // 每轮测试的起始位置（绝对位置）
static uint32_t tick_counter = 0;
static uint32_t return_counter = 0;    // 回退计时
static uint32_t return_settle_cnt = 0; // 回退到位稳定计数
#define RETURN_TIMEOUT_TICKS 300       // 回退超时 (~3s)
#define RETURN_SETTLE_TICKS  20        // 稳定判定 (~200ms)

// Twiddle Algorithm State
static float p[3] = {0}; // kp, ki, kd
static float best_p[3] = {0}; // 记录产生历史最优误差的真正参数
static float dp[3] = {0.5f, 0.2f, 0.01f}; // Initial mutation steps
static int param_idx = 0; // 0=kp, 1=ki, 2=kd
static int phase = 0; // 0=try +dp, 1=try -dp

static float best_err = 1e9f; // Best error so far
static float current_run_err = 0.0f;
static int iteration_cnt = 0; // global iteration counter

static void ApplyParams(void) {
    StepperY_SetPID(p[0], p[1], p[2]);
    // Print current params for debugging (via UART if available, here just comment)
    // printf("Trying: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", p[0], p[1], p[2]);
}

void Optimizer_Init(PID_Params init_params) {
    p[0] = init_params.kp;
    p[1] = init_params.ki;
    p[2] = init_params.kd;
    
    best_p[0] = p[0];
    best_p[1] = p[1];
    best_p[2] = p[2];
    
    // Initial best error set to huge
    best_err = 1e9f;
    current_state = OPT_IDLE;
}

void Optimizer_Start(int32_t target_steps) {
    test_target = target_steps;
    // Prepare first run
    best_err = 1e9f; // Reset best error for a new optimization session
    
    // Start with current params
    phase = 0;
    param_idx = 0;
    
    // Initial perturbation
    p[param_idx] += dp[param_idx];
    
    current_state = OPT_RESET_WAIT;
}

bool Optimizer_IsRunning(void) {
    return current_state != OPT_IDLE;
}

void Optimizer_Tick(void) {
    switch (current_state) {
        case OPT_IDLE:
            break;

        case OPT_RESET_WAIT:
            // 记录当前位置作为本轮起点（不重置位置计数器）
            start_pos = StepperY_GetPosition();
            StepperY_ResetPIDState(); // 只重置 PID 状态
            ApplyParams();
            tick_counter = 0;
            current_run_err = 0;
            current_state = OPT_RUN_TEST;
            break;

        case OPT_RUN_TEST: {
            // 目标位置 = 起始位置 + 测试目标
            int32_t abs_target = start_pos + test_target;
            float current_pos = (float)StepperY_GetPosition();
            float input_y = current_pos - (float)abs_target;
            
            StepperY_PID_Ctrl(input_y);

            // 误差计算（相对于目标位置）
            float error = (float)abs_target - current_pos;
            current_run_err += (fabs(error) + 0.1f * fabs(error) * tick_counter);

            tick_counter++;
            if (tick_counter >= TEST_DURATION_TICKS) {
                StepperY_SetFrequency(0);
                current_state = OPT_EVALUATE;
            }
            break;
        }

        case OPT_EVALUATE: {
            // 先打印本轮真实使用的参数和对应的误差
            extern void uart0_send_string(char* str);
            char debug_buf[128];
            iteration_cnt++;
            sprintf(debug_buf, "%d,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", 
                    iteration_cnt, p[0], p[1], p[2], current_run_err, best_err);
            uart0_send_string(debug_buf);

            if (current_run_err < best_err) {
                // Improvement found
                best_err = current_run_err;
                
                // 记录下产生这个历史最佳成绩的参数
                best_p[0] = p[0];
                best_p[1] = p[1];
                best_p[2] = p[2];

                dp[param_idx] *= 1.1f; // Increase step size
                
                // Move to next param
                param_idx = (param_idx + 1) % 3;
                p[param_idx] += dp[param_idx];
                phase = 0;
            } else {
                // No improvement
                if (phase == 0) {
                    // Try other direction
                    p[param_idx] -= 2 * dp[param_idx]; // Go back and subtract
                    phase = 1;
                } else {
                    // Both directions failed
                    p[param_idx] += dp[param_idx]; // Restore original
                    dp[param_idx] *= 0.9f; // Decrease step size
                    
                    // Move to next param
                    param_idx = (param_idx + 1) % 3;
                    p[param_idx] += dp[param_idx];
                    phase = 0;
                }
            }
            
            // 回退电机到起始位置后再开始下一轮
            return_counter = 0;
            return_settle_cnt = 0;
            current_state = OPT_RETURN;
            
            // Check convergence (optional exit condition)
            if ((dp[0] + dp[1] + dp[2]) < TOLERANCE) {
                StepperY_SetFrequency(0);
                uart0_send_string("PID Optimization Complete.\r\n");
                
                // 结束时输出真正的最佳参数
                char res_buf[64];
                sprintf(res_buf, "Result - Kp:%.3f, Ki:%.3f, Kd:%.3f\r\n", best_p[0], best_p[1], best_p[2]);
                uart0_send_string(res_buf);
                
                current_state = OPT_IDLE; // Done
            }
            break;
        }

        case OPT_RETURN: {
            // 首次进入回退阶段时，切换到固定保守 PID 参数
            if (return_counter == 0) {
                StepperY_SetPID(RETURN_KP, RETURN_KI, RETURN_KD);
            }

            // 驱动电机回到 start_pos
            float ret_pos = (float)(StepperY_GetPosition() - start_pos);
            StepperY_PID_Ctrl(ret_pos);  // target=0, error = 0 - ret_pos

            if (fabs(ret_pos) < 1.0f) {
                return_settle_cnt++;
            } else {
                return_settle_cnt = 0;
            }

            return_counter++;
            if (return_settle_cnt >= RETURN_SETTLE_TICKS || return_counter >= RETURN_TIMEOUT_TICKS) {
                StepperY_SetFrequency(0);
                current_state = OPT_RESET_WAIT;
            }
            break;
        }

        default:
            break;
    }
}

int Optimizer_GetIteration(void) {
    return iteration_cnt;
}

float Optimizer_GetBestErr(void) {
    return best_err;
}

void Optimizer_GetBestParams(float *kp, float *ki, float *kd) {
    if (kp) *kp = best_p[0];
    if (ki) *ki = best_p[1];
    if (kd) *kd = best_p[2];
}

float Optimizer_GetDpSum(void) {
    return dp[0] + dp[1] + dp[2];
}
