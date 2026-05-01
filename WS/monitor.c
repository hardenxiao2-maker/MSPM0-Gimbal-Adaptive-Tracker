#include "ti_msp_dl_config.h"
#include "monitor.h"

// 外部变量声明
extern float  ActualL, ActualR, OutL, OutR, TargetR, TargetL, e, target, e_last;	
extern float L2, L1, M0, R1, R2;
extern uint8_t zhijiao;
extern int left_motor_speed;
extern int right_motor_speed;
extern int base_speed ;        
extern int max_speed;          
extern int min_speed ;
extern float Kp, Ki, Kd;
extern float e, e_last, integral;
extern float dt;    
extern float output;
extern int right_angle_cnt ;  // 直角数（外部定义）
extern int lap_cnt ;          // 圈数（外部定义）
// 核心参数定义
#define FILTER_WINDOW 5
#define INTEGRAL_MAX 100.0f
#define INTEGRAL_MIN -100.0f
#define E_THRESHOLD 2.0f
#define RETURN_THRESH 3.5f
#define NORMAL_SPEED 400
#define OUTPUT_SMOOTH 0.7f
#define OUTPUT_MAX 1100.0f
#define OUTPUT_MIN -1100.0f

// 静态变量（关键：添加计数锁，确保一次转弯只计数一次）
static uint8_t last_turning = 0;    // 上一次转弯状态
static uint8_t turning = 0;         // 当前转弯状态（必须先更新再计数）
extern int count_locked;    // 计数锁：0=可计数，1=已计数（避免重复）
static float last_output = 0.0f; 
static float last_valid_pos = 4.8f; 
static float pos_buf[FILTER_WINDOW] = {4.8f, 4.8f, 4.8f, 4.8f, 4.8f};
static uint8_t buf_idx = 0;



void Read_Monitor(void)
{
    // 1. 读取传感器并计算原始位置
    uint32_t port_val = DL_GPIO_readPins(GPIOB, 
        DL_GPIO_PIN_20 | DL_GPIO_PIN_24 | DL_GPIO_PIN_10 | DL_GPIO_PIN_13 | DL_GPIO_PIN_12);
    
    float sum_x = 0.0f;
    int count = 0;
    L2 = (port_val & DL_GPIO_PIN_20) ? 0.8f : 0.0f;
    L1 = (port_val & DL_GPIO_PIN_24) ? 3.5f : 0.0f;
    M0 = (port_val & DL_GPIO_PIN_10) ? 4.8f : 0.0f;
    R1 = (port_val & DL_GPIO_PIN_13) ? 6.5f : 0.0f;
    R2 = (port_val & DL_GPIO_PIN_12) ? 9.3f : 0.0f;
    
    if (L2 > 0) { sum_x += L2; count++; }
    if (L1 > 0) { sum_x += L1; count++; }
    if (M0 > 0) { sum_x += M0; count++; }
    if (R1 > 0) { sum_x += R1; count++; }
    if (R2 > 0) { sum_x += R2; count++; }
    
    float current_pos;
    if (count > 0) {
        current_pos = sum_x / count;
        // 直行时滤波（逻辑不变）
        if (turning == 0) {
            pos_buf[buf_idx] = current_pos;
            buf_idx = (buf_idx + 1) % FILTER_WINDOW;
            float filter_sum = 0.0f;
            for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
                filter_sum += pos_buf[i];
            }
            current_pos = filter_sum / FILTER_WINDOW;
        }
        last_valid_pos = current_pos;
        e = current_pos - target;
    } else {
        // 无黑线处理（逻辑不变）
        if (last_valid_pos > 4.8f) {
            e = 3.6f;
            turning = 1; 
        } else if (last_valid_pos < 4.8f) {
            e = -3.6f;
            turning = 1; 
        } else {
            e = last_valid_pos - target;
            turning = 1; 
        }
    }


    // 2. 先更新当前turning状态（关键：必须在计数前更新！）
    uint8_t left_side = (L2 > 0 || (L2 > 0 && L1 > 0)) ? 1 : 0;
    uint8_t right_side = (R2 > 0 || (R2 > 0 && R1 > 0)) ? 1 : 0;
    if (left_side || right_side) {
        turning = 1; // 大幅转弯
    } else if (fabs(e) > RETURN_THRESH) {
        turning = 2; // 回正过渡
    } else {
        turning = 0; // 直行
        integral = 0;
        // 重置滤波缓存
        for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
            pos_buf[i] = last_valid_pos;
        }
        buf_idx = 0;
    }


    // 3. 圈数计数逻辑（必须在turning更新后！）
    // 3.1 下一次转弯开始时，解锁计数（允许新的计数）
    if (last_turning == 1 && ( turning == 2)) {
        count_locked = 0; // 解锁
    }

    // 3.2 转弯结束瞬间（且未锁定），计数一次并锁定
    if (( last_turning == 2) && turning == 0 && count_locked == 0) {
        right_angle_cnt++;          // 仅一次计数
        lap_cnt = (right_angle_cnt - 1) / 4;  // 计算圈数
        count_locked = 1;           // 锁定，避免重复
 
    }

    // 4. 最后更新last_turning（必须在计数后！）
    last_turning = turning;
}


void calculate_pid() 
{
    // PID逻辑不变（与turning状态正确关联）
    float current_Kp = Kp;
    float current_Ki = Ki;
    float current_Kd = Kd;
    if (turning == 1) {
        current_Kp = Kp;
        current_Ki = 0.0f;
        base_speed = 200;
    } else if (turning == 2) {
        current_Kp = 0.7 * Kp;
        current_Ki = Ki * 0.8f;
        base_speed = 200;
    } else {
        current_Kp = 0.5 * Kp;
        current_Ki = 3 * Ki;
        current_Kd = Kd;
        base_speed = 370;
    }
    
    if (turning != 1) {
        if (fabs(e) < E_THRESHOLD) {
            integral += current_Ki * e * dt;
            integral = (integral > INTEGRAL_MAX) ? INTEGRAL_MAX : integral;
            integral = (integral < INTEGRAL_MIN) ? INTEGRAL_MIN : integral;
        }
    }
    
    float P = current_Kp * e;
    float D = current_Kd * 1.2f * (e - e_last) / dt;
    output = P + integral + D;
    output = (output > OUTPUT_MAX) ? OUTPUT_MAX : output;
    output = (output < OUTPUT_MIN) ? OUTPUT_MIN : output;
    output = OUTPUT_SMOOTH * output + (1 - OUTPUT_SMOOTH) * last_output;
    last_output = output;
    
    left_motor_speed = base_speed - output;
    right_motor_speed = base_speed + output;
    left_motor_speed = (left_motor_speed > max_speed) ? max_speed : 
                      (left_motor_speed < min_speed) ? min_speed : left_motor_speed;
    right_motor_speed = (right_motor_speed > max_speed) ? max_speed : 
                       (right_motor_speed < min_speed) ? min_speed : right_motor_speed;
    
    e_last = e;
}