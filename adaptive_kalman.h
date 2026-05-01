#ifndef ADAPTIVE_KALMAN_H
#define ADAPTIVE_KALMAN_H

#include <stdint.h>

/*========================================================================
 *  自适应卡尔曼滤波器 (Adaptive Kalman Filter)
 *
 *  特性：
 *    1. 基于新息（innovation）序列自适应调整 R（测量噪声协方差）
 *    2. 使用滑动窗口计算新息方差，实时跟踪测量噪声变化
 *    3. Q（过程噪声）可在初始化时设定或在线调节
 *    4. 适合嵌入式系统，资源占用极低
 *
 *  用法：
 *    AKF_Filter  kf_x, kf_y;
 *    AKF_Init(&kf_x, 初始值, Q, R);
 *    float filtered = AKF_Update(&kf_x, 新采样值);
 *========================================================================*/

/* ---------- 可配置参数 ---------- */
#define AKF_WINDOW_SIZE    5     /* 新息滑动窗口大小（小窗口=快速响应） */
#define AKF_R_MIN          0.1f  /* R 下限 */
#define AKF_R_MAX          5.0f  /* R 上限（严格限制，防止滤波器忽略测量值） */

/* ---------- 滤波器结构体 ---------- */
typedef struct {
    /* 卡尔曼状态 */
    float x_hat;        /* 状态估计值（滤波输出） */
    float P;            /* 估计误差协方差 */
    float Q;            /* 过程噪声协方差 */
    float R;            /* 测量噪声协方差（自适应调整） */
    float K;            /* 卡尔曼增益 */

    /* 自适应：新息滑动窗口 */
    float innov_buf[AKF_WINDOW_SIZE];  /* 新息（残差）缓冲区 */
    uint8_t  innov_idx;                /* 当前写入索引 */
    uint8_t  innov_cnt;                /* 已填充数量（未满窗口时使用） */

    /* 初始 R 值（用于计算限幅参考） */
    float R_init;
} AKF_Filter;

/* ---------- API ---------- */

/**
 * @brief  初始化自适应卡尔曼滤波器
 * @param  kf         滤波器实例指针
 * @param  init_val   初始状态估计值（通常设为第一次测量值）
 * @param  Q          过程噪声协方差（越大对测量值跟踪越快，但越不平滑）
 *                    推荐范围：0.01 ~ 1.0
 * @param  R          初始测量噪声协方差（越大越平滑，对突变响应越慢）
 *                    推荐范围：1.0 ~ 50.0
 */
void AKF_Init(AKF_Filter *kf, float init_val, float Q, float R);

/**
 * @brief  输入新的测量值，返回滤波后的估计值
 * @param  kf         滤波器实例指针
 * @param  measurement 新的原始测量值
 * @return 滤波后的值
 */
float AKF_Update(AKF_Filter *kf, float measurement);

/**
 * @brief  获取当前卡尔曼增益（调试用）
 */
float AKF_GetGain(const AKF_Filter *kf);

/**
 * @brief  获取当前自适应 R 值（调试用）
 */
float AKF_GetR(const AKF_Filter *kf);

/**
 * @brief  在线修改 Q 值
 */
void AKF_SetQ(AKF_Filter *kf, float Q);

/**
 * @brief  重置滤波器到指定值（在需要立即跳变时使用）
 */
void AKF_Reset(AKF_Filter *kf, float new_val);

#endif /* ADAPTIVE_KALMAN_H */
