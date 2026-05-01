/*========================================================================
 *  adaptive_kalman.c  —  自适应卡尔曼滤波器实现
 *
 *  算法说明：
 *    标准一维卡尔曼滤波 + 基于新息序列的自适应 R 调整
 *
 *    1) 预测阶段：
 *       x_hat⁻ = x_hat（匀速模型，状态不变）
 *       P⁻     = P + Q
 *
 *    2) 更新阶段：
 *       innovation = z - x_hat⁻
 *       K          = P⁻ / (P⁻ + R)
 *       x_hat      = x_hat⁻ + K * innovation
 *       P          = (1 - K) * P⁻
 *
 *    3) 自适应阶段（每次更新后执行）：
 *       将 innovation 存入滑动窗口缓冲区
 *       R_new = (1/N) * Σ(innovation_i²)  —  新息方差作为 R 的估计
 *       R     = clamp(R_new, R_MIN, R_MAX)
 *
 *    这样当测量噪声突然增大时，R 自动增大，滤波器更依赖预测值（更平滑）；
 *    当测量噪声减小时，R 自动减小，滤波器更快跟随测量值。
 *========================================================================*/

#include "adaptive_kalman.h"
#include <string.h>

/* ---- 内部辅助：限幅 ---- */
static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/*------------------------------------------------------------------------
 *  AKF_Init  —  初始化滤波器
 *----------------------------------------------------------------------*/
void AKF_Init(AKF_Filter *kf, float init_val, float Q, float R)
{
    if (kf == (void*)0) return;

    kf->x_hat = init_val;
    kf->P     = 1.0f;       /* 初始协方差设为 1，收敛较快 */
    kf->Q     = Q;
    kf->R     = R;
    kf->R_init = R;
    kf->K     = 0.0f;

    /* 清零新息窗口 */
    memset(kf->innov_buf, 0, sizeof(kf->innov_buf));
    kf->innov_idx = 0;
    kf->innov_cnt = 0;
}

/*------------------------------------------------------------------------
 *  AKF_Update  —  一步滤波：输入原始测量值，输出滤波值
 *----------------------------------------------------------------------*/
float AKF_Update(AKF_Filter *kf, float measurement)
{
    if (kf == (void*)0) return measurement;

    /* ========== 1. 预测 ========== */
    /* 对于位置跟踪场景，假设状态转移 F=1（位置不会自行变化） */
    float x_pred = kf->x_hat;         /* x_hat⁻ = x_hat */
    float P_pred = kf->P + kf->Q;     /* P⁻ = P + Q */

    /* ========== 2. 更新 ========== */
    float innovation = measurement - x_pred;   /* 新息（残差） */

    float S = P_pred + kf->R;          /* 新息协方差 */
    kf->K = P_pred / S;               /* 卡尔曼增益 */

    kf->x_hat = x_pred + kf->K * innovation;   /* 状态更新 */
    kf->P     = (1.0f - kf->K) * P_pred;       /* 协方差更新 */

    /* ========== 3. 自适应 R ========== */
    /* 将本次新息存入滑动窗口 */
    kf->innov_buf[kf->innov_idx] = innovation;
    kf->innov_idx = (kf->innov_idx + 1) % AKF_WINDOW_SIZE;
    if (kf->innov_cnt < AKF_WINDOW_SIZE) {
        kf->innov_cnt++;
    }

    /* 计算窗口内新息的方差（均方值）作为 R 的在线估计 */
    if (kf->innov_cnt >= 3) {  /* 至少 3 个样本才更新 R */
        float sum_sq = 0.0f;
        uint8_t i;
        for (i = 0; i < kf->innov_cnt; i++) {
            sum_sq += kf->innov_buf[i] * kf->innov_buf[i];
        }
        float R_est = sum_sq / (float)kf->innov_cnt;

        /* 限幅，防止极端值 */
        kf->R = clampf(R_est, AKF_R_MIN, AKF_R_MAX);
    }

    return kf->x_hat;
}

/*------------------------------------------------------------------------
 *  AKF_GetGain  —  获取当前卡尔曼增益（调试用）
 *----------------------------------------------------------------------*/
float AKF_GetGain(const AKF_Filter *kf)
{
    if (kf == (void*)0) return 0.0f;
    return kf->K;
}

/*------------------------------------------------------------------------
 *  AKF_GetR  —  获取当前自适应 R（调试用）
 *----------------------------------------------------------------------*/
float AKF_GetR(const AKF_Filter *kf)
{
    if (kf == (void*)0) return 0.0f;
    return kf->R;
}

/*------------------------------------------------------------------------
 *  AKF_SetQ  —  在线修改 Q（过程噪声）
 *----------------------------------------------------------------------*/
void AKF_SetQ(AKF_Filter *kf, float Q)
{
    if (kf == (void*)0) return;
    kf->Q = Q;
}

/*------------------------------------------------------------------------
 *  AKF_Reset  —  重置滤波器到新值（需要立即跳变时使用）
 *----------------------------------------------------------------------*/
void AKF_Reset(AKF_Filter *kf, float new_val)
{
    if (kf == (void*)0) return;
    kf->x_hat = new_val;
    kf->P     = 1.0f;
    kf->R     = kf->R_init;
    kf->K     = 0.0f;
    memset(kf->innov_buf, 0, sizeof(kf->innov_buf));
    kf->innov_idx = 0;
    kf->innov_cnt = 0;
}
