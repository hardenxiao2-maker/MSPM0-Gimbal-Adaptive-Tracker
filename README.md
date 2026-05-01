# 双轴步进电机云台视觉跟踪系统

基于 TI MSPM0G3507 的双轴步进电机云台闭环跟踪系统，通过视觉传感器实时获取目标坐标，经自适应卡尔曼滤波处理后驱动 X/Y 轴步进电机进行精确跟踪。

## 系统架构

```
视觉传感器 ──UART──> MSPM0G3507 ──GPIO脉冲──> X/Y步进电机驱动器
                         │
                   ┌─────┴─────┐
                   │  控制核心  │
                   ├───────────┤
                   │ 卡尔曼滤波 │
                   │ PID 控制器 │
                   │ DDS 脉冲发生│
                   │ 状态机管理  │
                   └───────────┘
```

## 功能特性

- **双轴独立控制**：X/Y 轴各自拥有独立的 PID 控制器和 DDS 脉冲发生器
- **自适应卡尔曼滤波**：基于新息序列在线估计测量噪声 R，自动适应环境变化
- **DDS 频率合成**：10kHz 定时器中断 + 相位累加器，实现任意精细频率的脉冲输出
- **双状态锁定机制**：追击/锁定状态机 + 时间防抖，有效过滤视觉噪声干扰
- **分段限速策略**：远/中/近三段速度包络，兼顾响应速度与停车精度
- **目标丢失检测**：识别传感器默认值并自动进入保持模式
- **PID 自动整定**：集成 Twiddle 坐标上升法，可离线自动优化 PID 参数
- **循迹底盘控制**：五路灰度传感器巡线 + 编码器速度闭环

## 硬件平台

| 组件 | 型号/规格 |
|------|-----------|
| 主控 MCU | TI MSPM0G3507 (Cortex-M0+, 32MHz) |
| X 轴电机 | 步进电机 + ZDT X57 V2 闭环驱动器 |
| Y 轴电机 | 步进电机 + 脉冲方向式驱动器 |
| 视觉传感器 | OpenMV / 类似模块 (UART 输出坐标) |
| 显示 | 0.96" OLED (SSD1306, I2C) |
| IMU | MPU6050 |

### 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| X 轴 STP | PA21 | 步进脉冲 |
| X 轴 DIR | PB21 | 方向控制 |
| Y 轴 PUL | PA30 | 步进脉冲 |
| Y 轴 DIR | PB23 | 方向控制 |
| Y 轴 EN | PB22 | 驱动器使能 |
| UART0 TX/RX | - | 调试串口 / 步进驱动通信 |
| UART1 TX/RX | - | 视觉传感器数据接收 |

## 项目结构

```
.
├── empty.c                 # 主程序入口，UART 数据解析与控制主循环
├── adaptive_kalman.c/h     # 自适应卡尔曼滤波器
├── stepper_x_ctrl.c/h      # X 轴步进电机控制 (TIMG12)
├── stepper_y_ctrl.c/h      # Y 轴步进电机控制 (TIMER_0)
├── pid_optimizer.c/h       # Twiddle PID 自动整定算法
├── empty.syscfg            # TI SysConfig 外设配置
├── Board/
│   ├── board.c/h           # 板级支持 (延时、调试打印)
├── BSP/
│   └── OLED/               # SSD1306 OLED 驱动
├── WS/
│   ├── monitor.c/h         # 灰度巡线传感器 + 底盘 PID
│   ├── motor.c/h           # 直流电机 PWM 驱动
│   ├── encoder.c/h         # 编码器计数
│   ├── PWM.c/h             # PWM 输出封装
│   ├── button.c/h          # 按键处理
│   ├── track.c/h           # 巡线逻辑
│   ├── bsp_mpu6050.c/h     # MPU6050 IMU 驱动
│   └── inv_mpu*.c/h        # InvenSense MPU DMP 库
├── targetConfigs/          # CCS 调试目标配置
└── Debug/                  # 编译输出 (不纳入版本控制)
```

## 核心算法

### 自适应卡尔曼滤波 (AKF)

采用标准一维卡尔曼滤波框架，在每次更新后通过新息（innovation）滑动窗口在线估计测量噪声协方差 R：

```
预测: x̂⁻ = x̂,  P⁻ = P + Q
更新: K = P⁻/(P⁻+R),  x̂ = x̂⁻ + K·(z - x̂⁻),  P = (1-K)·P⁻
自适应: R = clamp(mean(innovation²), R_MIN, R_MAX)
```

### DDS 脉冲发生

利用固定 10kHz 的定时器中断作为时基，通过 16 位相位累加器实现任意频率的脉冲输出：

```
phase_acc += phase_inc  (每次中断)
if phase_acc >= 65536:  输出一个步进脉冲
其中 phase_inc = (target_freq × 65536) / 10000
```

### 双状态锁定机制

```
Hunting (追击) ──误差<1.5px──> Locked (锁定)
   ^                              │
   └──连续3帧误差>6.5px──────────┘
```

## 开发环境

- **IDE**: Code Composer Studio (CCS) Theia
- **SDK**: TI MSPM0 SDK
- **编译器**: TI Clang (ticlang)
- **调试器**: XDS-110

## 编译与烧录

1. 使用 CCS Theia 打开本项目
2. 确认已安装 MSPM0 SDK 并正确配置工具链路径
3. `Project` → `Build Project` 编译
4. 连接 LP-MSPM0G3507 LaunchPad，点击 `Debug` 烧录运行

## 串口协议

视觉传感器通过 UART1 (115200 baud) 发送目标坐标：

```
格式: x,y\r\n
示例: 95.3,42.7\r\n
```

调试输出通过 UART0 回传滤波数据和状态信息。

## 参数调优

主要可调参数位于 `empty.c` 顶部：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Y_Kp / Y_Ki / Y_Kd` | 1.2 / 0.0 / 2.0 | Y 轴 PID 参数 |
| `X_Kp / X_Ki / X_Kd` | 0.5 / 0.0 / 1.5 | X 轴 PID 参数 |
| `LASER_OFFSET_X/Y` | 20.0 / 15.0 | 激光与相机光轴偏移补偿 (像素) |
| `AKF_WINDOW_SIZE` | 5 | 卡尔曼滤波新息窗口大小 |
| `AKF_R_MIN / R_MAX` | 0.1 / 5.0 | 自适应 R 的限幅范围 |

## License

MIT
