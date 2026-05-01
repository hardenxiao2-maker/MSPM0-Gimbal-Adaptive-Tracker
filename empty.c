#include "ti/devices/msp/m0p/mspm0g350x.h"
#include <stdio.h>
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "oled.h"
#include "bmp.h"
#include "encoder.h"
#include "mid_timer.h"
#include "motor.h"
#include "PWM.h"
#include "math.h"
#include "button.h"
#include "monitor.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "bsp_mpu6050.h"
#include "stepper_y_ctrl.h"
#include "adaptive_kalman.h"
#include "pid_optimizer.h"
#include "stepper_x_ctrl.h"

short MPU6050ReadGyroZ(void);
float  ActualL, ActualR,OutL,OutR,TargetR,TargetL, target=4.8f;			//目标值，实际值，输出值
float KpL, KiL, KdL,KpR,KiR,KdR;					//比例项，积分项，微分项的权重
float ErrorL0, ErrorL1, ErrorIntL,ErrorR0, ErrorR1, ErrorIntR,e_last;		//本次误差，上次误差，误差积分
static uint16_t Count;
volatile unsigned int delay_times = 0;
uint8_t task1_state=0;
uint8_t task2_state=0;
float L2, L1, M0, R1, R2;
float TargetL1,TargetR1;
uint32_t a;
uint8_t N=0;
uint8_t zhijiao=0;
short startyaw,currentyaw,tagetyaw;
// 全局变量：电机转速（0~100，PWM占空比或速度等级）
int left_motor_speed;   // 左前轮转速
int right_motor_speed;  // 右前轮转速
int base_speed = 370;    // 基准前进速度（可根据实际调整）
int max_speed = 666;    // 电机最大转速
int min_speed = 0;      // 电机最小转速（0为停止）
int count_locked = 0; 
// PID参数（需根据电机特性重新调试）
float Kp =80.0f, Ki=25.0f, Kd = 10.0f;
float e = 0, e_last = 0, integral = 0;
float dt = 0.02f;       // 采样间隔20ms
float output;
// Y轴电机PID参数
float Y_Kp = 1.2f;     // 提高比例，让Y轴响应更敏捷
float Y_Ki = 0.0f;     // 坚决不用积分
float Y_Kd = 2.0f;     // 提高阻尼，抵消响应变快带来的超调

// X轴电机PID参数 
float X_Kp = 0.5f;     // 适当提高比例，拒绝慢吞吞
float X_Ki = 0.0f;     
float X_Kd = 1.5f;     // 增加微分刹车：防止到达目标后因为惯性或滤波延迟导致继续滑动（停不下）

int right_angle_cnt = 0;  // 直角数（转弯结束瞬间直接加一）
int lap_cnt = 0;
int btn1_debounce_cnt = 0;
int btn2_debounce_cnt = 0;
int btn1_pressed = 0;
int btn2_pressed = 0;
#define DEBOUNCE_MS 30
int square_wave_cnt = 0;  // 完整方波计次器（初始值0）
// 接收缓冲区配置（大小根据最大数据包长度调整）
#define UART_RX_BUF_SIZE 64
volatile char uart_rx_buf[UART_RX_BUF_SIZE] = {0};  // 存储接收的完整数据
volatile uint8_t rx_idx = 0;                        // 缓冲区索引
volatile bool rx_complete = false;                  // 接收完成标志
// 存储解析后的x、y数值
float x = 80.0f, y = 60.0f;
float x_filtered = 80.0f, y_filtered = 60.0f;  // 卡尔曼滤波后的值
AKF_Filter kf_x, kf_y;                         // x、y 自适应卡尔曼滤波器
bool kf_initialized = false;                    // 滤波器是否已初始化
#define KF_INIT_SAMPLES  5                      // 初始化采样帧数
uint8_t kf_init_cnt = 0;                        // 已采集帧数
float kf_init_sum_x = 0.0f;                     // x 累加器
float kf_init_sum_y = 0.0f;                     // y 累加器
int  pa=0,pb=0,cli=0;
bool is_wave_stop = false;
// 函数声明（串口0用于回传，串口1用于接收和初始化提示）
void uart0_send_char(char ch);
void uart0_send_string(char* str);
void uart1_send_char(char ch);
void uart1_send_string(char* str);
bool parse_xy(const char* str, float* x, float* y);  // 解析x,y字符串为数字
// 新增全局变量：OLED刷新计数器
static uint32_t oled_refresh_cnt = 0;
int main(void)
{
       // 初始化系统配置（包括UART0和UART1）
    SYSCFG_DL_init();

    // 配置串口1中断（负责接收数据）
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    // 初始化提示通过串口1发送（告知外设接收通道已就绪）
    uart1_send_string("UART1 已启动，等待接收数据（以\\r\\n结束）...\r\n");
    // 额外通过串口0提示回传通道就绪
    // 额外通过串口0提示回传通道就绪 (已注释，避免干扰电机)
    // uart0_send_string("UART0 回传通道已启动...\r\n");
	timer_init();
	encoder_init();
    StepperY_Init();
    StepperY_SetPID(Y_Kp, Y_Ki, Y_Kd);  // 只在初始化时设一次，避免每帧重置积分

    StepperX_Init();
    StepperX_SetPID(X_Kp, X_Ki, X_Kd);  // X轴同样只在初始化设置一次

    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	KpL = 4.5;				//修改Kp，调整范围：0~2
	KiL =2.0;				//修改Ki，调整范围：0~2
	KdL = 0;		//修改Kd，调整范围：0~2
	KpR = 4.5;				//修改Kp，调整范围：0~2
	KiR	= 2.0;				//修改Ki，调整范围：0~2
	KdR = 0;
    TargetL=0;
	TargetR=0;
	/*startyaw=MPU6050ReadGyroZ();
	tagetyaw=startyaw+90;*/
	volatile int32_t i = 0;
    volatile bool cntDir = 1; // 方向标志：false=递增（正转），true=递减（反转）
    uint32_t stp_port_val;
    float stp_level;

    // 启用 UART0 接收中断
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);


      delay_ms(3000);

    uart0_send_string("系统已启动，等待首次数据初始化滤波器...\r\n");

    while(1)
    {
        // Stepper_Loop(); // 循环调用
         //OLED_ShowString(60,32,"sq:",16,1);
         //OLED_ShowNum(95,32,square_wave_cnt,4,16,1,0);
         //OLED_ShowString(60,48,"vel:",16,1);
        // OLED_ShowNum(95,48,stp_level,3,16,1,0);
         //OLED_Refresh();

        // X轴 PID 测试序列
        // 动作：正转90° -> 反转130° -> 正转190° -> 回到0
        // X轴 PID 测试序列
        // 动作：正转90° -> 反转130° -> 正转190° -> 回到0
        if (rx_complete)
        {
            // 复制缓冲区数据（避免中断修改影响解析）
            char temp_buf[UART_RX_BUF_SIZE];
            memcpy(temp_buf, (const char*)uart_rx_buf, UART_RX_BUF_SIZE);
            temp_buf[UART_RX_BUF_SIZE - 1] = '\0';

            // 解析x,y字符串
            if (parse_xy(temp_buf, &x, &y))
            {
                // 将像素坐标转为偏离画面中心的偏移量
                // 摄像头分辨率 160×120，中心 (80, 60)
                // PID 目标是 0，所以传入偏移量，目标居中时偏移为 0
                // 正数使激光左移修正
                // 根据实际安装偏移量调整
                #define LASER_OFFSET_X  20.0f  // 激光X轴偏移补偿
                #define LASER_OFFSET_Y  15.0f   // 激光准星Y轴补偿量

                #define CAM_CENTER_X  (80.0f + LASER_OFFSET_X)
                #define CAM_CENTER_Y  (60.0f + LASER_OFFSET_Y)
                
                // 目标丢失检测
                // 视觉传感器在找不到目标时会默认发送 (80.0, 60.0)。
                // 如果是这种情况，我们直接拦截，不走任何滤波和PID修正，强制双电机停机并清空状态。
                if (fabs(x - 80.0f) < 0.1f && fabs(y - 60.0f) < 0.1f) {
                    StepperX_SetFrequency(0);
                    StepperY_SetFrequency(0);
                    // 可选：清理 PID 积分和历史误差
                    // 这里为了保持外部封装简洁，可以直接 return 或者跳出当前逻辑
                    continue;  // 跳过本次控制周期的后续滤波和 PID 计算，保持静止！
                }

                float x_corrected = x - CAM_CENTER_X;
                float y_corrected = y - CAM_CENTER_Y;

                // 滤波器初始化：收集前 N 帧取平均值
                if (!kf_initialized) {
                    kf_init_sum_x += x_corrected;
                    kf_init_sum_y += y_corrected;
                    kf_init_cnt++;

                    char init_buf[64];
                    sprintf(init_buf, "采样中... %d/%d\r\n", kf_init_cnt, KF_INIT_SAMPLES);
                    uart0_send_string(init_buf);

                    if (kf_init_cnt >= KF_INIT_SAMPLES) {
                        float avg_x = kf_init_sum_x / KF_INIT_SAMPLES;
                        float avg_y = kf_init_sum_y / KF_INIT_SAMPLES;
                        // 滤波参数设置:
                        // Q=50.0 (高过程噪声): 快速跟随新测量值
                        // R=0.1 (低测量噪声): 信任原始测量值
                        AKF_Init(&kf_x, avg_x, 50.0f, 0.1f);
                        AKF_Init(&kf_y, avg_y, 50.0f, 0.1f);
                        kf_initialized = true;

                        sprintf(init_buf, "滤波器初始化:x=%.2f,y=%.2f\r\n", avg_x, avg_y);
                        uart0_send_string(init_buf);
                    }
                    // 采样期间不执行 PID，跳过后续逻辑
                } else {
                    // 自适应卡尔曼滤波处理
                    x_filtered = AKF_Update(&kf_x, x_corrected);
                    y_filtered = AKF_Update(&kf_y, y_corrected);

                    // 打印偏移值 vs 滤波值（同一坐标系，方便对比）
                    char print_buf[128];
                    sprintf(print_buf, "偏移:x=%.1f,y=%.1f  滤波:x=%.1f,y=%.1f\r\n",
                            x_corrected, y_corrected, x_filtered, y_filtered);
                    uart0_send_string(print_buf);

                    // Y 轴 PID 控制（取负号匹配电机方向）
                    StepperY_PID_Ctrl(-y_filtered);

                    // X 轴 PID 控制
                    // 说明：x_filtered已经是偏离画面中心的坐标差(x - 80)。
                    // 如果发现方向反了，把它改成 StepperX_PID_Ctrl(-x_filtered) 即可。
                    StepperX_PID_Ctrl(x_filtered);
                }
            }
            else
            {
                // 解析失败：提示格式错误
                uart0_send_string("解析失败！请发送\"x,y\"格式（例如：12.3,45）\r\n");
            }

            // 重置缓冲区，准备下次接收9
            memset((char*)uart_rx_buf, 0, UART_RX_BUF_SIZE);
            rx_idx = 0;
            rx_complete = false;
        }
		/*
         OLED_ShowString(0,16,"BP:",16,1);
         OLED_ShowNum(25,16,btn1_pressed,3,16,1,0);
         OLED_ShowString(0,32,"tg:",16,1);
         OLED_ShowNum(25,32,btn2_pressed,3,16,1,0);
         OLED_ShowString(0,48,"x:",16,1);
         OLED_ShowNum(25,48,x,3,16,1,0);
         OLED_ShowString(60,16,"y:",16,1);
         OLED_ShowNum(95,16,y,3,16,1,0);
         OLED_ShowString(60,32,"ANG:",16,1);
         OLED_ShowNum(95,32,right_angle_cnt ,3,16,1,0);
         OLED_ShowString(60,48,"LAP:",16,1);
         OLED_ShowNum(95,48,lap_cnt,3,16,1,0);
         OLED_Refresh();
         //delay_ms(50);*/
// -------------------------- Stp引脚（PA21）翻转：修复电平判断逻辑 --------------------------
// 1. 读取PA21电平，提取为0/1逻辑
 //stp_port_val = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_21);
 //stp_level = (stp_port_val & DL_GPIO_PIN_21) ? 1 : 0; // 高=1，低=0




// 2. 按0/1逻辑翻转电平
 // 获取当前按键状态
        // 根据状态调用对应任务（使用声明的state变量）
      /*DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_21);  // 强制拉高
delay_us(100);
DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_21); // 强制拉低
delay_us(100);*/
	}
}
// 解析"x,y"格式字符串为x、y数字（支持整数、浮点数、负数）
bool parse_xy(const char* str, float* x, float* y)
{
    if (str == NULL || x == NULL || y == NULL)
        return false;

    // 查找逗号分隔符（核心分割逻辑）
    char* comma = strchr((char*)str, ',');
    if (comma == NULL)  // 没有逗号，格式错误
        return false;

    // 分割x部分（从开头到逗号前）
    char x_str[32] = {0};
    uint16_t x_len = comma - str;  // x部分长度
    if (x_len <= 0 || x_len >= 32)  // x为空或过长
        return false;
    strncpy(x_str, str, x_len);
    x_str[x_len] = '\0';  // 手动添加字符串结束符

    // 分割y部分（从逗号后到结尾）
    char y_str[32] = {0};
    strncpy(y_str, comma + 1, 31);  // 留1位给结束符
    y_str[31] = '\0';

    // 转换为浮点数（支持整数、小数、负数，例如"123"→123.0，"-45.6"→-45.6）
    *x = atof(x_str);
    *y = atof(y_str);

    return true;  // 解析成功
}


short MPU6050ReadGyroZ(void)
{
    short gyroData[3] = {0};  // 存储三轴数据的临时数组
    uint8_t buf[6] = {0};     // 存储寄存器原始字节
    uint8_t reg_status = 0;   // 读取状态

    // 读取陀螺仪6个字节数据（X、Y、Z轴，每个轴2字节）
    reg_status = MPU6050_ReadData(0x68, MPU6050_GYRO_OUT, 6, buf);
    
    if (reg_status == 0)  // 读取成功
    {
        // 拼接Z轴数据（对应gyroData[2]）
        gyroData[2] = (buf[4] << 8) | buf[5];  // Z轴高8位+低8位
        return gyroData[2];
    }
    else  // 读取失败（如IIC通信错误）
    {
        return 0;  // 返回0表示失败，可根据需求修改为其他错误码
    }
}
// 串口0发送单个字符（用于回传）
void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);  // 等待串口0空闲
    DL_UART_Main_transmitData(UART_0_INST, ch);   // 发送字符
}

// 串口0发送字符串（用于回传）
void uart0_send_string(char* str)
{
    if (str == NULL || *str == '\0') return;
    while (*str != '\0')
    {
        uart0_send_char(*str++);
    }
}

// 串口1发送单个字符（仅用于初始化提示）
void uart1_send_char(char ch)
{
    while (DL_UART_isBusy(UART_1_INST) == true);  // 等待串口1空闲
    DL_UART_Main_transmitData(UART_1_INST, ch);   // 发送字符
}

// 串口1发送字符串（仅用于初始化提示）
void uart1_send_string(char* str)
{
    if (str == NULL || *str == '\0') return;
    while (*str != '\0')
    {
        uart1_send_char(*str++);
    }
}

// 串口1中断服务函数（负责接收数据）
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_IIDX_RX:  // 接收中断（新字符到来）
        {
            char recv_char = DL_UART_Main_receiveData(UART_1_INST);  // 读取串口1接收的字符

            // 处理逻辑：未完成接收且缓冲区未满
            if (!rx_complete && rx_idx < UART_RX_BUF_SIZE - 1)
            {
                // 检测结束标志：连续收到\r和\n，视为数据包结束
                if (recv_char == '\n' && rx_idx > 0 && uart_rx_buf[rx_idx - 1] == '\r')
                {
                    uart_rx_buf[rx_idx - 1] = '\0';  // 用终止符替换\r，去掉结束标志
                    rx_complete = true;              // 标记接收完成
                }
                else
                {
                    // 存储接收的字符（保留所有内容）
                    uart_rx_buf[rx_idx++] = recv_char;
                }
            }
            else if (rx_idx >= UART_RX_BUF_SIZE - 1)
            {
                // 缓冲区满但未收到结束标志，通过串口0回传警告
                uart_rx_buf[rx_idx] = '\0';
                rx_complete = true;

            }
            break;
        }
        default:
            break;
    }
}
// 2. 中断处理函数：处理GPIOB按键中断
void GROUP1_IRQHandler(void)
{
 if (DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_25)) {
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_25);  // 清除中断标志
        
        // 启动防抖动计数（由定时器更新）
        btn1_debounce_cnt = DEBOUNCE_MS / 10;  // 30ms = 3*10ms
    }
    if (DL_GPIO_getEnabledInterruptStatus(GPIOA, DL_GPIO_PIN_29)) {
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_29);  // 清除中断标志
        
        // 启动防抖动计数（由定时器更新）
        btn2_debounce_cnt = DEBOUNCE_MS / 10;  // 30ms = 3*10ms
    }
    uint32_t gpio_status;
	//获取中断信号情况
	gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, DL_GPIO_PIN_26|DL_GPIO_PIN_27);
	//编码器A相上升沿触发
	if((gpio_status & DL_GPIO_PIN_26) == DL_GPIO_PIN_26)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_27))
		{
			temp_countL--;
		}
		else
		{
			temp_countL++;
		}
	}//编码器B相上升沿触发
    if((gpio_status & DL_GPIO_PIN_27)==DL_GPIO_PIN_27)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_26))
		{
			temp_countL++;
		}
		else
		{
			temp_countL--;
		}
	}
	//清除状态
	DL_GPIO_clearInterruptStatus(GPIOA,DL_GPIO_PIN_26|DL_GPIO_PIN_27);
    
	uint32_t gpioR_status;
	//获取中断信号情况
	gpioR_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, DL_GPIO_PIN_31|DL_GPIO_PIN_28);
	//编码器A相上升沿触发
	if((gpioR_status & DL_GPIO_PIN_31) == DL_GPIO_PIN_31)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_28))
		{
			temp_countR--;
		}
		else
		{
			temp_countR++;
		}
	}//编码器B相上升沿触发
	if((gpioR_status & DL_GPIO_PIN_28)==DL_GPIO_PIN_28)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_31))
		{
			temp_countR++;
		}
		else
		{
			temp_countR--;
		}
	}
	//清除状态
	DL_GPIO_clearInterruptStatus(GPIOA,DL_GPIO_PIN_28|DL_GPIO_PIN_31);
}


void TIMER_TICK_INST_IRQHandler(void)
{   
	//10ms归零中断触发
	if( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{  

         if (btn1_debounce_cnt > 0) {
            btn1_debounce_cnt--;
            // 防抖动结束，检查当前引脚状态（仍为低电平=真按下）
            if (btn1_debounce_cnt == 0 && !DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_26)) {
                btn1_pressed = !btn1_pressed;  // 确认按下，置处理标志
            }
        }
         if (btn2_debounce_cnt > 0) {
            btn2_debounce_cnt--;
            // 防抖动结束，检查当前引脚状态（仍为低电平=真按下）
            if (btn2_debounce_cnt == 0 && !DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_29)) {
                 btn2_pressed++;          // 每次按下+1
        if (btn2_pressed > 5) {  // 超过5则循环回0
            btn2_pressed = 0;
        }
            }
        }
			/*if(task1_state==1)
			{
				Task1_Handler();
			}
			if(task2_state==1)
			{
				Task2_Handler();
			}*/
		
		    //编码器更新
			/*currentyaw=MPU6050ReadGyroZ();*/
			if( btn1_pressed>0&&btn2_pressed!=lap_cnt)
            {
			Read_Monitor();
    
	
		    encoder_update();
            ActualR =get_encoder_countR();
            ActualL =-get_encoder_countL();
     			/*获取实际速度值*/
			/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
			/*此值正比于速度，所以可以表示速度，但它的单位并不是速度的标准单位*/
			/*此处每隔40ms获取一次计次值增量，电机旋转一周的计次值增量约为408*/
			/*因此如果想转换为标准单位，比如转/秒*/
			/*则可将此句代码改成Actual = Encoder_Get() / 408.0 / 0.04;*/
        calculate_pid();
        MotorR_SetPWM(left_motor_speed );
        MotorL_SetPWM(right_motor_speed );
            }
            else  if (btn2_pressed> 0 && lap_cnt >= btn2_pressed) {
            // 3.1 任务结束：清零所有计数
            right_angle_cnt = 0;    // 直角数清零
            lap_cnt = 0;            // 当前圈数清零
            btn2_pressed = 0;        // 目标圈数清零（重置按钮2的设置）

            // 3.2 任务结束：小车停止
            MotorR_SetPWM(0);       // 右电机停
            MotorL_SetPWM(0);       // 左电机停

            // 3.3 重置状态，确保下次任务可正常启动
            count_locked = 0;       // 解锁计数
            // （可选）停止循迹使能（如果依赖按钮1的track_enable）
            // track_enable = 0;  
        }
     
	}
}



void Delay_ms(unsigned int ms)
{
    delay_times = ms;
    while( delay_times != 0 );
}



//滴答定时器中断服务函数
void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}



// 串口0中断服务函数（负责接收步进电机返回的数据并转发到串口1）
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            // 读取串口0接收到的数据
            volatile uint8_t data = DL_UART_Main_receiveData(UART_0_INST);
            // 转发到串口1
            uart1_send_char(data);
            break;
        default:
            break;
    }
}