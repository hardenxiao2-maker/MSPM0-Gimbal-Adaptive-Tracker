#include "ti_msp_dl_config.h"
#include "track.h"
#include "board.h"
#include "stdio.h"
#include "motor.h"
#include "monitor.h"
#include "bsp_mpu6050.h"

volatile Gyro_Struct Gyro_Structure;


float gyro_Kp = 150, gyro_Ki=0, gyro_Kd =5.5;//PID参数
float gyro_P = 0, gyro_I = 0, gyro_D = 0, gyro_PID_value = 0;
float gyro_new_error = 0, gyro_previous_error = 0;
static int gyro_initial_motor_speed = 3000;//基础速度

/******************************************************************
 * 函 数 名 称：get_angle
 * 函 数 说 明：读角度数据
 * 函 数 形 参：无
 * 函 数 返 回：返回结构体
 * 作       者：LC
 * 备       注：无
******************************************************************/
short get_angle(void)
{
	short a[3];
	MPU6050ReadGyro(a);
	return a[2];
}


float gyro_pid(int angle)
{  
	
	gyro_new_error = get_angle();
	gyro_new_error = gyro_new_error - angle;
//	printf("gyro_new_error = %f\n",gyro_new_error);
	gyro_P = gyro_new_error;	        //当前误差
	gyro_I = gyro_I+gyro_new_error;	        //误差累加
	gyro_D = gyro_new_error-gyro_previous_error;	//当前误差与之前误差的误差
	 
	gyro_PID_value=(gyro_Kp*gyro_P)+(gyro_Ki*gyro_I)+(gyro_Kd*gyro_D);
	
	gyro_previous_error=gyro_new_error;//更新之前误差
	
	return gyro_PID_value; //返回控制值
}

//电机动作
void  gyro_motorsWrite(int speedL,int speedR)
{
	MotorL_SetPWM(speedL);
    MotorR_SetPWM(speedR);
}

int gyro_Set_PWM(float pwm_value)
{
	//基础速度+PID值
	int left_motor_speed = pwm_value;
	int right_motor_speed = -pwm_value;
	
	printf("%.2f,%d,%d\n",gyro_new_error,left_motor_speed,right_motor_speed);
	
	if(left_motor_speed>9999) left_motor_speed=9999;
	else if(left_motor_speed<-9999) left_motor_speed=-9999;
	if(right_motor_speed>9999) right_motor_speed=9999;
	else if(right_motor_speed<-9999) right_motor_speed=-9999;
	
	//旋转到目标角度则停止
	if( gyro_new_error >= -0.5 && gyro_new_error <= 0.5 )
	{
		motor_stop();
		return 1;
	}
	gyro_motorsWrite(left_motor_speed,right_motor_speed);
	return 0;
}
//返回1说明旋转到位  返回0说明还在旋转
int gyro_control(int angle)
{
	return gyro_Set_PWM(gyro_pid(angle));
}
