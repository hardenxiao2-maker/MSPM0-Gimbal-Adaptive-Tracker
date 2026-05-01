#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_i2c.h"
#include "ti_msp_dl_config.h"
#include "PWM.h"


void MotorL_SetPWM(int PWML)		//side为0时：左轮，为1时：右轮
{
	
	if (PWML>= 0)							//如果设置正转的PWM
	{
			DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_14);	//PB12置高电平
		DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_15);

		PWM_SetCompare1(PWML);				//设置PWM占空比

	}
	else									//否则，即设置反转的速度值
	{	DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_14);	//PB12置低电平
		DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_15);
		PWM_SetCompare1(-PWML);				//设置PWM占空比

	}
	
}
void MotorR_SetPWM(int PWMR)
{
	if (PWMR >= 0)							//如果设置正转的PWM
	{	DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_28);//PB12置低电平
		DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_31);
  

				//设置PWM占空比
	  PWM_SetCompare2(PWMR);
	}
	else									//否则，即设置反转的速度值
	{		     DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_28);	//PB12置高电平
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_31);	//PB13置低电平
		//PB13置高电平
				//设置PWM占空比
	  PWM_SetCompare2(-PWMR);
	}
}

void motor_stop(void)
{
    DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_14);
	DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_15);
	DL_TimerG_setCaptureCompareValue(PWM_1_INST, 0, GPIO_PWM_1_C1_IDX);
 	DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_12);
	DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_13);
	DL_TimerG_setCaptureCompareValue(PWM_1_INST, 0, GPIO_PWM_1_C0_IDX);
}

