#include "ti_msp_dl_config.h"


void PWM_SetCompare1(int Compare)
{
	DL_TimerG_setCaptureCompareValue(PWM_1_INST,Compare,GPIO_PWM_1_C1_IDX);		//设置CCR1的值
}

void PWM_SetCompare2(int Compare)
{
	DL_TimerG_setCaptureCompareValue(PWM_1_INST,Compare,GPIO_PWM_1_C0_IDX);	//设置CCR2的值
}
