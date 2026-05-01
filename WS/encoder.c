#include "encoder.h"
#include "ti_msp_dl_config.h"


//编码器初始化
void encoder_init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
	NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

//获取编码器的值
int get_encoder_countL(void)
{
	return TempL;
}
int get_encoder_countR(void)
{
	return TempR;
}
void encoder_update(void)
{   
	TempL=temp_countL;
	TempR=temp_countR;
	temp_countL = 0;//编码器计数值清零
	temp_countR= 0;//编码器计数值清零
    
}


//外部中断处理函数



