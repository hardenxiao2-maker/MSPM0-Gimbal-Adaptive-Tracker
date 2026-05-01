#include "ti_msp_dl_config.h"
#include "button.h"

// 1. 函数原型声明：必须带返回类型 ButtonState

// 3. 按键状态判断函数：必须带返回类型 ButtonState
ButtonState Button_GetState(uint32_t status)  // 补充返回类型
{
    // 检测哪个引脚触发中断（根据实际硬件电平判断，此处假设高电平有效）
    if((status & DL_GPIO_PIN_25) == DL_GPIO_PIN_25) 
    {
        return BUTTON_1_PRESSED;
    } 
    else if((status & DL_GPIO_PIN_29) == DL_GPIO_PIN_29) 
    {
        return BUTTON_2_PRESSED;
    } 
    else if((status & DL_GPIO_PIN_26) == DL_GPIO_PIN_26) 
    {
        return BUTTON_3_PRESSED;
    } 
    
     return BUTTON_NONE;  // 无按键按下
}



// 4. 任务处理函数（控制LED）
