#ifndef _BUTTON_H_
#define _BUTTON_H_
#include "button.h"
#include "ti_msp_dl_config.h"

// 1. 先定义枚举类型（确保函数使用前可见）
typedef enum {
    BUTTON_NONE = 0,
    BUTTON_1_PRESSED,
    BUTTON_2_PRESSED,
    BUTTON_3_PRESSED
} ButtonState;




ButtonState Button_GetState(uint32_t status);  // 正确声明


#endif  // _BUTTON_H_