#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "ti_msp_dl_config.h"

void encoder_init(void);
int get_encoder_countL(void);
int get_encoder_countR(void);
void encoder_update(void);


    volatile long long temp_countL,temp_countR; //保存实时计数值
    int count;         				//根据定时器时间更新的计数值
    volatile long long TempL,TempR;       //修改为小写，保持命名统一


#endif