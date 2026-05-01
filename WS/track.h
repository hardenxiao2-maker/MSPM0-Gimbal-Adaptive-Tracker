#ifndef	__TRACK_H__
#define __TRACK_H__

#include "board.h"

// 调试开关
#define GYRO_DEBUG	0

// 定义一个结构体来存储
typedef struct {
    float x;
    float y;
    float z;
} Gyro_Struct;

extern volatile Gyro_Struct Gyro_Structure;

// 模块地址
#define	IIC_ADDR		0x50
// 航向角地址
#define YAW_REG_ADDR	0x3F	
// 寄存器解锁
#define UN_REG			0x69
// 保存寄存器
#define SAVE_REG		0x00
// 角度参考寄存器
#define ANGLE_REFER_REG	0x01

//设置SDA输出模式
/*#define SDA_OUT()   {                                                  \
                        DL_GPIO_initDigitalOutput(GYRO_SDA_IOMUX);     \
                        DL_GPIO_setPins(GYRO_PORT, GYRO_SDA_PIN);      \
                        DL_GPIO_enableOutput(GYRO_PORT, GYRO_SDA_PIN); \
                    }
//设置SDA输入模式
#define SDA_IN()    { DL_GPIO_initDigitalInput(GYRO_SDA_IOMUX); }

//获取SDA引脚的电平变化
#define SDA_GET()   ( ( ( DL_GPIO_readPins(GYRO_PORT,GYRO_SDA_PIN) & GYRO_SDA_PIN ) > 0 ) ? 1 : 0 )

//SDA与SCL输出
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(GYRO_PORT,GYRO_SDA_PIN)) : (DL_GPIO_clearPins(GYRO_PORT,GYRO_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(GYRO_PORT,GYRO_SCL_PIN)) : (DL_GPIO_clearPins(GYRO_PORT,GYRO_SCL_PIN)) )*/

short get_angle(void);

int gyro_control(int angle);

#endif