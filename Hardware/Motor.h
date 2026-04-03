#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

// 驱动初始化函数
void Motor_Init(void);

// 设置左轮速度：范围 -1000 到 1000 (正数前进，负数后退)
void Motor_SetLeftSpeed(int16_t Speed);

// 设置右轮速度：范围 -1000 到 1000 (正数前进，负数后退)
void Motor_SetRightSpeed(int16_t Speed);

// 快捷控制：同时设置左右轮速度
void Motor_SetSpeed(int16_t LeftSpeed, int16_t RightSpeed);

#endif