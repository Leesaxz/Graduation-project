#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init(void);
// 获取左轮速度 (读取后立刻清零，相当于测速)
int16_t Encoder_GetLeftSpeed(void);
// 获取右轮速度
int16_t Encoder_GetRightSpeed(void);

#endif