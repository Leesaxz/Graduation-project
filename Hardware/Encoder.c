#include "stm32f10x.h"

void Encoder_Init(void)
{
    // 1. 开启 GPIO 和 定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 2. 配置引脚为浮空输入 (PA0, PA1 接 TIM2; PB6, PB7 接 TIM4)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 配置 TIM2 和 TIM4 为编码器模式 (双边沿计数，精度最高)
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 4. 开启定时器
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// 获取左轮速度
int16_t Encoder_GetLeftSpeed(void) {
    int16_t speed = TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0);
    // 🚨 战地修复处：如果左轮一通电就原地疯狂暴走，把这里改成 return -speed;
    return -speed; 
}

// 获取右轮速度
int16_t Encoder_GetRightSpeed(void) {
    int16_t speed = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    // 🚨 战地修复处：如果右轮一通电就原地疯狂暴走，把这里改成 return -speed;
    return speed; 
}
