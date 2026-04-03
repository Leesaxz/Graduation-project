#include "stm32f10x.h"                  // Device header

/**
  * @brief  舵机 PWM 初始化 (使用 PA8 -> TIM1_CH1)
  */
void PWM_SG90_Init(void)
{
    // 1. 开启时钟：注意！TIM1 是高级定时器，挂在更快的 APB2 总线上！
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // 2. 初始化 PA8 引脚 (复用推挽输出)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 配置 TIM1 时基单元 (产生周期为 20ms 的 PWM，供舵机使用)
    // 72MHz / 72 = 1MHz (1us数一次)，数 20000 次就是 20000us = 20ms
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR: 周期
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC: 预分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;    // 高级定时器专属，设为0
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    // 4. 配置 TIM1 的输出比较通道 1
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure); // 先赋默认值是个好习惯
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;      // CCR: 初始占空比设为0
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); // 注意：这里必须是 OC1Init，因为是通道1！
    
    // 5. 👑 高级定时器终极魔法：主输出使能！没有这句绝对出不来波形！
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    // 6. 开启 TIM1
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * @brief  设置 TIM1 通道 1 的比较值
  */
void PWM_SG90_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM1, Compare); // 注意这里变成了 SetCompare1 和 TIM1
}

/**
  * @brief  设置舵机角度
  * @param  Angle 角度范围 0 ~ 180
  */
void Servo_SetAngle(float Angle)
{
    // SG90 要求 0度对应 0.5ms(500)，180度对应 2.5ms(2500)
    PWM_SG90_SetCompare1(Angle / 180 * 2000 + 500);
}