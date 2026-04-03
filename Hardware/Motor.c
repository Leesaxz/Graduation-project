#include "Motor.h"

/**
  * @brief  电机驱动初始化 (包括方向引脚和 PWM 引脚)
  */
void Motor_Init(void)
{
    // 1. 开启时钟：GPIOA, GPIOB 和 定时器 TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 2. 初始化方向控制引脚 (普通推挽输出)
    // 左轮 AIN1(PA4), AIN2(PA5)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 右轮 BIN1(PB0), BIN2(PB1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 初始化 PWM 输出引脚 (复用推挽输出交由定时器控制)
    // PWMA(PA6 -> TIM3_CH1), PWMB(PA7 -> TIM3_CH2)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 4. 配置 TIM3 时基单元 (产生 1kHz 的 PWM)
    // 72MHz / 72 / 1000 = 1000Hz = 1kHz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;     // ARR: 决定了 PWM 的分辨率是 1000
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;    // PSC: 预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 5. 配置 TIM3 的输出比较通道 1 和 2 (产生 PWM 波)
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure); // 给结构体赋默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;      // 初始速度为 0
    
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // 初始化通道 1 (PA6)
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // 初始化通道 2 (PA7)
    
    // 6. 使能 TIM3
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  设置左轮速度
  * @param  Speed 速度值，范围：-1000~1000
  */
void Motor_SetLeftSpeed(int16_t Speed)
{
    if (Speed >= 0) // 前进
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);   // AIN1 = 1
        GPIO_ResetBits(GPIOA, GPIO_Pin_5); // AIN2 = 0
        TIM_SetCompare1(TIM3, Speed);      // 设置 PWM 占空比
    }
    else // 后退
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4); // AIN1 = 0
        GPIO_SetBits(GPIOA, GPIO_Pin_5);   // AIN2 = 1
        TIM_SetCompare1(TIM3, -Speed);     // 占空比不能是负数，所以取反
    }
}

/**
  * @brief  设置右轮速度
  * @param  Speed 速度值，范围：-1000~1000
  */
void Motor_SetRightSpeed(int16_t Speed)
{
    if (Speed >= 0) // 前进
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);   // BIN1 = 1
        GPIO_ResetBits(GPIOB, GPIO_Pin_1); // BIN2 = 0
        TIM_SetCompare2(TIM3, Speed);
    }
    else // 后退
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0); // BIN1 = 0
        GPIO_SetBits(GPIOB, GPIO_Pin_1);   // BIN2 = 1
        TIM_SetCompare2(TIM3, -Speed);
    }
}

/**
  * @brief  同时设置双轮速度
  */
void Motor_SetSpeed(int16_t LeftSpeed, int16_t RightSpeed)
{
	LeftSpeed = -LeftSpeed;
//    RightSpeed = -RightSpeed;
    Motor_SetLeftSpeed(LeftSpeed);
    Motor_SetRightSpeed(RightSpeed);
}