#include "stm32f10x.h"
#include "Delay.h"

void HCSR04_Init(void)
{
    // 1. 开启 GPIO 时钟 (PB8, PB9)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 🛡️ 开启 TIM4 时钟 (挂载在 APB1 上)
    // 完美避开舵机的 TIM1，互不干扰！
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PB8 (Trig 触发引脚)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB9 (Echo 接收引脚)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    
    // 🛡️ 配置 TIM4 为 1us 计一次数的硬件秒表
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;     
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; // 72MHz / 72 = 1MHz (1us)
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    TIM_Cmd(TIM4, DISABLE); // 初始化后先关掉秒表
}

/**
  * @brief  获取超声波距离 (TIM4 硬件高精度无干扰版)
  */
uint16_t HCSR04_GetDistance(void)
{
    uint16_t Timeout = 0;
    
    // 1. 发送 15us 触发信号
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_us(15); 
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    
    // 2. 等待 Echo 拉高 (回波开始)
    Timeout = 0;
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_RESET)
    {
        Timeout++;
        Delay_us(1);
        if(Timeout > 10000) return 999; // 10ms 没反应认为前方空旷
    }
    
    // 3. 🎯 回波来了！立刻清零并启动 TIM4 硬件秒表！
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
    
    // 4. 等待 Echo 拉低 (回波结束)
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_SET)
    {
        // 🛡️ 硬件防卡死：超过 50000us (约8.5米) 强制退出
        // 哪怕舵机瞬间大电流把超声波“劈瞎了”，最多 50ms 就会被无情踢出，绝不拖死主循环！
        if(TIM_GetCounter(TIM4) > 50000) 
        {
            TIM_Cmd(TIM4, DISABLE);
            return 999; 
        }
    }
    
    // 5. 🎯 回波结束，立刻按下秒表暂停键，并读取时间
    TIM_Cmd(TIM4, DISABLE);
    uint16_t time_us = TIM_GetCounter(TIM4);
    
    // 6. 标准公式计算距离
    float Distance = time_us / 58.0; 
    
    if(Distance > 999) return 999;
    return (uint16_t)Distance;
}