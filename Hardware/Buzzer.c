#include "stm32f10x.h"                  // Device header

void Buzzer_Init(void)
{
    // 1. 开启 GPIOB 和 AFIO 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 2. 关闭 JTAG，保留 SWD (释放 PB4)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // 3. 配置 PB4 为推挽输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 🚨 核心修复：低电平触发的模块，初始化时必须给【高电平(1)】才能让它闭嘴！
    GPIO_SetBits(GPIOB, GPIO_Pin_4); 
}

// 蜂鸣器响 (给低电平 0)
void Buzzer_ON(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_4); 
}

// 蜂鸣器不响 (给高电平 1)
void Buzzer_OFF(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
}