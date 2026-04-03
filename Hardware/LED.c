#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
    // 1. 开启 GPIOA, GPIOB 和 AFIO 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 2. 核心魔法：关闭 JTAG，保留 SWD
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // 3. 配置 PA15 (LED1) 和 PB3 (LED2) 为推挽输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 默认关闭两个 LED (假设高电平亮，初始拉低)
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3);
}

// LED1 (绿灯-PA15) 控制
void LED1_ON(void)  { GPIO_SetBits(GPIOA, GPIO_Pin_15); }
void LED1_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_15); }

// LED2 (红灯-PB3) 控制
void LED2_ON(void)  { GPIO_SetBits(GPIOB, GPIO_Pin_3); }
void LED2_OFF(void) { GPIO_ResetBits(GPIOB, GPIO_Pin_3); }