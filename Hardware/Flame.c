#include "stm32f10x.h"                  // Device header
#include "Flame.h"

/**
  * @brief  五路火焰传感器初始化
  * @param  无
  * @retval 无
  */
void Flame_Init(void)
{
    // 1. 开启 GPIOA 和 GPIOB 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 2. 定义 GPIO 初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 江科大精髓：对于外部传感器输入，通常配置为上拉输入(IPU)，保证默认状态为高电平，抗干扰能力更强
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 3. 配置 PA2, PA3, PA11, PA12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 4. 配置 PB15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  获取火源位置
  * @param  无
  * @retval 返回火源所在的方向：0(无火), 1(极左), 2(偏左), 3(正前), 4(偏右), 5(极右)
  */
// 在 Flame_GetPosition 函数里，把所有的 Bit_RESET 改成 Bit_SET
uint8_t Flame_GetPosition(void)
{
    // 改为检测高电平 (Bit_SET)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_SET) return 1;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET) return 2;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_SET) return 3;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET) return 4;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == Bit_SET) return 5;
    
    return 0; 
}