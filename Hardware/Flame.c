#include "stm32f10x.h"                  // Device header
#include "Flame.h"

/**
  * @brief  五路火焰传感器初始化
  */
void Flame_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 下拉输入 (默认低电平，遇到火高电平)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  获取火源位置 (极速版，无软件延时)
  * @retval 0(无火), 1(极左), 2(偏左), 3(正前), 4(偏右), 5(极右)
  */
uint8_t Flame_GetPosition(void)
{
    // 物理防干扰做好了，直接读电平就行！(高电平Bit_SET代表有火)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_SET)  return 5; // D1 极右
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET)  return 4; // D2 偏右
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_SET) return 3; // D3 正前
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET) return 2; // D4 偏左
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == Bit_SET) return 1; // D5 极左
    
    return 0; // 无火
}