#include "stm32f10x.h"                  // Device header
#include "Flame.h"

void Flame_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  获取火源位置 (智能防震荡版)
  */
uint8_t Flame_GetPosition(void)
{
    // 获取当前每个探头的状态 (1为亮，0为灭)
    uint8_t d1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_SET;  // 极右
    uint8_t d2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET;  // 偏右
    uint8_t d3 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_SET; // 正前方
    uint8_t d4 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET; // 偏左
    uint8_t d5 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == Bit_SET; // 极左
    
    uint8_t count = d1 + d2 + d3 + d4 + d5; // 统计有几个探头亮了
    
    if (count == 0) return 0; // 没看到火
    
    // 🌟 绝招 1：扩大“瞄准死区 (Deadband)”
    // 只要正前方的探头(d3)亮了，或者左右两边把火夹在了中间，
    // 一律认为已经瞄准！直接返回 3 停车喷水，严禁再微调摇头！
    if (d3 == 1 || (d2 == 1 && d4 == 1)) 
    {
        return 3; 
    }
    
    // 🌟 绝招 2：多探头触发时的“加权重心算法”
    // 赋予每个方向一个坐标权重：极左10, 偏左20, 正前30, 偏右40, 极右50
    // 把亮起的探头权重加起来求平均值，就能找到真正的火源重心！
    uint16_t sum = d1 * 50 + d2 * 40 + d3 * 30 + d4 * 20 + d5 * 10;
    uint8_t center = sum / count; 
    
    if (center >= 45) return 5; // 重心在极右
    if (center >= 35) return 4; // 重心在偏右
    if (center >= 25) return 3; // 重心在正中 (兜底)
    if (center >= 15) return 2; // 重心在偏左
    return 1;                   // 重心在极左
}