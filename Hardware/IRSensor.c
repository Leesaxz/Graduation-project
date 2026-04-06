#include "stm32f10x.h"                  // Device header
#include "IRSensor.h"

void IRSensor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    // 配置为上拉输入，探头悬空或遇到黑线时默认高电平
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // PB12(左), PB13(中), PB14(右)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 🌟 终极修正：
// 探头亮灯（白底/安全） -> 引脚为低电平(Bit_RESET) -> 返回 0 (告诉主程序：安全)
// 探头灭灯（黑线/悬空） -> 引脚为高电平(Bit_SET)   -> 返回 1 (告诉主程序：危险！)

uint8_t IRSensor_GetLeft(void)  
{ 
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_SET) return 1; // 灭灯报警
    return 0; // 亮灯安全
}

uint8_t IRSensor_GetFront(void) 
{ 
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_SET) return 1; 
    return 0; 
}

uint8_t IRSensor_GetRight(void) 
{ 
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == Bit_SET) return 1; 
    return 0; 
}