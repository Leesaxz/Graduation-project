#include "stm32f10x.h"                  // Device header
#include "IRSensor.h"

/**
  * @brief  红外避障传感器初始化
  * @param  无
  * @retval 无
  */
void IRSensor_Init(void)
{
    // 1. 开启 GPIOB 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 2. 定义 GPIO 初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 配置为上拉输入 (默认保持高电平，抗干扰)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 3. 配置 PB12(左), PB13(前), PB14(右)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  获取左侧红外传感器状态
  * @retval 1: 有障碍物, 0: 无障碍物
  */
uint8_t IRSensor_GetLeft(void)
{
    // 假设遇到障碍物时，D0输出低电平(Bit_RESET)
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_RESET) 
    {
        return 1; // 有障碍
    }
    return 0; // 无障碍
}

/**
  * @brief  获取前方红外传感器状态
  * @retval 1: 有障碍物, 0: 无障碍物
  */
uint8_t IRSensor_GetFront(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_RESET) 
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  获取右侧红外传感器状态
  * @retval 1: 有障碍物, 0: 无障碍物
  */
uint8_t IRSensor_GetRight(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == Bit_RESET) 
    {
        return 1;
    }
    return 0;
}
