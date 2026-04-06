#include "stm32f10x.h"                  // Device header
#include "Flame.h"
#include "Delay.h"

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
  * @brief  获取火源位置 (100ms 宽频物理滤光算法)
  * @retval 返回方向：0(无火/阳光), 1(极左), 2(偏左), 3(正前), 4(偏右), 5(极右)
  */
uint8_t Flame_GetPosition(void)
{
    uint8_t i;
    // 记录在 100ms 内，每个探头感受到高电平的次数
    uint8_t d1_cnt = 0, d2_cnt = 0, d3_cnt = 0, d4_cnt = 0, d5_cnt = 0;

    // 1. 扩大采样窗口：连续读 20 次，每次间隔 5ms (总耗时 100ms)
    // 这个时间刚好覆盖真实火苗的一个完整跳动周期，同时不会让小车避障变得太迟钝
    for(i = 0; i < 20; i++)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_SET) d1_cnt++;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET) d2_cnt++;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_SET) d3_cnt++;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET) d4_cnt++;
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == Bit_SET) d5_cnt++;
        Delay_ms(5);
    }

    // 2. 🌟 核心判断逻辑 🌟
    // 【阳光/窗户光】：光线是持续的，哪怕小车在动，在 100ms 内它也会稳定输出高电平，计数会达到 19 或 20 满载。
    // 【真实火焰】：火苗在空气中会物理闪烁，100ms 内绝对不可能一直保持最高亮度，计数通常在 3 ~ 18 之间跳动。

    // 优先判断正前方的火源 (排除满载的假信号)
    if(d3_cnt >= 3 && d3_cnt <= 18) return 3;
    if(d2_cnt >= 3 && d2_cnt <= 18) return 4;
    if(d4_cnt >= 3 && d4_cnt <= 18) return 2;
    if(d1_cnt >= 3 && d1_cnt <= 18) return 5;
    if(d5_cnt >= 3 && d5_cnt <= 18) return 1;

    // 3. 极端情况兜底：如果小车已经贴到了火苗脸上，火光太强导致满载怎么办？
    // 真实火苗是点光源，通常只有 1 个（最多 2 个）探头会满载。
    // 如果是窗户光，通常是一大片，会有 3 个甚至全部探头满载。
    uint8_t saturated_sensors = 0;
    if(d1_cnt >= 19) saturated_sensors++;
    if(d2_cnt >= 19) saturated_sensors++;
    if(d3_cnt >= 19) saturated_sensors++;
    if(d4_cnt >= 19) saturated_sensors++;
    if(d5_cnt >= 19) saturated_sensors++;

    if(saturated_sensors == 1 || saturated_sensors == 2)
    {
        // 只有局部极度高亮，确认是贴脸的真火！
        if (d3_cnt >= 19) return 3;
        if (d2_cnt >= 19) return 4;
        if (d4_cnt >= 19) return 2;
        if (d1_cnt >= 19) return 5;
        if (d5_cnt >= 19) return 1;
    }

    // 如果满载的探头大于 2 个，绝对是窗户光/太阳光，直接无视！
    return 0; 
}

