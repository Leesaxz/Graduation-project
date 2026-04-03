#include "stm32f10x.h"

/**
  * @brief  微秒级延时 (纯CPU循环，不占用任何定时器)
  */
void Delay_us(uint32_t us)
{
    // 72MHz主频下，经过粗略计算的软件延时常数
    uint32_t delay = (SystemCoreClock / 8000000) * us;
    while(delay--);
}

/**
  * @brief  毫秒级延时
  */
void Delay_ms(uint32_t ms)
{
    while(ms--)
    {
        Delay_us(1000);
    }
}

/**
  * @brief  秒级延时
  */
void Delay_s(uint32_t s)
{
    while(s--)
    {
        Delay_ms(1000);
    }
}