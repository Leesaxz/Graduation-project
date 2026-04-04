#include "stm32f10x.h"                  
#include "Delay.h"                      

// 初始化保持不变
void HCSR04_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PB8 (Trig 触发)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB9 (Echo 接收)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

/**
  * @brief  获取一次原始距离 (内部函数)
  */
uint16_t HCSR04_GetRawDistance(void)
{
    uint32_t Time = 0;
    uint32_t Timeout = 0;
    
    // 1. 发送触发脉冲
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_us(15); 
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    
    // 2. 等待高电平出现
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_RESET)
    {
        Timeout++;
        Delay_us(1);
        if(Timeout > 100000) return 999; 
    }
    
    // 3. 记录高电平持续时间 (移除 Delay_us(1)，让循环极其紧凑！)
    Time = 0;
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_SET)
    {
        Time++;         
        // 🚨 删掉了 Delay_us(1)，只做纯加法！这样循环执行时间固定，方便后续校准！
        if(Time > 100000) return 999; 
    }
    
    // 4. 重点校准系数！
    // 因为去掉了 Delay_us，这里的 Time 并不是微秒，而是循环次数。
    // 经过测试，在 STM32 72MHz 且没有 Delay_us 时，大概循环 1 次是 0.1~0.2us。
    // 👉 初始建议值设为 0.0035，具体要拿尺子量！
    float Distance = Time * 0.0035; 
    
    if(Distance > 999) return 999;
    return (uint16_t)Distance;
}

/**
  * @brief  🏆 对外提供的接口：带中值滤波的超声波测距
  * @retval 极其稳定的距离值 (cm)
  */
uint16_t HCSR04_GetDistance(void)
{
    uint16_t dis[5] = {0}; // 采 5 个样本
    uint16_t temp = 0;
    
    // 1. 连续测量 5 次
    for(uint8_t i = 0; i < 5; i++)
    {
        dis[i] = HCSR04_GetRawDistance();
        Delay_ms(5); // 🚨 极度重要！每次测量之间必须延时，等待上一次超声波余音消失！
    }
    
    // 2. 冒泡排序 (把 5 个数据从小到大排好队)
    for(uint8_t i = 0; i < 4; i++)
    {
        for(uint8_t j = 0; j < 4 - i; j++)
        {
            if(dis[j] > dis[j+1])
            {
                temp = dis[j];
                dis[j] = dis[j+1];
                dis[j+1] = temp;
            }
        }
    }
    
    // 3. 抛弃两个最大值和两个最小值，取最中间那个最靠谱的值！
    return dis[2];
}