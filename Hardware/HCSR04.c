#include "stm32f10x.h"                  // Device header
#include "Delay.h"                      // 必须引入江科大的延时库

/**
  * @brief  超声波模块初始化
  * @param  无
  * @retval 无
  */
void HCSR04_Init(void)
{
    // 1. 开启 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 2. 配置 PB8 (Trig 触发引脚) - 推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 配置 PB9 (Echo 接收引脚) - 下拉输入 (没信号时默认低电平)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化时将 Trig 拉低，保持安静
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

/**
  * @brief  获取超声波测量的距离
  * @param  无
  * @retval 距离值，单位：厘米 (cm)。如果超时返回 999
  */
uint16_t HCSR04_GetDistance(void)
{
    uint32_t Time = 0;
    uint32_t Timeout = 0;
    float Distance = 0;
    
    // 1. 给 Trig 发送一个大于 10us 的高电平脉冲，触发测距
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_us(15); 
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    
    // 2. 等待 Echo 引脚变为高电平 (超声波刚发出去的瞬间)
    Timeout = 0;
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_RESET)
    {
        Timeout++;
        Delay_us(1);
        if(Timeout > 100000) return 999; // 超时防死机保护 (大约100ms没反应就退出)
    }
    
    // 3. 核心：Echo变高了，开始计时，直到 Echo 变低
    Time = 0;
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_SET)
    {
        Time++;         // 每次循环加1
        Delay_us(1);    // 延时 1 微秒
        if(Time > 100000) return 999; // 超出最大测量范围，退出
    }
    
    // 4. 计算距离
    // 声音速度是 340m/s，即 0.034cm/us
    // 距离 = (时间 * 0.034) / 2 （因为是往返时间，所以要除以2）
    // 即：距离 = 时间 * 0.017
    
    // 注意：因为加入了 while 语句本身执行的微小时间差，
    // 实际补偿系数根据经验改为 0.015 到 0.018 左右最准，这里我们用 0.017
    Distance = Time * 0.017; 
    
    return (uint16_t)Distance;
}