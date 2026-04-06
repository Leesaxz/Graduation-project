#include "stm32f10x.h"                  // Device header
#include "DS18B20.h"
#include "Delay.h"

// 宏定义底层引脚操作，方便修改和阅读 (原理图接在 PB5)
#define DS_OUT_0()  GPIO_ResetBits(GPIOB, GPIO_Pin_5)  // 拉低总线
#define DS_OUT_1()  GPIO_SetBits(GPIOB, GPIO_Pin_5)    // 释放总线 (由4.7K电阻拉高)
#define DS_IN()     GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) // 读取总线电平

/**
  * @brief  DS18B20 初始化 (配置 PB5 为开漏输出)
  */
void DS18B20_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    DS_OUT_1(); // 初始状态释放总线，保持高电平
}

/**
  * @brief  复位 DS18B20 并检测存在脉冲
  * @retval 0: 成功检测到传感器; 1: 传感器未响应
  */
uint8_t DS18B20_Reset(void)
{
    uint8_t ack = 0;
    
    DS_OUT_0();         // 主机拉低总线
    Delay_us(500);      // 保持 480~960us，发送复位脉冲
    DS_OUT_1();         // 主机释放总线
    Delay_us(70);       // 等待 15~60us 后，DS18B20 会拉低总线发出存在脉冲
    
    ack = DS_IN();      // 读取传感器的回应 (如果是 0，说明传感器在)
    Delay_us(400);      // 等待存在脉冲结束
    
    return ack;
}

/**
  * @brief  向 DS18B20 写入一个字节
  */
void DS18B20_WriteByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        DS_OUT_0();     // 拉低总线，产生写时间隙
        Delay_us(2);    // 保持大于 1us
        
        if (Byte & 0x01) // 从最低位开始发
            DS_OUT_1(); // 如果是 1，释放总线
        else
            DS_OUT_0(); // 如果是 0，继续拉低总线
        
        Delay_us(60);   // 整个写周期需要至少 60us
        DS_OUT_1();     // 释放总线，准备下一次发送
        Delay_us(2);    // 恢复时间
        
        Byte >>= 1;     // 数据右移
    }
}

/**
  * @brief  从 DS18B20 读取一个字节
  */
uint8_t DS18B20_ReadByte(void)
{
    uint8_t i, Byte = 0;
    for (i = 0; i < 8; i++)
    {
        DS_OUT_0();     // 拉低总线，产生读时间隙
        Delay_us(2);    // 保持大于 1us
        DS_OUT_1();     // 释放总线，让传感器控制
        Delay_us(12);   // 延时一会儿去采样 (要在 15us 内完成采样)
        
        if (DS_IN())    // 采样总线电平
            Byte |= (0x01 << i); // 如果读到 1，存入变量
        
        Delay_us(50);   // 整个读周期需要至少 60us
    }
    return Byte;
}

// ================= 将原来的 DS18B20_GetTemp 替换为以下两个函数 =================

/**
  * @brief  第一步：发送温度转换命令 (不等待)
  */
void DS18B20_ConvertT(void)
{
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0x44); // Convert T
}

/**
  * @brief  第二步：读取已经转换好的温度
  */
float DS18B20_ReadT(void)
{
    uint8_t LSB, MSB;
    int16_t temp_raw;
    float temp_real;
    
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0xBE); // Read Scratchpad
    
    LSB = DS18B20_ReadByte(); // 低位
    MSB = DS18B20_ReadByte(); // 高位
    
    temp_raw = (MSB << 8) | LSB;
    temp_real = temp_raw * 0.0625;
    
    return temp_real;
}
