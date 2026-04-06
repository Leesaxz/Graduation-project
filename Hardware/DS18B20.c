#include "stm32f10x.h"                  // Device header
#include "DS18B20.h"
#include "Delay.h"

// ==============================================================
// 🌟 动态引脚模式切换 (终极防线：不依赖外部4.7K上拉电阻)
// ==============================================================
void DS_Mode_Out(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出：单片机强力拉高拉低
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void DS_Mode_In(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入：借用单片机内部电阻保持高电平
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ==============================================================
// 基础驱动函数
// ==============================================================
void DS18B20_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    DS_Mode_Out();
    GPIO_SetBits(GPIOB, GPIO_Pin_5); // 初始释放总线，保持高电平
}

uint8_t DS18B20_Reset(void)
{
    uint8_t ack = 0;
    
    DS_Mode_Out();
    GPIO_ResetBits(GPIOB, GPIO_Pin_5); 
    Delay_us(500);      // 强力拉低 500us
    
    GPIO_SetBits(GPIOB, GPIO_Pin_5); 
    Delay_us(30);       // 释放总线，等待 30us 
    
    DS_Mode_In();       // 🌟 立刻切换为输入模式，听传感器回应
    ack = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5); // 0代表传感器在线
    
    Delay_us(400);      // 等待脉冲结束
    return ack;
}

void DS18B20_WriteByte(uint8_t Byte)
{
    uint8_t i;
    DS_Mode_Out();      // 🌟 写数据时，全程使用强力推挽模式
    for (i = 0; i < 8; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        Delay_us(2); 
        
        if (Byte & 0x01) {
            GPIO_SetBits(GPIOB, GPIO_Pin_5); // 强制输出1
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_5); // 强制输出0
        }
        
        Delay_us(60);
        GPIO_SetBits(GPIOB, GPIO_Pin_5); // 恢复高电平
        Delay_us(2); 
        Byte >>= 1; 
    }
}

uint8_t DS18B20_ReadByte(void)
{
    uint8_t i, Byte = 0;
    for (i = 0; i < 8; i++)
    {
        DS_Mode_Out();
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        Delay_us(2); 
        
        GPIO_SetBits(GPIOB, GPIO_Pin_5); 
        DS_Mode_In(); // 🌟 释放后立刻切为输入模式
        
        Delay_us(10); // 🌟 修正：缩短到 10us 就去采样！提前抢读，绝不错过 15us 窗口！
        
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) {
            Byte |= (0x01 << i); 
        }
        
        Delay_us(50); // 补足剩余周期
    }
    return Byte;
}

// ==============================================================
// 业务应用函数
// ==============================================================
void DS18B20_ConvertT(void)
{
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0x44); // Convert T
}

float DS18B20_ReadT(void)
{
    uint8_t LSB, MSB;
    int16_t temp_raw;
    
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC); 
    DS18B20_WriteByte(0xBE); // Read Scratchpad
    
    LSB = DS18B20_ReadByte(); 
    MSB = DS18B20_ReadByte(); 
    
    temp_raw = (MSB << 8) | LSB;
    return temp_raw * 0.0625;
}