#include "stm32f10x.h"                  // Device header

// 定义全局变量，供 main.c 跨文件调用
char WIFI_Command = 0; 

void ESP8266_Init(void)
{
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    // 2. 配置引脚 (修正了 RX 引脚模式)
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // TX (PA9) - 复用推挽输出 (用来发送)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX (PA10) - 浮空输入 (用来接收)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    // 速度对输入模式无效，可省略
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1 参数
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    
    // 4. 开启串口接收中断 (核心魔法)
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 5. 配置 NVIC 中断优先级控制器
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 放到 main 里或者这里都行，分组2
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // 响应优先级
    NVIC_Init(&NVIC_InitStructure);
    
    // 6. 使能串口
    USART_Cmd(USART1, ENABLE);
}

void ESP8266_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        // 发送一个字节
        USART_SendData(USART1, String[i]);
        // 等待发送数据寄存器清空
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

// ==========================================
// 🚨 串口1接收中断服务函数 🚨
// 只要 ESP8266 发送数据给 STM32，就会瞬间触发这个函数！
// ==========================================
void USART1_IRQHandler(void)
{
    // 判断是不是“接收非空”中断
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // 读取接收到的那一个字节
        uint8_t RxData = USART_ReceiveData(USART1);
        
        // 粗略过滤：如果收到的是指定的遥控字符，就存进全局变量里
        if(RxData == 'F' || RxData == 'B' || RxData == 'L' || RxData == 'R' || RxData == 'S')
        {
            WIFI_Command = RxData;
        }
        
        // 清除中断标志位，等待下一次接收
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}