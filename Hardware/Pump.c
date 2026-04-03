#include "stm32f10x.h"                  // Device header

void Pump_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC ,ENABLE);
	
	GPIO_InitTypeDef GPIO_initStructure;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_initStructure);
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void Pump_Open(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void Pump_Close(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}