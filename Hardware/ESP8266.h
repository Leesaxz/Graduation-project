#ifndef __ESP8266_H
#define __ESP8266_H
#include "stm32f10x.h"

void ESP8266_Init(void);
void ESP8266_SendString(char *str);

#endif