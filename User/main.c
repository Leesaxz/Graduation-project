#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "LED.h"
#include "HCSR04.h"
#include "PWM_SG90.h"
#include "Flame.h"
#include "Pump.h"
#include "Buzzer.h"
#include "DS18B20.h"
#include "IRSensor.h" 
#include "ESP8266.h"   
#include <stdio.h>     

#define DIST_SAFE  35  
#define DIST_WARN  25  

#define STATE_PATROL 0 
#define STATE_FIRE   1 

uint8_t Current_State = STATE_PATROL;

// ==========================================
// 高级滤波函数：连续3次取最远值，防超声波杂波
// ==========================================
uint16_t Get_Filtered_Distance(void) {
    uint16_t d1 = HCSR04_GetDistance(); if(d1 == 0) d1 = 999; Delay_ms(5);
    uint16_t d2 = HCSR04_GetDistance(); if(d2 == 0) d2 = 999; Delay_ms(5);
    uint16_t d3 = HCSR04_GetDistance(); if(d3 == 0) d3 = 999;
    
    uint16_t max = d1;
    if (d2 > max) max = d2;
    if (d3 > max) max = d3;
    return max;
}

// 电机平滑控制
int16_t Cur_Speed_L = 0; 
int16_t Cur_Speed_R = 0; 
void Motor_SmoothSpeed(int16_t target_L, int16_t target_R) {
    int16_t step = 80; 
    while(Cur_Speed_L != target_L || Cur_Speed_R != target_R) {
        if(Cur_Speed_L < target_L) { Cur_Speed_L += step; if(Cur_Speed_L > target_L) Cur_Speed_L = target_L; } 
        else if(Cur_Speed_L > target_L) { Cur_Speed_L -= step; if(Cur_Speed_L < target_L) Cur_Speed_L = target_L; }
        
        if(Cur_Speed_R < target_R) { Cur_Speed_R += step; if(Cur_Speed_R > target_R) Cur_Speed_R = target_R; } 
        else if(Cur_Speed_R > target_R) { Cur_Speed_R -= step; if(Cur_Speed_R < target_R) Cur_Speed_R = target_R; }
        
        Motor_SetSpeed(Cur_Speed_L, Cur_Speed_R);
        Delay_ms(5); 
    }
}

int main(void)
{
    OLED_Init(); Motor_Init(); LED_Init(); HCSR04_Init(); PWM_SG90_Init();
    Flame_Init(); Pump_Init(); Buzzer_Init(); DS18B20_Init(); IRSensor_Init(); 
    ESP8266_Init(); // PA9(TX), PA10(RX)
    
    Servo_SetAngle(90); Buzzer_OFF(); Pump_Close();
    
    OLED_ShowString(1, 1, "System Booting..");
    Delay_ms(1500);
    OLED_Clear();
    OLED_ShowString(3, 1, "Temp: --.- C");
    
    uint16_t dist_front = 0, dist_left = 0, dist_right = 0;
    uint8_t  ir_left = 0, ir_right = 0; 
    uint16_t fire_missing_cnt = 0; 
    uint8_t  last_fire_pos = 0;    
    uint16_t temp_timer = 0; 
    uint8_t  temp_state = 0; 
    
    float Global_Temp = -100.0;  
    uint16_t mqtt_timer = 0;      
    uint8_t stuck_cnt = 0; // 卡死挣脱计数器

    while (1)
    {
        // --------------------------------------------------
        // 温度读取 (防0.00，平滑数字滤波)
        // --------------------------------------------------
        if (temp_state == 0) {
            DS18B20_ConvertT(); 
            temp_state = 1;     
            temp_timer = 0;     
        } else if (temp_timer >= 800) {
            __disable_irq(); 
            float temp = DS18B20_ReadT(); 
            __enable_irq();  
            
            if (temp > 1.0 && temp < 70.0 && temp != 85.0) {
                if (Global_Temp == -100.0) {
                    Global_Temp = temp; 
                } else {
                    if (temp - Global_Temp < 15.0 && Global_Temp - temp < 15.0) {
                        Global_Temp = (Global_Temp * 0.7) + (temp * 0.3); 
                    }
                }
                
                int temp_int = (int)Global_Temp;                  
                int temp_frac = (int)(Global_Temp * 10) % 10;     
                
                OLED_ShowString(3, 1, "Temp:           "); 
                OLED_ShowString(3, 1, "Temp: ");
                OLED_ShowNum(3, 7, temp_int, 2);
                OLED_ShowChar(3, 9, '.');
                OLED_ShowNum(3, 10, temp_frac, 1);
                OLED_ShowString(3, 11, " C"); 
            }
            temp_state = 0; 
        }

        // --------------------------------------------------
        // 🌟 极速 MQTT 发送 (频率：每秒1次，发送4个核心数据)
        // --------------------------------------------------
        if (mqtt_timer >= 1000 && Global_Temp != -100.0) 
        {
            char simple_msg[40];
            int t_int = (int)Global_Temp;
            int t_frac = (int)(Global_Temp * 10) % 10;
            
            // 格式: 温度,状态,距离,火源方位 (例如: 26.5,0,45,0)
            sprintf(simple_msg, "%d.%d,%d,%d,%d\n", t_int, t_frac, Current_State, dist_front, last_fire_pos);
            ESP8266_SendString(simple_msg); 
            
            mqtt_timer = 0; 
        }

        // --------------------------------------------------
        // 火灾检测
        // --------------------------------------------------
        uint8_t fire_pos = Flame_GetPosition(); 
        if (fire_pos != 0) { Current_State = STATE_FIRE; fire_missing_cnt = 0; last_fire_pos = fire_pos; } 
        else {
            if (Current_State == STATE_FIRE) { fire_missing_cnt++; if (fire_missing_cnt > 50) { Current_State = STATE_PATROL; fire_missing_cnt = 0; last_fire_pos = 0; } } 
            else { Current_State = STATE_PATROL; last_fire_pos = 0; }
        }

        // --------------------------------------------------
        // 走位调度与防卡死
        // --------------------------------------------------
        if (Current_State == STATE_PATROL) 
        {
            Buzzer_OFF(); Pump_Close(); LED1_ON(); LED2_OFF();   
            OLED_ShowString(1, 1, "Mode: Patrol   ");
            
            Servo_SetAngle(90); Delay_ms(20);       
            dist_front = Get_Filtered_Distance();
            ir_left = IRSensor_GetLeft(); ir_right = IRSensor_GetRight(); 
            
            OLED_ShowString(2, 1, "D:     L:  R: ");
            if (dist_front == 999) OLED_ShowString(2, 3, "MAX");
            else OLED_ShowNum(2, 3, dist_front, 3); 
            OLED_ShowNum(2, 10, ir_left, 1);
            OLED_ShowNum(2, 14, ir_right, 1);
            
            // 正常巡逻
            if (dist_front > DIST_SAFE && ir_left == 0 && ir_right == 0) { 
                Motor_SmoothSpeed(500, 500); 
                stuck_cnt = 0; 
            } 
            else if (dist_front > DIST_WARN && dist_front <= DIST_SAFE && ir_left == 0 && ir_right == 0) { 
                Motor_SmoothSpeed(300, 300); 
                stuck_cnt = 0;
            } 
            // 触发避障
            else {
                Motor_SmoothSpeed(0, 0); Delay_ms(200); 
                
                stuck_cnt++; 
                
                // 暴躁老哥机制：防止左右横跳
                if (stuck_cnt >= 3) {
                    OLED_ShowString(1, 1, "Mode: ESCAPE!  ");
                    Motor_SmoothSpeed(-400, -400); Delay_ms(600); 
                    Motor_SmoothSpeed(500, -500); Delay_ms(800);  // 猛转180度掉头
                    stuck_cnt = 0; 
                    temp_timer += 1600; mqtt_timer += 1600; 
                } 
                // 微操避障
                else {
                    Servo_SetAngle(45); Delay_ms(300); dist_right = Get_Filtered_Distance();
                    Servo_SetAngle(135); Delay_ms(300); dist_left = Get_Filtered_Distance();
                    Servo_SetAngle(90); Delay_ms(300);
                    
                    if (ir_right == 1 || dist_right <= dist_left) { 
                        Motor_SmoothSpeed(-400, -400); Delay_ms(400); 
                        Motor_SmoothSpeed(-450, 450); Delay_ms(250); 
                    } 
                    else if (ir_left == 1 || dist_right > dist_left) { 
                        Motor_SmoothSpeed(-400, -400); Delay_ms(400); 
                        Motor_SmoothSpeed(450, -450); Delay_ms(250);  
                    } 
                    temp_timer += 1750; mqtt_timer += 1750; 
                }
            }
        }
        else if (Current_State == STATE_FIRE) 
        {
            LED1_OFF(); LED2_ON(); Buzzer_ON(); 
            OLED_ShowString(1, 1, "Mode: FIRE!    ");
            OLED_ShowString(2, 1, "Extinguishing."); 
            
            if (fire_pos == 1) { Motor_SmoothSpeed(-450, 450); Pump_Close(); } 
            else if (fire_pos == 2) { Motor_SmoothSpeed(-200, 200); Pump_Close(); } 
            else if (fire_pos == 5) { Motor_SmoothSpeed(450, -450); Pump_Close(); } 
            else if (fire_pos == 4) { Motor_SmoothSpeed(200, -200); Pump_Close(); } 
            else if (fire_pos == 3) { Motor_SmoothSpeed(0, 0); Pump_Open(); } 
            else if (fire_pos == 0) { Motor_SmoothSpeed(0, 0); }
            
            Delay_ms(50); temp_timer += 50; mqtt_timer += 50; 
        }
        Delay_ms(20); temp_timer += 20; mqtt_timer += 20; 
    }
}