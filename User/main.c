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

// 定义距离阈值 (单位：厘米 cm)
#define DIST_SAFE  40  // 安全距离，全速前进
#define DIST_WARN  20  // 警戒距离，停车摇头侦查

// 定义状态机状态
#define STATE_PATROL 0 // 巡逻避障状态
#define STATE_FIRE   1 // 灭火状态

uint8_t Current_State = STATE_PATROL;

// ==========================================
// 🚀 核心保留：电机平滑加减速控制
// ==========================================
int16_t Cur_Speed_L = 0; // 记录左轮当前真实速度
int16_t Cur_Speed_R = 0; // 记录右轮当前真实速度

void Motor_SmoothSpeed(int16_t target_L, int16_t target_R)
{
    int16_t step = 80; 
    
    while(Cur_Speed_L != target_L || Cur_Speed_R != target_R)
    {
        if(Cur_Speed_L < target_L) {
            Cur_Speed_L += step;
            if(Cur_Speed_L > target_L) Cur_Speed_L = target_L; 
        } else if(Cur_Speed_L > target_L) {
            Cur_Speed_L -= step;
            if(Cur_Speed_L < target_L) Cur_Speed_L = target_L; 
        }
        
        if(Cur_Speed_R < target_R) {
            Cur_Speed_R += step;
            if(Cur_Speed_R > target_R) Cur_Speed_R = target_R;
        } else if(Cur_Speed_R > target_R) {
            Cur_Speed_R -= step;
            if(Cur_Speed_R < target_R) Cur_Speed_R = target_R;
        }
        
        Motor_SetSpeed(Cur_Speed_L, Cur_Speed_R);
        Delay_ms(5); 
    }
}
// ==========================================

int main(void)
{
    // 1. 硬件初始化 (全部上阵)
    OLED_Init();
    Motor_Init();
    LED_Init();
    HCSR04_Init();
    PWM_SG90_Init();
    Flame_Init();
    Pump_Init();
    Buzzer_Init();
    
    // 2. 初始状态设定
    Servo_SetAngle(90);
    Buzzer_OFF();
    Pump_Close();
    
    // 3. 上电延时启动
    OLED_ShowString(1, 1, "System Ready...");
    OLED_ShowString(4, 1, "Starting in 3s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 2s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 1s");
    Delay_ms(1000);
    OLED_Clear();
    
    uint16_t dist_front = 0;
    uint16_t dist_left = 0;
    uint16_t dist_right = 0;

    while (1)
    {
        // --------------------------------------------------
        // 【最高优先级】全局感知：火灾检测
        // --------------------------------------------------
        uint8_t fire_pos = Flame_GetPosition();
        
        if (fire_pos != 0) 
        {
            Current_State = STATE_FIRE;   // 发现火情，强制进入灭火状态
        } 
        else 
        {
            Current_State = STATE_PATROL; // 无火，保持或恢复巡逻状态
        }

        // --------------------------------------------------
        // 状态分支执行
        // --------------------------------------------------
        if (Current_State == STATE_PATROL)
        {
            // --- 巡逻状态的硬件表现 ---
            Buzzer_OFF(); // 关闭蜂鸣器
            Pump_Close(); // 确保水泵关闭
            OLED_ShowString(1, 1, "Mode: Patrol   ");
            
            // --- 纯超声波避障核心逻辑 (完全无改动) ---
            Servo_SetAngle(90); 
            Delay_ms(20);       
            dist_front = HCSR04_GetDistance();
            OLED_ShowString(2, 1, "Dist:     cm");
            OLED_ShowNum(2, 7, dist_front, 3); 
            
            if (dist_front > DIST_SAFE) 
            {
                Motor_SmoothSpeed(500, 500); 
                LED1_ON();   
                LED2_OFF();  
            }
            else if (dist_front > DIST_WARN && dist_front <= DIST_SAFE) 
            {
                Motor_SmoothSpeed(300, 300); 
                LED1_ON();
                LED2_OFF();
            }
            else 
            {
                Motor_SmoothSpeed(0, 0); 
                LED1_OFF();
                LED2_ON();     
                Delay_ms(200); 
                
                Servo_SetAngle(20);
                Delay_ms(400); 
                dist_right = HCSR04_GetDistance();
                
                Servo_SetAngle(160);
                Delay_ms(400); 
                dist_left = HCSR04_GetDistance();
                
                Servo_SetAngle(90);
                Delay_ms(400);
                
                if (dist_right > dist_left) 
                {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(200);                 
                    Motor_SmoothSpeed(500, -500);  
                    Delay_ms(350);                 
                }
                else 
            {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(200);
                    Motor_SmoothSpeed(-500, 500);  
                    Delay_ms(350);                 
                }
            }
        }
        else if (Current_State == STATE_FIRE)
        {
            // --- 灭火状态的硬件表现 ---
            LED1_OFF();  // 绿灯灭
            LED2_ON();   // 红灯亮 (警告)
            Buzzer_ON(); // 蜂鸣器响
            OLED_ShowString(1, 1, "Mode: FIRE!    ");
            OLED_ShowString(2, 1, "Pos:  Exting...");
            OLED_ShowNum(2, 6, fire_pos, 1);
            
            // --- 追踪火源与定点喷水逻辑 ---
            if (fire_pos == 1 || fire_pos == 2) 
            {
                // 火在左侧，车体平滑左转寻找正中心
                Motor_SmoothSpeed(-400, 400); 
                Pump_Close(); // 没对准，先别喷水
            }
            else if (fire_pos == 4 || fire_pos == 5) 
            {
                // 火在右侧，车体平滑右转
                Motor_SmoothSpeed(400, -400); 
                Pump_Close();
            }
            else if (fire_pos == 3) 
            {
                // 🎯 正对火源！
                Motor_SmoothSpeed(0, 0); // 停车
                Pump_Open();             // 启动水泵/继电器吸合！
            }
            
            Delay_ms(50); // 灭火微调的采样延时
        }
        
        // 大循环节奏控制
        Delay_ms(20); 
    }
}