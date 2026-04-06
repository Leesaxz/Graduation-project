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

// ==========================================
// 🚨 避障距离阈值优化
// ==========================================
#define DIST_SAFE  45  
#define DIST_WARN  30  

#define STATE_PATROL 0 
#define STATE_FIRE   1 

uint8_t Current_State = STATE_PATROL;

// ==========================================
// 🚀 电机平滑加减速控制
// ==========================================
int16_t Cur_Speed_L = 0; 
int16_t Cur_Speed_R = 0; 

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

int main(void)
{
    OLED_Init();
    Motor_Init();
    LED_Init();
    HCSR04_Init();
    PWM_SG90_Init();
    Flame_Init();
    Pump_Init();
    Buzzer_Init();
    DS18B20_Init(); 
    IRSensor_Init(); 
    
    Servo_SetAngle(90);
    Buzzer_OFF();
    Pump_Close();
    
    OLED_ShowString(1, 1, "System Ready...");
    OLED_ShowString(4, 1, "Starting in 3s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 2s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 1s");
    Delay_ms(1000);
    OLED_Clear();
    
    OLED_ShowString(3, 1, "Temp: --.- C");
    
    uint16_t dist_front = 0, dist_left = 0, dist_right = 0;
    uint8_t  ir_left = 0;  
    uint8_t  ir_right = 0; 
    
    uint16_t fire_missing_cnt = 0; 
    uint8_t  last_fire_pos = 0;    
    
    uint16_t temp_timer = 0; 
    uint8_t  temp_state = 0; 

    while (1)
    {
        // --------------------------------------------------
        // 【最高优先级全局任务】异步测量火场温度
        // --------------------------------------------------
        // 不管是巡逻还是灭火，都雷打不动地执行测温！
        if (temp_state == 0) 
        {
            DS18B20_ConvertT(); 
            temp_state = 1;     
            temp_timer = 0;     
        } 
        else if (temp_timer >= 800) 
        {
            float temp = DS18B20_ReadT(); 
            
            // 顶着水泵的干扰强行滤噪更新：
            // 如果水泵造成了一次误码(如85.0)，这里直接无视，屏幕保持上一秒的高温
            // 直到下个 800ms 读出正确数据再刷新，保证监控不中断！
            if (temp > 5.0 && temp < 80.0 && temp != 85.0) 
            {
                int temp_int = (int)temp;                  
                int temp_frac = (int)(temp * 10) % 10;     
                
                OLED_ShowNum(3, 7, temp_int, 2);
                OLED_ShowChar(3, 9, '.');
                OLED_ShowNum(3, 10, temp_frac, 1);
                OLED_ShowString(3, 11, " C"); 
            }
            
            temp_state = 0; 
        }

        // --------------------------------------------------
        // 全局感知：火灾检测
        // --------------------------------------------------
        uint8_t fire_pos = Flame_GetPosition(); 
        
        if (fire_pos != 0) 
        {
            Current_State = STATE_FIRE;   
            fire_missing_cnt = 0;         
            last_fire_pos = fire_pos;     
        } 
        else 
        {
            if (Current_State == STATE_FIRE) 
            {
                fire_missing_cnt++; 
                if (fire_missing_cnt > 50) 
                {
                    Current_State = STATE_PATROL; 
                    fire_missing_cnt = 0;
                }
            } 
            else 
            {
                Current_State = STATE_PATROL; 
            }
        }

        // --------------------------------------------------
        // 状态分支执行
        // --------------------------------------------------
        if (Current_State == STATE_PATROL)
        {
            Buzzer_OFF(); 
            Pump_Close(); 
            LED1_ON();    
            LED2_OFF();   
            
            OLED_ShowString(1, 1, "Mode: Patrol   ");
            
            Servo_SetAngle(90); 
            Delay_ms(20);       
            dist_front = HCSR04_GetDistance();
            
            ir_left = IRSensor_GetLeft();   
            ir_right = IRSensor_GetRight(); 
            
            OLED_ShowString(2, 1, "Dist:     cm");
            OLED_ShowNum(2, 7, dist_front, 3); 
            
            if (dist_front > DIST_SAFE && ir_left == 0 && ir_right == 0) 
            {
                Motor_SmoothSpeed(500, 500); 
            }
            else if (dist_front > DIST_WARN && dist_front <= DIST_SAFE && ir_left == 0 && ir_right == 0) 
            {
                Motor_SmoothSpeed(300, 300); 
            }
            else 
            {
                Motor_SmoothSpeed(0, 0);    
                Delay_ms(200); 
                
                Servo_SetAngle(20);
                Delay_ms(400); 
                dist_right = HCSR04_GetDistance();
                
                Servo_SetAngle(160);
                Delay_ms(400); 
                dist_left = HCSR04_GetDistance();
                
                Servo_SetAngle(90);
                Delay_ms(400);
                
                if (ir_right == 1 && dist_right <= dist_left) {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400); 
                    Motor_SmoothSpeed(-550, 550);  
                    Delay_ms(350); 
                } else if (ir_left == 1 && dist_right >= dist_left) {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400);  
                    Motor_SmoothSpeed(550, -550);  
                    Delay_ms(350); 
                } else if (dist_right >= dist_left || ir_left == 1) {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400);               
                    Motor_SmoothSpeed(550, -550);  
                    Delay_ms(350);              
                } else if (dist_right < dist_left || ir_right == 1) {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400);  
                    Motor_SmoothSpeed(-550, 550);  
                    Delay_ms(350);               
                }
                
                // 补偿避障耗时
                temp_timer += 2150; 
            }
        }
        else if (Current_State == STATE_FIRE)
        {
            LED1_OFF();  
            LED2_ON();   
            Buzzer_ON(); 
            
            OLED_ShowString(1, 1, "Mode: FIRE!    ");
            OLED_ShowString(2, 1, "Pos:  Exting...");
            OLED_ShowNum(2, 6, last_fire_pos, 1); 
            
            if (fire_pos == 1) {
                Motor_SmoothSpeed(-500, 500); 
                Pump_Close(); 
            } else if (fire_pos == 2) {
                Motor_SmoothSpeed(-300, 300); 
                Pump_Close(); 
            } else if (fire_pos == 5) {
                Motor_SmoothSpeed(500, -500); 
                Pump_Close();
            } else if (fire_pos == 4) {
                Motor_SmoothSpeed(300, -300); 
                Pump_Close();
            } else if (fire_pos == 3) {
                Motor_SmoothSpeed(0, 0); 
                Pump_Open();             
            } else if (fire_pos == 0) {
                Motor_SmoothSpeed(0, 0); 
            }
            
            Delay_ms(50); 
            // 🌟 核心补偿：灭火状态微调耗时 50ms，必须加给温度计时器！
            temp_timer += 50; 
        }
        
        // 主循环心跳延时
        Delay_ms(20); 
        // 基础时间补偿
        temp_timer += 20; 
    }
}