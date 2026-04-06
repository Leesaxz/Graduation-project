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

// ==========================================
// 🚨 避障距离阈值优化 (适配宽车身)
// ==========================================
#define DIST_SAFE  45  // 安全距离，全速前进 
#define DIST_WARN  30  // 警戒距离，提前停车侦查

// ==========================================
// 🚨 状态机定义
// ==========================================
#define STATE_PATROL 0 // 巡逻避障状态
#define STATE_FIRE   1 // 灭火状态

uint8_t Current_State = STATE_PATROL;

// ==========================================
// 🚀 电机平滑加减速控制算法 (保护电源，防止抽搐)
// ==========================================
int16_t Cur_Speed_L = 0; 
int16_t Cur_Speed_R = 0; 

void Motor_SmoothSpeed(int16_t target_L, int16_t target_R)
{
    int16_t step = 80; // 加速度步长，越小越平滑
    
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
    // 1. 硬件全部初始化
    OLED_Init();
    Motor_Init();
    LED_Init();
    HCSR04_Init();
    PWM_SG90_Init();
    Flame_Init();
    Pump_Init();
    Buzzer_Init();
    DS18B20_Init(); // 测温初始化
    
    // 2. 初始状态归位
    Servo_SetAngle(90);
    Buzzer_OFF();
    Pump_Close();
    
    // 3. 上电延时启动 (给时间放置小车)
    OLED_ShowString(1, 1, "System Ready...");
    OLED_ShowString(4, 1, "Starting in 3s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 2s");
    Delay_ms(1000);
    OLED_ShowString(4, 1, "Starting in 1s");
    Delay_ms(1000);
    OLED_Clear();
    
    // 预留 OLED 第三行用于显示温度
    OLED_ShowString(3, 1, "Temp: --.- C");
    
    // 传感器变量定义
    uint16_t dist_front = 0;
    uint16_t dist_left = 0;
    uint16_t dist_right = 0;
    
    // 🌟 抗干扰核心变量 🌟
    uint16_t fire_missing_cnt = 0; // 记录火焰消失的时长
    uint8_t  last_fire_pos = 0;    // 记忆最后一次看到的有效火焰位置 (防屏幕闪烁)
    uint8_t  temp_tick = 0;        // 异步测温心跳计数器

    while (1)
    {
        // --------------------------------------------------
        // 【并行任务】异步测量温度 (自带硬件抗电磁干扰滤波)
        // --------------------------------------------------
        temp_tick++;
        if (temp_tick == 1) 
        {
            // 发送转换指令，立刻返回，不阻塞主循环
            DS18B20_ConvertT(); 
        }
        else if (temp_tick == 10) 
        {
            // 获取温度数据
            float temp = DS18B20_ReadT(); 
            
            // 🛡️ 核心滤噪：剔除水泵/电机干扰产生的 0.00 和重启故障码 85.0
            if (temp > 5.0 && temp < 80.0 && temp != 85.0)
            {
                int temp_int = (int)temp;                  // 整数部分
                int temp_frac = (int)(temp * 10) % 10;     // 小数第一位
                
                // 仅当读到正常物理温度时，才更新 OLED 屏幕
                OLED_ShowNum(3, 7, temp_int, 2);
                OLED_ShowChar(3, 9, '.');
                OLED_ShowNum(3, 10, temp_frac, 1);
            }
        }
        else if (temp_tick >= 20) 
        {
            // 约 2.4 秒一个周期，计时器归零
            temp_tick = 0; 
        }

        // --------------------------------------------------
        // 【最高优先级】全局感知：火灾检测
        // --------------------------------------------------
        uint8_t fire_pos = Flame_GetPosition(); // 这里面包含着100ms滤光算法
        
        if (fire_pos != 0) 
        {
            Current_State = STATE_FIRE;   
            fire_missing_cnt = 0;         // 只要看到火光，立刻清零消抖计数器
            last_fire_pos = fire_pos;     // 更新记忆值，供 OLED 稳定显示
        } 
        else 
        {
            if (Current_State == STATE_FIRE) 
            {
                fire_missing_cnt++; 
                // 连续约 1.4秒 没看到火，才确认火真灭了
                if (fire_missing_cnt > 20) 
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
            // --- 巡逻状态硬件表现 ---
            Buzzer_OFF(); 
            Pump_Close(); 
            OLED_ShowString(1, 1, "Mode: Patrol   ");
            
            // --- 纯超声波避障逻辑 ---
            Servo_SetAngle(90); 
            Delay_ms(20);       
            dist_front = HCSR04_GetDistance();
            OLED_ShowString(2, 1, "Dist:     cm");
            OLED_ShowNum(2, 7, dist_front, 3); 
            
            if (dist_front > DIST_SAFE) 
            {
                Motor_SmoothSpeed(500, 500); // 畅通无阻，全速
                LED1_ON();   
                LED2_OFF();  
            }
            else if (dist_front > DIST_WARN && dist_front <= DIST_SAFE) 
            {
                Motor_SmoothSpeed(300, 300); // 靠近障碍，平滑减速
                LED1_ON();
                LED2_OFF();
            }
            else 
            {
                // 遇到障碍物：停车侦查
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
                
                // 决策转向
                if (dist_right > dist_left) 
                {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400);  // 🌟 后退久一点，防止宽车身侧边刮墙               
                    Motor_SmoothSpeed(550, -550);  
                    Delay_ms(350);  // 右转               
                }
                else 
                {
                    Motor_SmoothSpeed(-350, -350); 
                    Delay_ms(400);  // 🌟 后退久一点
                    Motor_SmoothSpeed(-550, 550);  
                    Delay_ms(350);  // 左转               
                }
            }
        }
        else if (Current_State == STATE_FIRE)
        {
            // --- 灭火报警状态表现 ---
            LED1_OFF();  
            LED2_ON();   
            Buzzer_ON(); 
            OLED_ShowString(1, 1, "Mode: FIRE!    ");
            OLED_ShowString(2, 1, "Pos:  Exting...");
            OLED_ShowNum(2, 6, last_fire_pos, 1); // 🌟 使用记忆值显示，解决 0 的鬼畜闪烁！
            
            // --- 比例差速寻火动作 ---
            if (fire_pos == 1) 
            {
                Motor_SmoothSpeed(-500, 500); // 极左：大动作快转
                Pump_Close(); 
            }
            else if (fire_pos == 2) 
            {
                Motor_SmoothSpeed(-300, 300); // 偏左：小动作微调，防转过头
                Pump_Close(); 
            }
            else if (fire_pos == 5) 
            {
                Motor_SmoothSpeed(500, -500); // 极右：大动作快转
                Pump_Close();
            }
            else if (fire_pos == 4) 
            {
                Motor_SmoothSpeed(300, -300); // 偏右：小动作微调
                Pump_Close();
            }
            else if (fire_pos == 3) 
            {
                Motor_SmoothSpeed(0, 0); // 正中：停车
                Pump_Open();             // 开水泵！
            }
            else if (fire_pos == 0)
            {
                // 传感器短暂丢视野时：保持原地不动，保留当前水泵状态，等火再次出现
                Motor_SmoothSpeed(0, 0); 
            }
            
            Delay_ms(50); // 采样微调周期
        }
        
        // 主循环心跳
        Delay_ms(20); 
    }
}