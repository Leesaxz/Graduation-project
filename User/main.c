#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Flame.h"
#include "Pump.h"
#include "DS18B20.h"
#include "Buzzer.h"
#include "HCSR04.h"
#include "IRSensor.h"
#include "LED.h"
#include "Timer.h"   
#include "Encoder.h" 
#include "PWM_SG90.h" // 🚨 终于把你加进来了！

// ================= 状态机定义 =================
typedef enum {
    STATE_PATROL = 0,   // 巡逻寻迹
    STATE_OBSTACLE,     // 雷达避障扫描
    STATE_AIM_FIRE,     // 瞄准火源
    STATE_EXTINGUISH    // 喷水灭火
} SystemState_t;

SystemState_t CarState = STATE_PATROL; 

// ================= 全局传感器数据 =================
float Global_Temp = 0.0;     
uint16_t Global_Dist = 0;    
uint8_t Global_Fire = 0;     

// ================= 👑 PID 控制参数 =================
float Kp = 8.0, Ki = 0.5, Kd = 1.0; 
int16_t Target_Speed_L = 0, Target_Speed_R = 0; 

int16_t Err_L = 0, LastErr_L = 0, SumErr_L = 0, PWM_L = 0;
int16_t Err_R = 0, LastErr_R = 0, SumErr_R = 0, PWM_R = 0;

int16_t Limit_PWM(int16_t pwm) {
    if(pwm > 1000) return 1000;  
    if(pwm < -1000) return -1000;
    return pwm;
}

// ================= 辅助功能 =================
void Alarm_Effect(uint8_t State) {
    // 🔇 蜂鸣器暂时毒哑，安心调试。需要响的时候解开注释！
    // if (State) { Buzzer_ON(); LED2_ON(); } 
    // else { Buzzer_OFF(); LED2_OFF(); }
}

int main(void)
{
    // 1. 硬件外设大点兵
    OLED_Init();
    Motor_Init();
    Flame_Init();
    Pump_Init();
    DS18B20_Init();    
    HCSR04_Init();
    IRSensor_Init();
    LED_Init();       
    Encoder_Init(); 
    Timer_Init();
	Buzzer_Init();
    
    // 🚨 唤醒舵机！
    PWM_SG90_Init();
    Servo_SetAngle(90.0); // 上电立刻转到正前方！

    OLED_ShowString(1, 1, "RADAR SYS OK...");
    LED1_ON();        
    Delay_ms(1500);
    OLED_Clear();

    // 2. 状态机主循环
    while(1)
    {
        // ================= [10ms 控制层] =================
        if (Flag_10ms == 1)
        {
            Flag_10ms = 0;
            
            Global_Fire = Flame_GetPosition(); 
            
            // 雷达扫描子状态机专属变量
            static uint8_t  Avoid_SubState = 0; 
            static uint16_t Avoid_Timer = 0;    
            static uint16_t Dist_Left = 0;      
            static uint16_t Dist_Right = 0;     
            
            // ================= 🧠 【决策层】状态转移 =================
            if (Global_Fire > 0) 
            {
                if (Global_Fire == 3) CarState = STATE_EXTINGUISH;
                else CarState = STATE_AIM_FIRE;
            } 
            // 触发条件：正前方距离小于 15cm，启动避障！
            else if (Global_Dist > 0 && Global_Dist < 15 && CarState != STATE_OBSTACLE) 
            {
                CarState = STATE_OBSTACLE; 
                Avoid_SubState = 1; // 启动雷达扫描序列
                Avoid_Timer = 0;    
            } 
            // 避障退出逻辑
            else if (CarState == STATE_OBSTACLE) 
            {
                if (Avoid_SubState == 0) {
                    CarState = STATE_PATROL; // 避障全套动作打完，恢复巡逻
                }
            }
            else 
            {
                CarState = STATE_PATROL;   
            }

            // ================= ⚔️ 【战术层】执行动作 =================
            switch (CarState)
            {
                case STATE_EXTINGUISH:
                    Target_Speed_L = 0; Target_Speed_R = 0; 
                    Servo_SetAngle(90.0); // 灭火时必须盯住前方
                    Pump_Open();      
                    Alarm_Effect(1);
                    break;
                    
                case STATE_AIM_FIRE:
                    Pump_Close();
                    Alarm_Effect(0);
                    Servo_SetAngle(90.0);
                    if (Global_Fire < 3) { Target_Speed_L = -15; Target_Speed_R = 15; } 
                    else                 { Target_Speed_L = 15; Target_Speed_R = -15; } 
                    break;
                    
                // 🛸 核心：雷达扫描避障逻辑
                case STATE_OBSTACLE:
                    Pump_Close();
                    Alarm_Effect(0);
                    
                    if (Avoid_SubState == 1) {
                        Target_Speed_L = 0; Target_Speed_R = 0; // 刹车停住
                        Servo_SetAngle(160.0);                  // 扭头看左边
                        Avoid_Timer = 40;                       // 等待 400ms
                        Avoid_SubState = 2;                     
                    } 
                    else if (Avoid_SubState == 2) {
                        if (Avoid_Timer > 0) Avoid_Timer--;     
                        else {
                            Dist_Left = Global_Dist;            // 记录左边有多宽敞
                            Servo_SetAngle(20.0);               // 扭头看右边
                            Avoid_Timer = 40;                   // 等待 400ms
                            Avoid_SubState = 3;
                        }
                    }
                    else if (Avoid_SubState == 3) {
                        if (Avoid_Timer > 0) Avoid_Timer--;
                        else {
                            Dist_Right = Global_Dist;           // 记录右边有多宽敞
                            Servo_SetAngle(90.0);               // 雷达回正
                            Avoid_Timer = 30;                   // 等待 300ms
                            Avoid_SubState = 4;
                        }
                    }
                    else if (Avoid_SubState == 4) {
                        if (Avoid_Timer > 0) Avoid_Timer--;
                        else {
                            // 聪明的大脑开始分析两边的数据
                            if (Dist_Left > Dist_Right && Dist_Left > 15) {
                                Target_Speed_L = -20; Target_Speed_R = 20;  // 左边宽，向左转
                            } else if (Dist_Right >= Dist_Left && Dist_Right > 15) {
                                Target_Speed_L = 20; Target_Speed_R = -20;  // 右边宽，向右转
                            } else {
                                Target_Speed_L = -15; Target_Speed_R = -15; // 两边都是墙，倒车！
                            }
                            Avoid_Timer = 50; // 转向动作执行 500ms
                            Avoid_SubState = 5;
                        }
                    }
                    else if (Avoid_SubState == 5) {
                        if (Avoid_Timer > 0) Avoid_Timer--;
                        else {
                            Target_Speed_L = 0; Target_Speed_R = 0; // 转向结束，踩刹车
                            Avoid_SubState = 0;                     // 标志着避障序列彻底完成！
                        }
                    }
                    break;
                    
                case STATE_PATROL:
                    Pump_Close();
                    Alarm_Effect(0);
                    Servo_SetAngle(90.0); // 巡逻时雷达死死盯住正前方防撞
                    
                    uint8_t IR_L = IRSensor_GetLeft();
                    uint8_t IR_R = IRSensor_GetRight();
                    
                    if (IR_L == 0 && IR_R == 1) {
                        Target_Speed_L = -5; Target_Speed_R = 25; // 左偏，向左纠正
                    } else if (IR_L == 1 && IR_R == 0) {
                        Target_Speed_L = 25; Target_Speed_R = -5; // 右偏，向右纠正
                    } else {
                        Target_Speed_L = 20; Target_Speed_R = 20; // 居中直行
                    }
                    break;
            }

            // ================= ⚙️ 【执行层】PID 运算 =================
            int16_t Actual_Speed_L = Encoder_GetLeftSpeed();
            int16_t Actual_Speed_R = Encoder_GetRightSpeed();
            
            // 左轮
            Err_L = Target_Speed_L - Actual_Speed_L;
            SumErr_L += Err_L; 
            if(SumErr_L > 800) SumErr_L = 800; if(SumErr_L < -800) SumErr_L = -800;
            PWM_L = Kp * Err_L + Ki * SumErr_L + Kd * (Err_L - LastErr_L);
            LastErr_L = Err_L;
            
            // 右轮
            Err_R = Target_Speed_R - Actual_Speed_R;
            SumErr_R += Err_R;
            if(SumErr_R > 800) SumErr_R = 800; if(SumErr_R < -800) SumErr_R = -800;
            PWM_R = Kp * Err_R + Ki * SumErr_R + Kd * (Err_R - LastErr_R);
            LastErr_R = Err_R;
            
            Motor_SetSpeed(Limit_PWM(PWM_L), Limit_PWM(PWM_R));
        }
        
        // ================= [100ms 刷新层] =================
        if (Flag_100ms == 1)
        {
            Flag_100ms = 0;
            Global_Dist = HCSR04_GetDistance(); 
            
            if(CarState == STATE_PATROL)         OLED_ShowString(1, 1, "Mode: PATROL    ");
            else if(CarState == STATE_OBSTACLE)  OLED_ShowString(1, 1, "Mode: SCANNING..");
            else if(CarState == STATE_AIM_FIRE)  OLED_ShowString(1, 1, "Mode: AIMING... ");
            else if(CarState == STATE_EXTINGUISH)OLED_ShowString(1, 1, "Mode: FIRE PUT! ");
            
            OLED_ShowString(2, 1, "Dist:"); OLED_ShowNum(2, 6, Global_Dist, 3); OLED_ShowString(2, 9, "cm  ");
            OLED_ShowString(3, 1, "L:"); OLED_ShowNum(3, 3, Target_Speed_L, 3);
            OLED_ShowString(3, 8, "R:"); OLED_ShowNum(3, 10, Target_Speed_R, 3);
        }
        
        // ================= [2000ms 慢速层] =================
        if (Flag_2000ms == 1) { Flag_2000ms = 0; }
    }
}