#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "HCSR04.h"
#include "Timer.h"   
#include "Encoder.h" 
#include "PWM_SG90.h" 

// 暂时屏蔽不需要的模块，让系统极度干净
// #include "Flame.h"
// #include "Pump.h"
// #include "DS18B20.h"
// #include "Buzzer.h"
// #include "IRSensor.h"
// #include "LED.h"

// ================= 精简版：纯雷达巡航状态机 =================
typedef enum {
    STATE_CRUISE = 0,   // 直线巡航
    STATE_SCAN,         // 停车，启动摇头雷达扫描
    STATE_TURN          // 执行避障转向动作
} SystemState_t;

SystemState_t CarState = STATE_CRUISE; 

// ================= 全局变量 =================
uint16_t Global_Dist = 0;    

// ================= 👑 温和版 PID 控制参数 =================
// Kp 调小到了 4.0，防止轮子抽搐打滑
float Kp = 4.0, Ki = 0.5, Kd = 1.0; 
int16_t Target_Speed_L = 0, Target_Speed_R = 0; 

int16_t Err_L = 0, LastErr_L = 0, SumErr_L = 0, PWM_L = 0;
int16_t Err_R = 0, LastErr_R = 0, SumErr_R = 0, PWM_R = 0;

int16_t Limit_PWM(int16_t pwm) {
    if(pwm > 1000) return 1000;  
    if(pwm < -1000) return -1000;
    return pwm;
}

int main(void)
{
    // 1. 硬件初始化 (只初始化巡航需要的模块)
    OLED_Init();
    Motor_Init();
    HCSR04_Init();
    Encoder_Init(); 
    PWM_SG90_Init();
    Timer_Init();   // SysTick 心跳
    
    // 初始化时让舵机回正，只发一次指令！
    Servo_SetAngle(90.0); 

    OLED_ShowString(1, 1, "RADAR CRUISE..");
    Delay_ms(1500);
    OLED_Clear();

    // 2. 主循环
    while(1)
    {
        // ================= [10ms 动作控制层] =================
        if (Flag_10ms == 1)
        {
            Flag_10ms = 0;
            
            // 静态变量，用于管理复杂的雷达扫描动作
            static uint8_t  Scan_Step = 0; 
            static uint16_t Action_Timer = 0;    
            static uint16_t Dist_Left = 0;      
            static uint16_t Dist_Right = 0;     
            
            // 🧠 【状态转移触发器】
            if (CarState == STATE_CRUISE) 
            {
                // 如果在直行时，前方距离小于 20cm，立刻刹车并开始扫描
                if (Global_Dist > 0 && Global_Dist < 20) {
                    CarState = STATE_SCAN;
                    Scan_Step = 1; // 启动扫描第一步
                }
            }

            // ⚔️ 【战术执行层】
            switch (CarState)
            {
                case STATE_CRUISE:
                    // 匀速直行巡航 (15 比较慢，适合测试)
                    Target_Speed_L = 15; 
                    Target_Speed_R = 15; 
                    break;
                    
                case STATE_SCAN:
                    Target_Speed_L = 0; // 停车
                    Target_Speed_R = 0; 
                    
                    // 采用“步进式”扫描，彻底解决舵机发抖问题！
                    if (Scan_Step == 1) {
                        Servo_SetAngle(160.0);       // 指令只发一次：看左边
                        Action_Timer = 40;           // 等待 400ms 让物理舵机转到位
                        Scan_Step = 2;               // 步进到下一状态
                    } 
                    else if (Scan_Step == 2) {
                        if (Action_Timer > 0) Action_Timer--;
                        else {
                            Dist_Left = Global_Dist; // 读取左边距离
                            Servo_SetAngle(20.0);    // 指令只发一次：看右边
                            Action_Timer = 40;       
                            Scan_Step = 3;
                        }
                    }
                    else if (Scan_Step == 3) {
                        if (Action_Timer > 0) Action_Timer--;
                        else {
                            Dist_Right = Global_Dist; // 读取右边距离
                            Servo_SetAngle(90.0);     // 指令只发一次：回正
                            Action_Timer = 30;        // 等待 300ms 回正
                            Scan_Step = 4;
                        }
                    }
                    else if (Scan_Step == 4) {
                        if (Action_Timer > 0) Action_Timer--;
                        else {
                            // 大脑根据刚才侦察的距离做决策！
                            CarState = STATE_TURN;    // 准备转向
                            Action_Timer = 50;        // 预设转向动作持续 500ms
                            
                            if (Dist_Left > Dist_Right && Dist_Left > 15) {
                                Target_Speed_L = -15; Target_Speed_R = 15;  // 向左转
                            } else if (Dist_Right >= Dist_Left && Dist_Right > 15) {
                                Target_Speed_L = 15; Target_Speed_R = -15;  // 向右转
                            } else {
                                Target_Speed_L = -15; Target_Speed_R = -15; // 死胡同，倒车
                                Action_Timer = 80; // 倒车多倒一会 (800ms)
                            }
                        }
                    }
                    break;
                    
                case STATE_TURN:
                    // 正在执行转向或倒车动作...
                    if (Action_Timer > 0) {
                        Action_Timer--;
                    } else {
                        // 转向时间到，动作完成，恢复直行巡航！
                        CarState = STATE_CRUISE;
                    }
                    break;
            }

            // ⚙️ 【PID 底层稳速闭环】(保持不变)
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
        
        // ================= [100ms UI 刷新层] =================
        if (Flag_100ms == 1)
        {
            Flag_100ms = 0;
            Global_Dist = HCSR04_GetDistance(); // 采集超声波雷达数据
            
            // 显示当前小车的“内心情感状态”
            if(CarState == STATE_CRUISE)      OLED_ShowString(1, 1, "Mode: CRUISE    ");
            else if(CarState == STATE_SCAN)   OLED_ShowString(1, 1, "Mode: SCANNING  ");
            else if(CarState == STATE_TURN)   OLED_ShowString(1, 1, "Mode: TURNING   ");
            
            OLED_ShowString(2, 1, "Dist:"); OLED_ShowNum(2, 6, Global_Dist, 3); OLED_ShowString(2, 9, "cm  ");
            OLED_ShowString(3, 1, "L:"); OLED_ShowNum(3, 3, Target_Speed_L, 3);
            OLED_ShowString(3, 8, "R:"); OLED_ShowNum(3, 10, Target_Speed_R, 3);
        }
    }
}