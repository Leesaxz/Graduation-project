#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "HCSR04.h"
#include "Timer.h"   
#include "Encoder.h" 
#include "PWM_SG90.h" 
#include "Buzzer.h"  

// ================= 系统状态与全局变量 =================
typedef enum {
    STATE_STARTUP = 0,  // 开机延时准备
    STATE_CRUISE,       // 巡航（超声波+红外融合）
    STATE_SCAN,         // 停车摇头扫描
    STATE_BACKUP,       // 倒车避险
    STATE_TURN          // 原地转弯
} SystemState_t;

SystemState_t CarState = STATE_STARTUP; 
uint16_t Global_Dist = 0;   // 超声波距离

// PID 参数
float Kp = 5.0, Ki = 1.5, Kd = 0.5; 
int16_t Target_Speed_L = 0, Target_Speed_R = 0; 
int16_t SumErr_L = 0, SumErr_R = 0;
int16_t LastErr_L = 0, LastErr_R = 0;
int16_t Actual_Speed_L = 0, Actual_Speed_R = 0;

// ================= 适配最新 PCB 的引脚初始化 =================
void HardWare_Extra_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 三路红外避障模块 (移至 PB12, PB13, PB14) -> 上拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 2. 继电器/水泵控制 (PC13) -> 推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 默认关闭水泵
}

// PWM 限幅防打齿
int16_t Limit_PWM(int16_t pwm) {
    if(pwm > 800) return 800;  
    if(pwm < -800) return -800;
    return pwm;
}

int main(void)
{
    // 1. 底层大满配初始化 (OLED 光荣回归！)
    OLED_Init();
    Motor_Init();
    Encoder_Init(); 
    PWM_SG90_Init();
    HCSR04_Init();
    HardWare_Extra_Init();
    Timer_Init();   
    Buzzer_Init(); // 注意检查 Buzzer.c 里是否已改为 PB4
    
    Servo_SetAngle(90.0); // 舵机开机必须回正
    
    OLED_ShowString(1, 1, "SYSTEM BOOTING..");

    // 2. 主循环
    while(1)
    {
        // ================= [10ms 高频动作与PID层] =================
        if (Flag_10ms == 1)
        {
            Flag_10ms = 0;
            
            static uint8_t  Scan_Step = 0; 
            static uint16_t Action_Timer = 0;    
            static uint16_t Dist_L = 0, Dist_R = 0;
            
            // 读取红外避障状态 (0=有障碍, 1=安全) - 对应新引脚
            uint8_t IR_Left  = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12); // 假设 PB12 为左
            uint8_t IR_Front = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13); // 假设 PB13 为前
            uint8_t IR_Right = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14); // 假设 PB14 为右

            // 🧠 核心状态机决策
            switch (CarState) {
                
                // --- 阶段0：开机安全延时 (约 3 秒) ---
                case STATE_STARTUP:
                    Target_Speed_L = 0; Target_Speed_R = 0;
                    Action_Timer++;
                    if(Action_Timer > 300) { 
                        CarState = STATE_CRUISE; 
                        Action_Timer = 0;
                        OLED_Clear(); // 倒计时结束，清屏准备显示数据
                    }
                    break;

                // --- 阶段1：多传感器融合巡航 ---
                case STATE_CRUISE:
                    // 紧急情况：正前方红外贴脸了，或者超声波极近 -> 立刻倒车！
                    if (IR_Front == 0 || (Global_Dist > 0 && Global_Dist < 15)) {
                        CarState = STATE_BACKUP;
                        Action_Timer = 40; // 倒车 400ms
                    }
                    // 警报情况：超声波发现中距离障碍，或者侧面红外触发 -> 停车扫描
                    else if ((Global_Dist >= 15 && Global_Dist < 35) || IR_Left == 0 || IR_Right == 0) {
                        CarState = STATE_SCAN;
                        Scan_Step = 1;
                    }
                    // 安全情况：全速前进
                    else {
                        Target_Speed_L = 35; 
                        Target_Speed_R = 35; 
                    }
                    break;
                    
                // --- 阶段2：停车雷达扫描 ---
                case STATE_SCAN:
                    Target_Speed_L = 0; Target_Speed_R = 0; // 踩刹车
                    
                    if (Scan_Step == 1) { 
                        Servo_SetAngle(160.0); Action_Timer = 50; Scan_Step = 2; // 看左边
                    } 
                    else if (Scan_Step == 2) {
                        if (Action_Timer > 0) Action_Timer--;
                        else { Dist_L = Global_Dist; Servo_SetAngle(20.0); Action_Timer = 60; Scan_Step = 3; } // 看右边
                    }
                    else if (Scan_Step == 3) {
                        if (Action_Timer > 0) Action_Timer--;
                        else { Dist_R = Global_Dist; Servo_SetAngle(90.0); Action_Timer = 40; Scan_Step = 4; } // 看回正前方
                    }
                    else if (Scan_Step == 4) {
                        if (Action_Timer > 0) Action_Timer--;
                        else {
                            CarState = STATE_TURN; // 扫描完毕，决定转向
                            Action_Timer = 50;     // 转向动作维持 500ms
                            
                            // 融合判断：超声波哪边空旷，且红外哪边安全，就往哪转
                            if (Dist_L > Dist_R && IR_Left == 1) {
                                Target_Speed_L = -25; Target_Speed_R = 25; // 左转
                            } else if (Dist_R >= Dist_L && IR_Right == 1) {
                                Target_Speed_L = 25; Target_Speed_R = -25; // 右转
                            } else {
                                // 两边都有障碍，原路倒车
                                CarState = STATE_BACKUP;
                                Action_Timer = 60;
                            }
                        }
                    }
                    break;
                
                // --- 阶段3：战术倒车 ---
                case STATE_BACKUP:
                    Target_Speed_L = -25; Target_Speed_R = -25; 
                    if (Action_Timer > 0) Action_Timer--;
                    else {
                        // 倒车完后强制右转掉头一下，防止死胡同卡死
                        CarState = STATE_TURN; 
                        Target_Speed_L = 25; Target_Speed_R = -25; 
                        Action_Timer = 60; 
                    }
                    break;

                // --- 阶段4：执行转弯 ---
                case STATE_TURN:
                    if (Action_Timer > 0) Action_Timer--;
                    else CarState = STATE_CRUISE; // 转弯结束，恢复巡航
                    break;
            }

            // ⚙️ PID 闭环运算
            Actual_Speed_L = Encoder_GetLeftSpeed();
            Actual_Speed_R = Encoder_GetRightSpeed();
            
            int16_t Err_L = Target_Speed_L - Actual_Speed_L;
            SumErr_L += Err_L; if(SumErr_L > 1000) SumErr_L = 1000; if(SumErr_L < -1000) SumErr_L = -1000;
            int16_t PWM_L = Kp * Err_L + Ki * SumErr_L + Kd * (Err_L - LastErr_L);
            LastErr_L = Err_L;

            int16_t Err_R = Target_Speed_R - Actual_Speed_R;
            SumErr_R += Err_R; if(SumErr_R > 1000) SumErr_R = 1000; if(SumErr_R < -1000) SumErr_R = -1000;
            int16_t PWM_R = Kp * Err_R + Ki * SumErr_R + Kd * (Err_R - LastErr_R);
            LastErr_R = Err_R;
            
            Motor_SetSpeed(Limit_PWM(PWM_L), Limit_PWM(PWM_R));
        }
        
        // ================= [100ms OLED 传感器面板层] =================
        if (Flag_100ms == 1)
        {
            Flag_100ms = 0;
            Global_Dist = HCSR04_GetDistance(); 
            
            if(CarState == STATE_STARTUP)     OLED_ShowString(1, 1, "Mode: WAIT 3s ");
            else if(CarState == STATE_CRUISE) OLED_ShowString(1, 1, "Mode: CRUISING");
            else if(CarState == STATE_SCAN)   OLED_ShowString(1, 1, "Mode: SCANNING");
            else if(CarState == STATE_BACKUP) OLED_ShowString(1, 1, "Mode: BACKUP  ");
            else                              OLED_ShowString(1, 1, "Mode: TURNING ");
            
            // 显示真实速度，方便你一眼看出右轮掉没掉线
            if(Actual_Speed_L < 0) { OLED_ShowString(2,1,"L:-"); OLED_ShowNum(2,4,-Actual_Speed_L,3); }
            else { OLED_ShowString(2,1,"L: "); OLED_ShowNum(2,4,Actual_Speed_L,3); }
            
            if(Actual_Speed_R < 0) { OLED_ShowString(2,9,"R:-"); OLED_ShowNum(2,12,-Actual_Speed_R,3); }
            else { OLED_ShowString(2,9,"R: "); OLED_ShowNum(2,12,Actual_Speed_R,3); }

            OLED_ShowString(3, 1, "Dist:"); OLED_ShowNum(3, 6, Global_Dist, 3);
            OLED_ShowString(3, 12, "cm");
        }
    }
}