#include "Timer.h"
#include "Encoder.h"
#include "Pid.h"
#include "tb6612.h"
#include "Odometry.h"
#include "MPU6050.h" // 【新增】引入陀螺仪头文件

// 引用外部变量 (来自 main.c 或其他地方)
extern int Target_Left_Speed;
extern int Target_Right_Speed;
extern int Show_PWM_Left; 

extern PID_TypeDef PID_Left;
extern PID_TypeDef PID_Right;

// 【新增】全局角度变量，供 main 函数读取显示
float MPU_Total_Angle = 0.0f; 
// 【新增】零偏变量，由 main 函数校准后赋值给它
float Gyro_Z_Offset = 0.0f;   

void Timer4_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. 配置定时器 (10ms 中断 -> 100Hz)
    TIM_TimeBaseStructure.TIM_Period = 100 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 

    // 3. 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}

// =========================================================
// 核心中断函数：所有控制算法都在这里执行 (每 10ms 一次)
// =========================================================
void TIM4_IRQHandler(void)
{
    int current_speed_L, current_speed_R;
    int pwm_L, pwm_R;

    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        // -------------------------------------------------------------
        // 1. 读取传感器数据 (Sensors Update)
        // -------------------------------------------------------------
        
        // 1.1 读取编码器速度
        current_speed_L = Read_Encoder(2);
        current_speed_R = Read_Encoder(3);
        
        // 【新增】1.2 读取陀螺仪并积分
        // -----------------------------------------------------------
        int16_t raw_gyro = MPU_Get_Gyro_Z(); // 读取原始数据
        
        // 扣除零偏 (Gyro_Z_Offset 会在 main 函数里算好填进来)
        float true_gyro = raw_gyro - Gyro_Z_Offset;
        
        // 死区处理：如果静止时的噪音在 ±3 之间，强制归零
        if (true_gyro > -3.0f && true_gyro < 3.0f) true_gyro = 0;
        
        // 积分公式：Angle = Angle + (Speed * dt)
        // Speed = true_gyro / 16.4 (量程±2000时的系数)
        // dt    = 0.01 (定时器是 10ms)
        MPU_Total_Angle += (true_gyro / 16.4f) * 0.01f;
        // -----------------------------------------------------------

        // 1.3 (可选) 如果你还需要原来的里程计，可以保留
        Odometry_Update(current_speed_L, current_speed_R);

        // -------------------------------------------------------------
        // 2. 闭环 PID 计算 (Control Loop)
        // -------------------------------------------------------------
        // 注意：PID_Realize 返回的是计算好的 PWM 值
        pwm_L = PID_Realize(&PID_Left,  Target_Left_Speed,  current_speed_L);
        pwm_R = PID_Realize(&PID_Right, Target_Right_Speed, current_speed_R);

        // -------------------------------------------------------------
        // 3. 输出给电机 (Actuators Output)
        // -------------------------------------------------------------
        Motor_Left_Control(pwm_L);
        Motor_Right_Control(pwm_R);

        // 清除中断标志
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
