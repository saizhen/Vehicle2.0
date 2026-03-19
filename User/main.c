#include "stm32f10x.h"
#include "tb6612.h"
#include "Encoder.h"
#include "OLED.h"
#include "Pid.h"
#include "Timer.h"
#include "MPU6050.h" 
#include "Odometry.h"
#include <math.h>

/* ============================================================
 * 全局变量 & 外部引用
 * ============================================================ */
int Target_Left_Speed = 0;
int Target_Right_Speed = 0;
int Show_PWM_Left = 0; 

extern float MPU_Total_Angle; 
extern float Gyro_Z_Offset;
extern float Global_X;
extern float Global_Y;

/* ============================================================
 * 运行参数配置区
 * ============================================================ */
// 【修改】：改为正方形，且距离是原来(60)的一倍
#define SQUARE_SIDE     80.0f  // 正方形边长 (单位: cm)
#define SPEED_STRAIGHT  140      // 直行基础速度

/* ============================================================
 * 辅助工具函数
 * ============================================================ */
void Delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for(i = 0; i < ms; i++) {
        for(j = 0; j < 8000; j++);
    }
}

float my_abs_f(float val) {
    return (val < 0) ? -val : val;
}

void Stop_Car(void) {
    Target_Left_Speed = 0;
    Target_Right_Speed = 0;
    Delay_ms(300); 
    
    PID_Left.integral = 0;
    PID_Right.integral = 0;
    PID_Left.last_error = 0;
    PID_Right.last_error = 0;
}

/* ============================================================
 * 核心功能 1：绝对角度丝滑转弯 (不再清零陀螺仪)
 * 参数 target_angle: 在全局坐标系下的绝对目标角度
 * ============================================================ */
void Turn_With_Gyro(float target_angle)
{
    // 【核心修改】：删除了 MPU_Total_Angle = 0.0f; 让陀螺仪保持绝对记忆
    
    PID_Left.integral = 0;
    PID_Right.integral = 0;
    
    while(1)
    {
        // 计算当前离“绝对目标角度”还差多少
        float error = target_angle - MPU_Total_Angle;
        
        if (my_abs_f(error) < 1.0f) break; 
        
        float k_angle = 1.2f; 
        int turn_speed = (int)(my_abs_f(error) * k_angle);
        
        if (turn_speed > 35) turn_speed = 35; 
        if (turn_speed < 12) turn_speed = 12; 
        
        if (error > 0) { 
            Target_Left_Speed  = -turn_speed;
            Target_Right_Speed =  turn_speed;
        } else {         
            Target_Left_Speed  =  turn_speed;
            Target_Right_Speed = -turn_speed;
        }
        
        OLED_ShowString(3, 1, "State: Turning ");
        OLED_ShowString(4, 1, "Tgt: ");
        OLED_ShowNum(4, 6, (int)target_angle, 3);
        OLED_ShowString(4, 11, "V:");
        OLED_ShowNum(4, 13, turn_speed, 2); 
        
        Delay_ms(10); 
    }
    Stop_Car(); 
}

/* ============================================================
 * 核心功能 2：绝对角度辅助走直线
 * 参数 distance_cm: 要走的距离
 * 参数 target_angle: 直行时需要维持的绝对角度 (防跑偏)
 * ============================================================ */
void Go_Straight_With_Gyro(float distance_cm, float target_angle)
{
    float start_x = Global_X;
    float start_y = Global_Y;

    // 【核心修改】：删除了 MPU_Total_Angle = 0.0f;

    PID_Left.integral = 0;
    PID_Right.integral = 0;

    while(1)
    {
        float diff_x = Global_X - start_x;
        float diff_y = Global_Y - start_y;
        float current_dist = sqrt(diff_x * diff_x + diff_y * diff_y);

        if (current_dist >= distance_cm) break;

        // 根据“绝对目标角度”进行纠偏
        float error = target_angle - MPU_Total_Angle; 
        float k_gyro = 2.0f; 
        
        Target_Left_Speed  = SPEED_STRAIGHT - (int)(error * k_gyro);
        Target_Right_Speed = SPEED_STRAIGHT + (int)(error * k_gyro);

        OLED_ShowString(3, 1, "State: Straight");
        OLED_ShowString(4, 1, "Dist:");
        OLED_ShowNum(4, 6, (int)current_dist, 3);
        OLED_ShowString(4, 9, "/");
        OLED_ShowNum(4, 10, (int)distance_cm, 3);
        
        Delay_ms(10);
    }
    Stop_Car(); 
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_ms(200);
    
    OLED_Init();
    MPU_Init();    
    TB6612_Init(); 
    Encoder_Init();
    PID_Init();    
    Odometry_Init(); 
    
    OLED_Clear();
    OLED_ShowString(1, 1, "Calibrating...");
    
    float sum = 0;
    for(int i=0; i<200; i++)
    {
        sum += MPU_Get_Gyro_Z();
        Delay_ms(5);
    }
    Gyro_Z_Offset = sum / 200.0f;
    
    Timer4_Init(); 
    
    OLED_Clear();
    OLED_ShowString(1, 1, "Sys Ready!");
    OLED_ShowString(2, 1, "Square Start");
    Delay_ms(1500);

    // ==========================================
    // 动作脚本区：绝对坐标下的正方形
    // ==========================================
    int loop_count = 1; 
    
    while (1)
    {
        OLED_Clear(); 
        OLED_ShowString(1, 1, "Loop:");
        OLED_ShowNum(1, 6, loop_count, 3);

        // 第 1 条边，维持 0 度朝向
        OLED_ShowString(2, 1, "Edge 1         ");
        Go_Straight_With_Gyro(SQUARE_SIDE, 0.0f);
        Delay_ms(500); 

        // 转到全局 90 度
        OLED_ShowString(2, 1, "Turn to 90     ");
        Turn_With_Gyro(90.0f);
        Delay_ms(500);

        // 第 2 条边，维持 90 度朝向
        OLED_ShowString(2, 1, "Edge 2         ");
        Go_Straight_With_Gyro(SQUARE_SIDE, 90.0f);
        Delay_ms(500);

        // 转到全局 180 度
        OLED_ShowString(2, 1, "Turn to 180    ");
        Turn_With_Gyro(180.0f);
        Delay_ms(500);
        
        // 第 3 条边，维持 180 度朝向
        OLED_ShowString(2, 1, "Edge 3         ");
        Go_Straight_With_Gyro(SQUARE_SIDE, 180.0f);
        Delay_ms(500);

        // 转到全局 270 度
        OLED_ShowString(2, 1, "Turn to 270    ");
        Turn_With_Gyro(270.0f);
        Delay_ms(500);
        
        // 第 4 条边，维持 270 度朝向
        OLED_ShowString(2, 1, "Edge 4         ");
        Go_Straight_With_Gyro(SQUARE_SIDE, 270.0f);
        Delay_ms(500);

        // 闭环：转到全局 360 度 (恢复发车朝向)
        OLED_ShowString(2, 1, "Turn to 360    ");
        Turn_With_Gyro(360.0f);
        
        // ==========================================
        // 收尾与倒计时
        // ==========================================
        Stop_Car(); 
        OLED_Clear();
        
        OLED_ShowString(1, 1, "Loop ");
        OLED_ShowNum(1, 6, loop_count, 3);
        OLED_ShowString(1, 10, "Done!");
        
        OLED_ShowString(2, 1, "Ang:");
        OLED_ShowSignedNum(2, 5, (int)MPU_Total_Angle, 4);
        
        OLED_ShowString(3, 1, "Waiting...");
        
        // 10秒倒计时
        for(int wait_time = 10; wait_time > 0; wait_time--)
        {
            OLED_ShowString(4, 1, "Time Left: ");
            OLED_ShowNum(4, 12, wait_time, 2);
            OLED_ShowString(4, 14, "s ");
            Delay_ms(1000); 
        }
        
        // 【最关键的一步】：一圈跑完后，把 360 度扣除，完美衔接下一圈的 0 度！
        // 这样小车就算跑 100 圈，角度也不会溢出，并且始终以发车的那条线为绝对参考。
        MPU_Total_Angle -= 360.0f; 
        
        loop_count++; 
    }
}