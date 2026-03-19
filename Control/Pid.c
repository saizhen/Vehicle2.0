#include "Pid.h"

PID_TypeDef PID_Left;
PID_TypeDef PID_Right;

void PID_Init(void)
{
    // 左轮参数
    PID_Left.Kp = 3.0f;   // 比例
    PID_Left.Ki = 1.0f;   // 积分
    PID_Left.Kd = 0.0f;
    PID_Left.integral = 0;
    PID_Left.last_error = 0;

    // 右轮参数
    PID_Right.Kp = 3.0f;
    PID_Right.Ki = 1.0f;
    PID_Right.Kd = 0.0f;
    PID_Right.integral = 0;
    PID_Right.last_error = 0;
}

int PID_Realize(PID_TypeDef *pid, int target, int actual)
{
    float output_f;
    int output;

    pid->target_val = target;
    pid->actual_val = actual;
    pid->error = pid->target_val - pid->actual_val;

    pid->integral += pid->error;
    
    // 积分限幅
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;

    output_f = (pid->Kp * pid->error) + 
               (pid->Ki * pid->integral) + 
               (pid->Kd * (pid->error - pid->last_error));

    pid->last_error = pid->error;

    // PWM 限幅 (-999 到 999)
    output = (int)output_f;
    if (output > 999) output = 999;
    if (output < -999) output = -999;

    return output;
}