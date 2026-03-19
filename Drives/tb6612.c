#include "tb6612.h"

static int my_abs(int val) {
    return (val < 0) ? -val : val;
}

void TB6612_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 开启时钟：TIM1, GPIOA, AFIO
    // 现在所有引脚都在 GPIOA，所以只需要开 GPIOA 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // 2. 配置普通 GPIO (方向 + STBY)
    // 包含: PA2(STBY), PA3(AIN1), PA4(AIN2), PA5(BIN1), PA6(BIN2)
    GPIO_InitStructure.GPIO_Pin = MOTOR_STBY_PIN | 
                                  MOTOR_L_PIN_IN1 | MOTOR_L_PIN_IN2 |
                                  MOTOR_R_PIN_IN1 | MOTOR_R_PIN_IN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 PWM 引脚 (PA8, PA9) -> 复用推挽
    // PA8 = TIM1_CH1 (PWMA)
    // PA9 = TIM1_CH2 (PWMB)
    GPIO_InitStructure.GPIO_Pin = MOTOR_L_PWM_PIN | MOTOR_R_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 必须是 AF_PP
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 定时器基础配置 (1kHz)
    TIM_TimeBaseStructure.TIM_Period = 999;       // ARR
    TIM_TimeBaseStructure.TIM_Prescaler = 71;     // PSC (72M/72 = 1M计数)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 5. PWM 模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 配置通道 1 (PA8 -> Left Motor A)
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 配置通道 2 (PA9 -> Right Motor B)
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 6. 开启定时器
    TIM_Cmd(TIM1, ENABLE);
    // 【关键】高级定时器必须开启主输出 (MOE)
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 7. 拉高 STBY，开启驱动芯片
    GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN); // PA2 = High

    // 默认停止
    Motor_Stop();
}

// 左轮控制 (PA3, PA4, PA8)
void Motor_Left_Control(int speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;

    if (speed >= 0) {
        // 正转逻辑：AIN1=0, AIN2=1 (请根据实际转向调整)
        GPIO_ResetBits(MOTOR_L_DIR_PORT, MOTOR_L_PIN_IN1); 
        GPIO_SetBits(MOTOR_L_DIR_PORT, MOTOR_L_PIN_IN2);
    } else {
        // 反转逻辑：AIN1=1, AIN2=0
        GPIO_SetBits(MOTOR_L_DIR_PORT, MOTOR_L_PIN_IN1);
        GPIO_ResetBits(MOTOR_L_DIR_PORT, MOTOR_L_PIN_IN2);
    }

    // 设置通道 1 (PA8)
    TIM_SetCompare1(TIM1, my_abs(speed));
}

// 右轮控制 (PA5, PA6, PA9)
void Motor_Right_Control(int speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;

    if (speed >= 0) {
        GPIO_ResetBits(MOTOR_R_DIR_PORT, MOTOR_R_PIN_IN1); 
        GPIO_SetBits(MOTOR_R_DIR_PORT, MOTOR_R_PIN_IN2);
    } else {
        GPIO_SetBits(MOTOR_R_DIR_PORT, MOTOR_R_PIN_IN1);
        GPIO_ResetBits(MOTOR_R_DIR_PORT, MOTOR_R_PIN_IN2);
    }

    // 设置通道 2 (PA9)
    TIM_SetCompare2(TIM1, my_abs(speed));
}

void Motor_Stop(void)
{
    // 停止方向引脚
    GPIO_ResetBits(MOTOR_L_DIR_PORT, MOTOR_L_PIN_IN1 | MOTOR_L_PIN_IN2);
    GPIO_ResetBits(MOTOR_R_DIR_PORT, MOTOR_R_PIN_IN1 | MOTOR_R_PIN_IN2);
    // PWM 归零
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare2(TIM1, 0);
}
