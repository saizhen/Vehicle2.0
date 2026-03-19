#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f10x.h"

/* ================== 1. STBY 引脚定义 ================== */
// STBY (PA2) -> 必须接高电平芯片才工作
#define MOTOR_STBY_PORT     GPIOA
#define MOTOR_STBY_PIN      GPIO_Pin_2

/* ================== 2. 左轮 (Channel A) 定义 ================== */
// 方向控制: AIN1=PA3, AIN2=PA4
#define MOTOR_L_DIR_PORT    GPIOA
#define MOTOR_L_PIN_IN1     GPIO_Pin_3
#define MOTOR_L_PIN_IN2     GPIO_Pin_4

// 速度控制: PWMA=PA8 (TIM1_CH1)
#define MOTOR_L_PWM_PORT    GPIOA
#define MOTOR_L_PWM_PIN     GPIO_Pin_8

/* ================== 3. 右轮 (Channel B) 定义 ================== */
// 方向控制: BIN1=PA5, BIN2=PA6
#define MOTOR_R_DIR_PORT    GPIOA
#define MOTOR_R_PIN_IN1     GPIO_Pin_5
#define MOTOR_R_PIN_IN2     GPIO_Pin_6

// 速度控制: PWMB=PA9 (TIM1_CH2)
#define MOTOR_R_PWM_PORT    GPIOA
#define MOTOR_R_PWM_PIN     GPIO_Pin_9

/* ================== 函数声明 ================== */
void TB6612_Init(void);
void Motor_Left_Control(int speed);  // 控制 AIN/PWMA
void Motor_Right_Control(int speed); // 控制 BIN/PWMB
void Motor_Stop(void);               // 停车

#endif
