#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

// ================================================================
// 引脚定义 (如果以后要改引脚，改这里即可)
// ================================================================
#define MPU_I2C_PORT    GPIOB
#define MPU_I2C_RCC     RCC_APB2Periph_GPIOB
#define MPU_SCL_PIN     GPIO_Pin_12  
#define MPU_SDA_PIN     GPIO_Pin_13

// MPU6050 AD0 引脚状态 (如果不接通常是接地->地址0xD0)
// 如果 AD0 接 VCC，地址变为 0xD2
#define MPU_ADDR        0xD0 

// ================================================================
// MPU6050 寄存器地址
// ================================================================
#define MPU_SMPLRT_DIV      0x19    // 采样率分频
#define MPU_CONFIG          0x1A    // 低通滤波配置
#define MPU_GYRO_CONFIG     0x1B    // 陀螺仪量程配置
#define MPU_ACCEL_CONFIG    0x1C    // 加速度计量程配置
#define MPU_ACCEL_XOUT_H    0x3B    // 加速度数据开始地址
#define MPU_GYRO_ZOUT_H     0x47    // Z轴角速度数据高8位
#define MPU_GYRO_ZOUT_L     0x48    // Z轴角速度数据低8位
#define MPU_PWR_MGMT_1      0x6B    // 电源管理 1
#define MPU_WHO_AM_I        0x75    // ID寄存器

// ================================================================
// 函数声明
// ================================================================
void MPU_Init(void);              // 初始化函数
uint8_t MPU_ReadID(void);         // 读取设备ID (测试通讯用)
int16_t MPU_Get_Gyro_Z(void);     // 读取 Z 轴原始数据

#endif
