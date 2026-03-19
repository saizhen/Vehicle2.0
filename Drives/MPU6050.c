#include "MPU6050.h"

/* ==================================================================================
 * 1. 软件 I2C 底层驱动部分
 * ================================================================================== */

// 简单的微秒延时
static void I2C_Delay(void)
{
    volatile int i = 10; // 这里的数值决定 I2C 速度，10 大概对应 400KHz 左右
    while (i--);
}

// 写 SCL 电平
void MPU_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(MPU_I2C_PORT, MPU_SCL_PIN, (BitAction)BitValue);
    I2C_Delay();
}

// 写 SDA 电平
void MPU_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(MPU_I2C_PORT, MPU_SDA_PIN, (BitAction)BitValue);
    I2C_Delay();
}

// 读 SDA 电平
uint8_t MPU_R_SDA(void)
{
    uint8_t val = GPIO_ReadInputDataBit(MPU_I2C_PORT, MPU_SDA_PIN);
    I2C_Delay();
    return val;
}

// I2C 起始信号
void MPU_I2C_Start(void)
{
    MPU_W_SDA(1);
    MPU_W_SCL(1);
    MPU_W_SDA(0);
    MPU_W_SCL(0);
}

// I2C 停止信号
void MPU_I2C_Stop(void)
{
    MPU_W_SDA(0);
    MPU_W_SCL(1);
    MPU_W_SDA(1);
}

// 发送一个字节
void MPU_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        MPU_W_SDA(Byte & (0x80 >> i));
        MPU_W_SCL(1);
        MPU_W_SCL(0);
    }
}

// 接收一个字节
uint8_t MPU_I2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0;
    MPU_W_SDA(1); // 释放 SDA 总线以便读取
    for (i = 0; i < 8; i++)
    {
        MPU_W_SCL(1);
        if (MPU_R_SDA() == 1) { Byte |= (0x80 >> i); }
        MPU_W_SCL(0);
    }
    return Byte;
}

// 发送应答
void MPU_I2C_SendAck(uint8_t AckBit)
{
    MPU_W_SDA(AckBit);
    MPU_W_SCL(1);
    MPU_W_SCL(0);
}

// 接收应答
uint8_t MPU_I2C_ReceiveAck(void)
{
    uint8_t AckBit;
    MPU_W_SDA(1); // 释放 SDA
    MPU_W_SCL(1);
    AckBit = MPU_R_SDA();
    MPU_W_SCL(0);
    return AckBit;
}

/* ==================================================================================
 * 2. MPU6050 寄存器读写部分
 * ================================================================================== */

// 写指定寄存器
void MPU_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MPU_I2C_Start();
    MPU_I2C_SendByte(MPU_ADDR);     // 发送设备地址(写)
    MPU_I2C_ReceiveAck();
    MPU_I2C_SendByte(RegAddress);   // 发送寄存器地址
    MPU_I2C_ReceiveAck();
    MPU_I2C_SendByte(Data);         // 发送数据
    MPU_I2C_ReceiveAck();
    MPU_I2C_Stop();
}

// 读指定寄存器
uint8_t MPU_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MPU_I2C_Start();
    MPU_I2C_SendByte(MPU_ADDR);     // 发送设备地址(写)
    MPU_I2C_ReceiveAck();
    MPU_I2C_SendByte(RegAddress);   // 发送寄存器地址
    MPU_I2C_ReceiveAck();
    
    MPU_I2C_Start();                // 重复起始
    MPU_I2C_SendByte(MPU_ADDR | 0x01); // 发送设备地址(读)
    MPU_I2C_ReceiveAck();
    Data = MPU_I2C_ReceiveByte();
    MPU_I2C_SendAck(1);             // 发送非应答(NACK)，表示只读一个字节
    MPU_I2C_Stop();
    
    return Data;
}

/* ==================================================================================
 * 3. MPU6050 功能函数部分
 * ================================================================================== */

// MPU6050 初始化
void MPU_Init(void)
{
    // 1. 初始化 GPIO
    RCC_APB2PeriphClockCmd(MPU_I2C_RCC, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = MPU_SCL_PIN | MPU_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 重点：开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU_I2C_PORT, &GPIO_InitStructure);
    
    // 默认拉高
    GPIO_SetBits(MPU_I2C_PORT, MPU_SCL_PIN | MPU_SDA_PIN);
    
    // 2. 复位 MPU6050
    MPU_WriteReg(MPU_PWR_MGMT_1, 0x80); // 解除休眠 + 复位
    
    // 稍微延时等待复位完成
    uint32_t i;
    for(i=0; i<10000; i++); 
    
    MPU_WriteReg(MPU_PWR_MGMT_1, 0x00); // 唤醒
    
    // 3. 配置陀螺仪
    MPU_WriteReg(MPU_SMPLRT_DIV, 0x00); // 采样率分频，越小越快
    MPU_WriteReg(MPU_CONFIG, 0x06);     // 低通滤波 (0x06 是最强滤波，波形最平滑)
    
    // 量程配置：这是关键！
    // 0x00: ±250°/s
    // 0x18: ±2000°/s (推荐用于转弯很猛的小车，防止爆表)
    MPU_WriteReg(MPU_GYRO_CONFIG, 0x18); 
}

// 读取 ID
uint8_t MPU_ReadID(void)
{
    return MPU_ReadReg(MPU_WHO_AM_I);
}

// 读取 Z 轴角速度原始值 (16位有符号)
int16_t MPU_Get_Gyro_Z(void)
{
    uint8_t DataH, DataL;
    
    // 连续读取两个寄存器
    DataH = MPU_ReadReg(MPU_GYRO_ZOUT_H);
    DataL = MPU_ReadReg(MPU_GYRO_ZOUT_L);
    
    return (int16_t)((DataH << 8) | DataL);
}
