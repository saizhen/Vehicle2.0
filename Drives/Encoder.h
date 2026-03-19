#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

// 初始化编码器 
// 左轮: TIM2 (PA0, PA1)
// 右轮: TIM3 (PB4, PB5) -> 通过重映射实现
void Encoder_Init(void);

// 读取编码器数值 (读取后自动清零)
// 参数: TIMx (2 代表左轮TIM2，3 代表右轮TIM3)
int Read_Encoder(u8 TIMx);

#endif
