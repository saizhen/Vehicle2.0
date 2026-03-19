#include "Encoder.h"

// 初始化编码器
// 左轮 -> TIM2 (PA0, PA1)
// 右轮 -> TIM3 (PB4, PB5) [重映射]
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // ==========================================
    // 左轮 TIM2 (PA0, PA1)
    // ==========================================
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_Cmd(TIM2, ENABLE);

    // ==========================================
    // 右轮 TIM3 (PB4, PB5) - 完全重映射
    // ==========================================
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 解锁 PB4
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);     // 映射 TIM3 到 PB4/PB5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM3, &TIM_ICInitStructure); // 使用相同的滤波器配置
    
    TIM_Cmd(TIM3, ENABLE);
}

// 读取速度函数 (核心修复)
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;
    switch (TIMX)
    {
        case 2: // 读取左轮
            Encoder_TIM = (short)TIM_GetCounter(TIM2); 
            TIM_SetCounter(TIM2, 0); // 读完清零
            break;
            
        case 3: // 读取右轮 (注意这里是 TIM3)
            Encoder_TIM = (short)TIM_GetCounter(TIM3); 
            TIM_SetCounter(TIM3, 0); 
            break;
            
        default: 
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}