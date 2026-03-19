#include "Serial.h"
#include "OLED.h" // 用于调试显示

// 引用 Timer.c 里的全局变量
extern int Target_Left_Speed;
extern int Target_Right_Speed;

/**
  * @brief  初始化 USART3 (PB10 TX, PB11 RX)
  * 代码与之前保持一致，无需更改
  */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // PB10 -> TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB11 -> RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  USART3 中断服务函数
  * 修改：将指令改为 '1', '2', '3', '4', '0'
  */
void USART3_IRQHandler(void)
{
    uint8_t RxData;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        RxData = USART_ReceiveData(USART3); // 读取蓝牙发来的字符

        // 【调试】在 OLED 第一行最右边显示收到的字符
        // 这样你可以看到手机到底发过来的是 '1' 还是别的什么
        OLED_ShowChar(1, 15, RxData); 

        switch (RxData)
        {
            // === 前进 (Forward) ===
            case '1': 
                Target_Left_Speed =70;  
                Target_Right_Speed = 70; 
                break;

            // === 后退 (Back) ===
            case '2': 
                Target_Left_Speed = -70;
                Target_Right_Speed = -70;
                break;

            // === 左转 (Left) ===
            case '3': 
                Target_Left_Speed = 20; 
                Target_Right_Speed = 60; 
                break;

            // === 右转 (Right) ===
            case '4': 
                Target_Left_Speed = 60; 
                Target_Right_Speed = 20;
                break;

            // === 停止 (Stop) ===
            case '0': 
                Target_Left_Speed = 0;
                Target_Right_Speed = 0;
                break;
                
            default:
                // 收到其他无关字符保持不变
                break;
        }

        USART_ClearITPendingBit(USART3, USART_IT_RXNE); // 清除中断标志
    }
}