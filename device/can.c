#include "can.h"

static volatile uint16_t s_dist1     = CAN_DIST_INVALID;
static volatile uint16_t s_dist2     = CAN_DIST_INVALID;
static volatile uint8_t  s_new_data  = 0;

void CAN_Master_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    CAN_InitTypeDef         CAN_InitStructure;
    CAN_FilterInitTypeDef   CAN_FilterInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PA12: CAN_TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA11: CAN_RX */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CAN 时基：500 Kbps，APB1=36MHz */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_ABOM       = ENABLE;
    CAN_InitStructure.CAN_Mode       = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW        = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1        = CAN_BS1_8tq;
    CAN_InitStructure.CAN_BS2        = CAN_BS2_3tq;
    CAN_InitStructure.CAN_Prescaler  = 6;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* 过滤器：只接收 ID=0x101 标准帧 */
    CAN_FilterInitStructure.CAN_FilterNumber         = 0;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = (CAN_ID_DIST << 5);
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = (0x7FF << 5);
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t CAN_GetDist(uint16_t *dist1_cm, uint16_t *dist2_cm)
{
    if (!s_new_data) return 0;
    *dist1_cm  = s_dist1;
    *dist2_cm  = s_dist2;
    s_new_data = 0;
    return 1;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;
    CAN_Receive(CAN1, CAN_FIFO0, &rx);
    if (rx.StdId == CAN_ID_DIST && rx.DLC == 4) {
        s_dist1    = ((uint16_t)rx.Data[0] << 8) | rx.Data[1];
        s_dist2    = ((uint16_t)rx.Data[2] << 8) | rx.Data[3];
        s_new_data = 1;
    }
}
