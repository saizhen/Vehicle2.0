#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"

/*
 * ============================================================
 * 主机端 CAN 接收模块
 * ============================================================
 * 引脚：PA11 (CAN_RX, 浮空输入)
 *       PA12 (CAN_TX, 复用推挽，总线需要但主机只接收)
 * 外设：CAN1 (APB1)
 * 波特率：500 Kbps (APB1=36MHz, Prescaler=6, BS1=8tq, BS2=3tq)
 *
 * CAN 帧协议：
 *   ID  = 0x101  标准帧，从机测距数据
 *   DLC = 4
 *   Data[0~1]: 传感器1距离 uint16_t 大端，单位 cm
 *   Data[2~3]: 传感器2距离 uint16_t 大端，单位 cm
 * ============================================================
 */

#define CAN_ID_DIST     0x101U   /* 从机测距帧 ID */
#define CAN_DIST_INVALID 0xFFFFU /* 无效/超时距离标志 */

void    CAN_Master_Init(void);

/* 获取最新测距数据，有新帧返回 1，否则返回 0 */
uint8_t CAN_GetDist(uint16_t *dist1_cm, uint16_t *dist2_cm);

#endif
