#ifndef __AVOIDANCE_H
#define __AVOIDANCE_H

#include "stm32f10x.h"

/*
 * ============================================================
 * 避障决策模块（主机端）
 * ============================================================
 * 依赖：can.h 提供的测距数据
 * 输出：通过外部变量 Target_Left_Speed / Target_Right_Speed
 *       控制电机（与现有 PID 控制链兼容）
 *
 * 避障策略：
 *   dist1 = 左前方传感器，dist2 = 右前方传感器
 *   两路均 > AVOID_SAFE_DIST  → 正常行驶（不干预）
 *   任一路 < AVOID_STOP_DIST  → 立即停车
 *   仅 dist1 < AVOID_SAFE_DIST → 右转避障
 *   仅 dist2 < AVOID_SAFE_DIST → 左转避障
 * ============================================================
 */

#define AVOID_SAFE_DIST   40U   /* 安全距离 cm，小于此值开始避障 */
#define AVOID_STOP_DIST   15U   /* 紧急停车距离 cm */

typedef enum {
    AVOID_STATE_CLEAR = 0,  /* 无障碍，正常行驶 */
    AVOID_STATE_LEFT,       /* 左转避障 */
    AVOID_STATE_RIGHT,      /* 右转避障 */
    AVOID_STATE_STOP        /* 紧急停车 */
} AvoidState_t;

/* 根据两路距离数据做决策，返回当前避障状态 */
AvoidState_t Avoidance_Update(uint16_t dist1_cm, uint16_t dist2_cm);

/* 将避障决策写入电机目标速度（外部变量由 main.c 定义） */
void Avoidance_Apply(AvoidState_t state);

#endif
