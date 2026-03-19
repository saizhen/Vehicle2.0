#include "avoidance.h"

/* 引用主机 main.c 中的电机目标速度变量 */
extern int Target_Left_Speed;
extern int Target_Right_Speed;

#define AVOID_TURN_SPEED  80   /* 避障转弯速度 */

AvoidState_t Avoidance_Update(uint16_t dist1_cm, uint16_t dist2_cm)
{
    /* 任一路无效值视为极近障碍 */
    if (dist1_cm == 0xFFFF) dist1_cm = 0;
    if (dist2_cm == 0xFFFF) dist2_cm = 0;

    if (dist1_cm < AVOID_STOP_DIST || dist2_cm < AVOID_STOP_DIST)
        return AVOID_STATE_STOP;

    if (dist1_cm < AVOID_SAFE_DIST && dist2_cm < AVOID_SAFE_DIST)
        return AVOID_STATE_STOP;   /* 两侧均受阻，停车等待 */

    if (dist1_cm < AVOID_SAFE_DIST)
        return AVOID_STATE_RIGHT;  /* 左侧近，向右避 */

    if (dist2_cm < AVOID_SAFE_DIST)
        return AVOID_STATE_LEFT;   /* 右侧近，向左避 */

    return AVOID_STATE_CLEAR;
}

void Avoidance_Apply(AvoidState_t state)
{
    switch (state) {
        case AVOID_STATE_STOP:
            Target_Left_Speed  = 0;
            Target_Right_Speed = 0;
            break;
        case AVOID_STATE_LEFT:
            Target_Left_Speed  = -AVOID_TURN_SPEED;
            Target_Right_Speed =  AVOID_TURN_SPEED;
            break;
        case AVOID_STATE_RIGHT:
            Target_Left_Speed  =  AVOID_TURN_SPEED;
            Target_Right_Speed = -AVOID_TURN_SPEED;
            break;
        case AVOID_STATE_CLEAR:
        default:
            /* 不干预，由上层行驶逻辑控制速度 */
            break;
    }
}
