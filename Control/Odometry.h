#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32f10x.h"
#include <math.h>

/* ==========================================================
 * 全局位置变量 (外部可访问)
 * X, Y 单位: cm
 * Yaw 单位: 弧度 (rad), 范围 -PI ~ +PI
 * ========================================================== */
extern float Global_X;
extern float Global_Y;
extern float Global_Yaw;

/* ==========================================================
 * 函数声明
 * ========================================================== */

// 初始化里程计 (归零)
void Odometry_Init(void);

/**
 * @brief  里程计核心更新函数
 * @param  enc_L: 左轮在这一段时间内的增量脉冲 (注意不是总计数值，是差值)
 * @param  enc_R: 右轮在这一段时间内的增量脉冲
 * @note   建议在定时器中断中每 10ms 或 20ms 调用一次
 */
void Odometry_Update(int enc_L, int enc_R);

#endif
