#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    int target_val;
    int actual_val;
    int error;
    int last_error;
    int integral;
} PID_TypeDef;

void PID_Init(void);
int PID_Realize(PID_TypeDef *pid, int target, int actual);

extern PID_TypeDef PID_Left;
extern PID_TypeDef PID_Right;

#endif