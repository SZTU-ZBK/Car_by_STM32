#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "./SYSTEM/sys/sys.h"
void Motor_Control_Init(void);
int Motor_PI (float current,float Target);
int Motor_PI_R (float current,float Target);
int Pos_PI_R (float current,float Target);
int Pos_PI_L (float current,float Target);
#endif