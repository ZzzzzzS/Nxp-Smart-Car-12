#ifndef __MOTOR_H__
#define __MOTOR_H__

extern void Motor_Init();
extern void Motor_Control();
extern void Motor_PID_Init();
extern void Motor_PID();
extern void Get_Motor_Speed_Init();
extern void Get_Motor_Speed();
extern void FuzzyPID();
extern void FuzzyKp();
extern void FuzzyKi();
extern void FuzzyKd();

#endif //__MOTOR_H__