#ifndef __MOTOR_H__
#define __MOTOR_H__

extern void Speed_Control();
extern void Motor_Init();
extern void Motor_Control();
extern void Motor_PID_Init();
extern void Motor_PID();
extern void Speed_Comput();
extern void Speed_Stable();
extern void Get_Motor_Speed_Init();
extern void Get_Motor_Speed();
extern void Speed_Chack();
extern void FuzzyPID();
extern double FuzzyKp(int16 e, double ec);
extern double FuzzyKi(int e, double ec);
extern double FuzzyKd(int e, double ec);
#endif //__MOTOR_H__