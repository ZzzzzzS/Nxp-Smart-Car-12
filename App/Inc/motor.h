#ifndef __MOTOR_H__
#define __MOTOR_H__

extern void Motor_Init();
extern void Motor_Control();
extern void Motor_PID_Init();
extern void Motor_PID();
extern void Get_Motor_Speed_Init();
extern void Get_Motor_Speed();
extern void Speed_Chack_PID();
extern void FuzzyPID();
extern double FuzzyKp(int16 e, double ec);
extern double FuzzyKi(int e, double ec);
extern double FuzzyKd(int e, double ec);

#endif //__MOTOR_H__