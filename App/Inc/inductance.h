#ifndef __INDUCTANCE_H__
#define __INDUCTANCE_H__

extern void ADC_Init();
extern void Direction_Control();
extern void Get_AD_Value();
extern void Direction_Calculate();
extern void Direction_PID();
extern bool hasToroid();

#endif //__INDUCTANCE_H__