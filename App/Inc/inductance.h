#ifndef __INDUCTANCE_H__
#define __INDUCTANCE_H__

extern void ADC_Init();
extern void Direction_Control();
extern void Get_AD_Value();
extern void Direction_Calculate();
extern void Direction_PID();
extern void Direction_Control_Fuzzy();
extern void Similarity_Count_Fuzzy();
extern void eRule_Init_Fuzzy();

#endif //__INDUCTANCE_H__