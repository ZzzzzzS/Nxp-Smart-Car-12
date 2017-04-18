#ifndef __INDUCTANCE_H__
#define __INDUCTANCE_H__

void ADC_Init();
void Direction_Control();
void Get_AD_Value();
void Direction_Calculate();
void Direction_PID();
void Direction_Control_Fuzzy();
void Similarity_Count_Fuzzy();
void eRule_Init_Fuzzy();

#endif //__INDUCTANCE_H__