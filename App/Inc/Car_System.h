#ifndef __CAR_SYSTEM_H__
#define __CAR_SYSTEM_H__

void System_Init();
void Get_System_Ready();
void  LPTMR_IRQHandler();
void Debug_Init();
void System_Error(error Error_Number);

#endif //__CAR_SYSTEM_H__
