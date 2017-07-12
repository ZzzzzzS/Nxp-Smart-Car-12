#ifndef __CAR_SYSTEM_H__
#define __CAR_SYSTEM_H__

extern void Init_System();
extern void Set_User_Information();
extern void Get_System_Ready();
extern void  LPTMR_IRQHandler();
extern void Debug_Init();
extern void System_Error(error Error_Number);
extern void system_RunTime_Update();

#endif //__CAR_SYSTEM_H__
