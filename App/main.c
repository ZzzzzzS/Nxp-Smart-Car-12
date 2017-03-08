//先跑起来分支
#include "include.h"

void main()
{
	System_Init();
	Get_System_Ready();
	while (true)
	{
		Send_Data();
          //Main_Control_Interrupt();
	}
}