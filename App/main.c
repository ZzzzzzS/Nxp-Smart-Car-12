#include "include.h"

void main()     
{
	System_Init();						//系统初始化
	Get_System_Ready();			//准备发车
	while (Service.isDebug)
	{
		DELAY_MS(20);
			Debug();								//调试模式
	}
}
