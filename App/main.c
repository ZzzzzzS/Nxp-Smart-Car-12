//先跑起来分支2电感
//新的差比和算法
#include "include.h"

void main()     
{
	System_Init();						//系统初始化
	Get_System_Ready();			//准备发车

	while (Service.isDebug==true)
	{
		Debug();						//调试模式
	}
}
