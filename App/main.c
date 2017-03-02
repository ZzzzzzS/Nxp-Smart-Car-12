#include "include.h"

void main()
{
	System_Init();
	Get_System_Ready();
	while (true)
	{
		if (Service.Send_Data == true)
		{
			Send_Data();
		}
	}
}