//先跑起来分支2电感
//新的差比和算法
#include "include.h"

void main()     
{

	System_Init();
	Get_System_Ready();

	while (true)
	{
		DELAY_MS(500);
		OLED_CLS();
		char temp[10];
                OLED_Print(2,0,"哈尔滨工业大学");
		sprintf(temp, "Speed=%d %d", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
		OLED_Print(2, 2, temp);
		sprintf(temp, "value=%d %d", Road_Data[LEFT].AD_Value, Road_Data[RIGHT].AD_Value);
		OLED_Print(2, 4, temp);
		sprintf(temp, "%d %d", Road_Data[LEFT].Normalized_Value, Road_Data[RIGHT].Normalized_Value);
		//OLED_Print(2, 6, temp);

	}
        
}
