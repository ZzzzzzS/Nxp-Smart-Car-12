//先跑起来分支2电感
//新的差比和算法
#include "include.h"

void main()     
{

	System_Init();
	Get_System_Ready();

        //Right_Speed.Go_Speed=0;
        
	while (true)
	{
		DELAY_MS(100);
		OLED_CLS();
		char temp[10];
                OLED_Print(2,0,"哈尔滨工业大学");
		sprintf(temp, "Speed=L%d R%d", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
		OLED_Print(2, 2, temp);
		sprintf(temp, "value=L%d R%d", Road_Data[LEFT].AD_Value, Road_Data[RIGHT].AD_Value);
		OLED_Print(2, 4, temp);
		//sprintf(temp, "%d %d", Road_Data[LEFT].Normalized_Value, Road_Data[RIGHT].Normalized_Value);
                sprintf(temp, "L%d R%d", Left_Speed.Go_Speed, Right_Speed.Go_Speed);
		OLED_Print(2, 6, temp);
                
                
                
            if (gpio_get(PTC11) != 0)
			{
				//DELAY_MS(10);
				//if (gpio_get(PTC11) != 0)
				//{
					Left_Speed.Go_Speed++;
					Right_Speed.Go_Speed++;
				//}
			}
            if (gpio_get(PTC7) != 0)
			{
				//DELAY_MS(10);
				//if (gpio_get(PTC9) != 0)
				//{
					Left_Speed.Go_Speed--;
					Right_Speed.Go_Speed--;


					
				//}
			}
		
            if (Left_Speed.Go_Speed < 0)
		Left_Speed.Go_Speed = 0;
            if (Right_Speed.Go_Speed < 0)
		Right_Speed.Go_Speed = 0;
                

	}
        
}
