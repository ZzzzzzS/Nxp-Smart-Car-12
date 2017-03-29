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
                sprintf(temp, "Out=L%d R%d", Left_Speed.Out_Speed, Right_Speed.Out_Speed);
               // OLED_Print(2, 0, temp);
		sprintf(temp, "  L%d R%d", Road_Data[4].AD_Value, Road_Data[3].AD_Value);
		//OLED_Print(0, 2, temp);
		sprintf(temp, "L%d M%d R%d", Road_Data[LEFT].AD_Value,0, Road_Data[RIGHT].AD_Value);
		OLED_Print(0, 4, temp);
        //sprintf(temp, "Aim=L%d R%d", Left_Speed.Aim_Speed, Right_Speed.Aim_Speed);
		//OLED_Print(2, 6, temp);
                
                
                
            if (gpio_get(PTC11) != 0)
			{
					Left_Speed.Go_Speed+=10;
					Right_Speed.Go_Speed+=10;
			}
            if (gpio_get(PTC7) != 0)
			{
					Left_Speed.Go_Speed-=10;
					Right_Speed.Go_Speed-=10;
			}
		
            if (Left_Speed.Go_Speed < 0)
		Left_Speed.Go_Speed = 0;
            if (Right_Speed.Go_Speed < 0)
		Right_Speed.Go_Speed = 0;
                

	}
        
}
