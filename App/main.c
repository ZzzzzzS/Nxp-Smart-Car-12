//����������֧2���
//�µĲ�Ⱥ��㷨
#include "include.h"

void main()     
{
	System_Init();
	Get_System_Ready();

	while (true)
	{
		DELAY_MS(1000);
		OLED_CLS();
		char temp[10];
                OLED_Print(2,0,"��������ҵ��ѧ");
		sprintf(temp, "Speed=%d %d", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
		OLED_Print(2, 2, temp);
		sprintf(temp, "value=%d %d", Road_Data[LEFT].AD_Value, Road_Data[RIGHT].AD_Value);
		OLED_Print(2, 4, temp);
		sprintf(temp, "%d %d", Road_Data[LEFT].Normalized_Value, Road_Data[RIGHT].Normalized_Value);
		//OLED_Print(2, 6, temp);

	}


	//OLED_Init();
	/*port_init(PTC11, ALT1 | PULLDOWN);
	port_init(PTC13, ALT1 | PULLDOWN);
	gpio_init(PTC11, GPI, 0);                       //���밴��
	gpio_init(PTC13, GPI, 0);
	while(1)
        {
          
			if (gpio_get(PTC11) != 0)
			{
				DELAY_MS(10);
				if (gpio_get(PTC11) != 0)
				{
                                        OLED_CLS();
					OLED_Print(2, 2, "1");
				}
				while (gpio_get(PTC11) != 0);
			}
			else if (gpio_get(PTC13) != 0)
			{
				DELAY_MS(10);
				if (gpio_get(PTC13) != 0)
				{
                                  OLED_CLS();
					OLED_Print(2, 2, "2");
				}
				while (gpio_get(PTC13) != 0);
			}
                        

	}*/
        
}
