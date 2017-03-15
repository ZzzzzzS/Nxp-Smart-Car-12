//先跑起来分支2电感
//新的差比和算法
#include "include.h"

void main()
{
	System_Init();
	Get_System_Ready();
	//OLED_Init();
	/*port_init(PTC11, ALT1 | PULLDOWN);
	port_init(PTC13, ALT1 | PULLDOWN);
	gpio_init(PTC11, GPI, 0);                       //输入按键
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
        //
	while (1)
	{
          
             
	}
        
}