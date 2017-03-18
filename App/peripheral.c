#include "include.h"
#include "peripheral.h"

/*============================================
函数名：Stop_Car_Init()
作用：停车检测初始化
==========================================*/

/*void Stop_Car_Init()
{//不清楚上升沿还是下降沿
	port_init(CAR_STOP, ALT1 | IRQ_FALLING | PULLUP);			//初始化停车干簧管，下降沿中断
	set_vector_handler(PORTE_VECTORn, Stop_Car);				//设置停车中断向量
	enable_irq(PORTE_IRQn);										//使能向量
}*/

/*============================================
函数名：Stop_Car()
作用：停车检测
==========================================*/

/*void Stop_Car()
{
	if (PORTA_ISFR & (1 << CAR_STOP_NUM))						//确定中断管脚号
	{
		PORTA_ISFR = (1 << CAR_STOP_NUM);						//清除中断标志位

		disable_irq(LPTMR_IRQn);								//禁止低功耗定时计数器中断
		Right_Speed.Out_Speed = 0;								//调整速度为0
		Left_Speed.Out_Speed = 0;								//调整速度为0
		ftm_pwm_duty(FTM0, FTM_CH1, Right_Speed.Out_Speed);		//控制左轮转动
		ftm_pwm_duty(FTM0, FTM_CH2, Left_Speed.Out_Speed);		//控制右轮转动
	}
}*/

/*============================================
函数名：Send_Data()
作用：蓝牙串口发送
==========================================*/

void Send_Data()
{
	if (Service.Debug == true)
	{
		char var[2];
		for (char i = 0; i < 2; i++)
		{
			var[i] = Road_Data[i].Normalized_Value;					//向上位机发送电感归一化后的值
		}
		vcan_sendware(var, sizeof(var));							//发送到上位机，注意发送协议，发送端口
		printf("Out_Speed %d %d ", Left_Speed.Out_Speed, Right_Speed.Out_Speed);
		printf("NowSpeed %hd %hd", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
	}
	
}

/*============================================
函数名：Save_Inductance()
作用：flash储存电感数值
==========================================*/
/*============================================
flash储存位置: L=最大值 S=最小值
偏移量:0  1  2  3  4  5  6  7
	   L0 L1 L2 L3 S0 S1 S2 S3
==========================================*/

void Save_Inductance()
{
	char i;
	//flash_erase_sector(SECTOR_NUM);								//擦除flash扇区，准备写入

	for (i = 0; i < 4; i++)										//储存电感归一化的分母
	{
		//do
		//{
			//flash_write(SECTOR_NUM, i * 4, Road_Data[i].normalization);
		//} while //(flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i].normalization);	//储存后验证是否储存正确
	}

	for (i = 4; i < 8; i++)										//储存电感最小值
	{
		//do
		//{
			//flash_write(SECTOR_NUM, i * 4, Road_Data[i-4].Min_AD_Value);
		//} while //(flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i - 4].Min_AD_Value);//储存后验证是否储存正确
	}
}

/*============================================
函数名：Save_Inductance()
作用：flash储存电感数值
==========================================*/

void load_Inductance()
{
	//储存速度信息等
}

/*============================================
函数名：Key_Init()
作用：初始化按键
==========================================*/

void Init_Key()
{
	port_init(Key1, ALT1 | PULLDOWN);
	port_init(Key2, ALT1 | PULLDOWN);
	port_init(Key3, ALT1 | PULLDOWN);
	port_init(Key4, ALT1 | PULLDOWN);
	gpio_init(Key1, GPI, 0);                       
	gpio_init(Key2, GPI, 0);
	gpio_init(Key3, GPI, 0); 
	gpio_init(Key4, GPI, 0);
}

/*============================================
函数名：OLED_Interface()
作用：OLED显示发车前界面
==========================================*/

void OLED_Interface()
{
  	
	OLED_Print(15, 0, "718创新实验室");
	OLED_Print(27, 2, "untitled组");
        OLED_Rectangle(0, 35, 127, 45, 1);
	OLED_Print(0, 6, "按键来继续...");
	while (true)
	{
		if (gpio_get(PTC13) != 0)
		{
			DELAY_MS(10);
			if (gpio_get(PTC13) != 0)
			{
                          OLED_CLS();
			}
			break;
		}
	}

	Service.Debug = true;
}