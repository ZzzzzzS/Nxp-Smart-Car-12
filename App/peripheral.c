#include "include.h"
#include "peripheral.h"
#include "stdlib.h"
#include "string.h"
/*============================================
函数名：Stop_Car_Init()
作用：停车检测初始化
==========================================*/

void Stop_Car_Init()
{
	gpio_init(REED, GPI, 0);                       //干簧管初始化
}

/*============================================
函数名：Stop_Car()
作用：停车检测
==========================================*/

void Stop_Car()
{
	if (gpio_get(REED) == 0)
	{
		System_Error(Car_Stop);
	}
}

/*============================================
函数名：Send_Data()
作用：蓝牙串口发送
==========================================*/

void Send_Data()
{
	if (Service.BlueToothBase.AllowedSendData)
	{
		char var[AMP_MAX+2];
		for (counter i = 0; i < AMP_MAX; i++)
		{
			var[i] = Road_Data[i].AD_Value_fixed;						//向上位机发送电感归一化后的值
		}
                
                var[AMP_MAX]=Speed.Left.Now_Speed;
                var[AMP_MAX+1]=Speed.Right.Now_Speed;
                
		vcan_sendware(var, sizeof(var));							//发送到上位机，注意发送协议，发送端口
	}
}

/*============================================
函数名：Receive_Data()
作用：蓝牙串口接收
==========================================*/


void Receive_Data()
{
	if (uart_querystr(Bluetooth, Service.BlueToothBase.ReceiveArea, 19))
	{
		led(LED1, LED_ON);
		if (hasData("STOP"))
		{
			System_Error(user_Stop);
		}
		else if (hasData("SPEEDADD"))
		{
			Service.BlueToothBase.Information.speed += 5;
			printf("Current Speed:%d\n", Service.BlueToothBase.Information.speed);
		}
		else if (hasData("SPEEDCUT"))
		{
			Service.BlueToothBase.Information.speed -= 5;
			if (Service.BlueToothBase.Information.speed <= 0)
				Service.BlueToothBase.Information.speed = 0;

			printf("Current Speed:%d\n", Service.BlueToothBase.Information.speed);
		}
		else if (hasData("SENDDATA"))
		{
			if (Service.BlueToothBase.AllowedSendData == 1)
			{
				Service.BlueToothBase.AllowedSendData = 0;
				printf("AllowedSendData=false\n");
			}
			else if (Service.BlueToothBase.AllowedSendData == 0)
			{
				Service.BlueToothBase.AllowedSendData = 1;
				printf("AllowedSendData=true\n");
			}
		}
       else if(hasData("DADD"))
       {
         Service.BlueToothBase.Information.D=Service.BlueToothBase.Information.D+1;
         printf("current D:%f\n",Service.BlueToothBase.Information.D);
       }
       else if(hasData("DCUT"))
       {
         Service.BlueToothBase.Information.D=Service.BlueToothBase.Information.D-1;
         printf("current D:%f\n",Service.BlueToothBase.Information.D);
       }
		else if(hasData("PADD"))
       {
         Service.BlueToothBase.Information.P=Service.BlueToothBase.Information.P+0.2;
         printf("current P:%f\n",Service.BlueToothBase.Information.P);
       }
       else if(hasData("PCUT"))
       {
         Service.BlueToothBase.Information.P=Service.BlueToothBase.Information.P-0.2;
         printf("current P:%f\n",Service.BlueToothBase.Information.P);
       }
       else
       {
         printf("ERROR COMMAND\n");
       }
                
		memset(Service.BlueToothBase.ReceiveArea, 0, 20);		//清零数组防止一些奇怪的情况
	}
	else
		led(LED1, LED_OFF);
}

/*============================================
函数名：Key_Init()
作用：初始化按键
==========================================*/

void Init_Key()
{
	gpio_init(Key1, GPI, 0);                       //按键初始化
	gpio_init(Key2, GPI, 0);						//按键初始化
	gpio_init(Key3, GPI, 0);                      //按键初始化
	gpio_init(Key4, GPI, 0);                      //按键初始化
	//gpio_init(key5, GPI, 0);					    //按键初始化
}

/*============================================
函数名：OLED_Interface()
作用：OLED显示发车前界面
==========================================*/

void OLED_Interface()
{
	OLED_Print(Position(Line1), "Energy Star");
	OLED_Print(Position(Line2), "inductance");
	OLED_Print(Position(Line3), "fast");
	OLED_Print(Position(Line4), "slow");
	Service.RunMode = FastMode;							//模式判断标志位
	OLED_Print(100, 2 * Service.RunMode, "<-");
	while (1)
	{
		OLED_Print(100, 2 * Service.RunMode, "<-");
		DELAY_MS(100);										//刷新指针位置
		if (gpio_get(Key1) != 0)
		{
			DELAY_MS(10);
			if (gpio_get(Key1) != 0)
			{
				OLED_CLS();

				switch (Service.RunMode)
				{
				case inductance_Mode:
					Debug_Init();
					Service.MotorBase.AllowRun = false;
					Service.MotorBase.GetSpeedAbs = true;
					break;

				case FastMode:
					Debug_Init();
					Service.MotorBase.AllowRun = true;
					Service.MotorBase.GetSpeedAbs = true;
					break;

				case SlowMode:
					Debug_Init();
					Service.MotorBase.AllowRun = true;
					Service.MotorBase.GetSpeedAbs = true;
					break;

				default:
					System_Error(No_Mode);
					break;
				}
				break;
			}
		}

		if (gpio_get(Key4) != 0)
		{
			DELAY_MS(10);
			if (gpio_get(Key4) != 0)
			{
				OLED_CLS();
				OLED_Print(Position(Line1), "Energy Star");
				OLED_Print(Position(Line2), "inductance");
				OLED_Print(Position(Line3), "fast");
				OLED_Print(Position(Line4), "slow");
				Service.RunMode++;
				if (Service.RunMode >= Max_Mode)
					Service.RunMode = inductance_Mode;
			}
		}
	}
}

/*============================================
函数名：DeBug_Interface()
作用：OLED显示调试时界面
==========================================*/

void DeBug_Interface()
{
  OLED_CLS();
	char temp[10];
	switch (Service.OLEDbase.OLED_Interface)
	{
	case Inductance_Interface:
		OLED_CLS();
		OLED_Print(Position(Line1), "inductance");
		sprintf(temp, "F%d", Road_Data[FRONT].AD_Value_fixed);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "L%dR%d", Road_Data[LEFT].AD_Value_fixed, Road_Data[RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "M%d", Road_Data[MIDDLE].AD_Value_fixed);
		OLED_Print(Position(Line4), temp);
		break;

	case Speed_Interface:
		OLED_CLS();																														//先清屏
		sprintf(temp, "out L%dR%d", Speed.Left.Out_Speed, Speed.Right.Out_Speed);					//电机输出功率
		OLED_Print(Position(Line1), temp);
		sprintf(temp, "now L%dR%d", Speed.Left.Now_Speed, Speed.Right.Now_Speed);				//当前速度
		OLED_Print(Position(Line2), temp);
		/*sprintf(temp, "L%dR%d", Speed.Left.Out_Speed, Speed.Right.Out_Speed);							//目标速度
		OLED_Print(Position(Line3), temp);*/
		sprintf(temp, "%d %d", Speed.Base.Aim_Speed, Speed.Base.Now_Speed);																//设定速度
		OLED_Print(Position(Line4), temp);

		if (gpio_get(Key2) != 0)
		{
			Speed.Base.Aim_Speed += 2;
		}
		if (gpio_get(Key4) != 0)
		{
			if (Speed.Base.Aim_Speed<5)
				Speed.Base.Aim_Speed = 1;
			else
				Speed.Base.Aim_Speed -= 2;
		}
		break;

	case Direction_Interface:
		OLED_CLS();
		sprintf(temp, "out%d", Direction.PIDbase.PID_Out_Speed);
		OLED_Print(Position(Line1), temp);
		sprintf(temp, "err%d", Direction.err);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "Fix%d", Direction.PIDbase.Error_Speed[Now_Error]);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "%d %d %d", Direction.sum[0], Direction.sum[1], Direction.sum[2]);
		OLED_Print(Position(Line4), temp);
		break;

	default:
		sprintf(temp, "No Interface");
		OLED_Print(Position(Line1), temp);
		break;
	}

	if (gpio_get(Key1) != 0)
	{
		DELAY_MS(10);
		if (gpio_get(Key1) != 0)
		{
			Service.OLEDbase.OLED_Interface++;
			if (Service.OLEDbase.OLED_Interface >= MAX_Interface)
				Service.OLEDbase.OLED_Interface = Inductance_Interface;
		}
	}
}

/*============================================
函数名：Debug_Init()
作用：调试模式时初始化调试组件
==========================================*/

void Debug_Init()
{
	Service.OLEDbase.OLED_Renew = 0;
	Service.OLEDbase.OLED_Interface = Inductance_Interface;
	Service.BlueToothBase.AllowedReceiveData = true;
	Service.BlueToothBase.AllowedSendData = true;
	init_LED();
	OLED_CLS();
}

/*============================================
函数名：System_Error(error Error_Number)
作用：调试模式系统错误紧急停车
==========================================*/

void System_Error(error Error_Number)
{
	disable_irq(LPTMR_IRQn);									//禁止低功耗定时计数器中断
	Speed.Right.Out_Speed = 0;								//调整速度为0
	Speed.Left.Out_Speed = 0;								//调整速度为0
	Motor_Control();												//控制电机

	switch (Error_Number)
	{
	case Motor_Stop:
		OLED_Init();
		OLED_Print(Position(Line1), "motor error");
		break;

	case Taget_Lost:
		OLED_Init();
		OLED_Print(Position(Line1), "Taget Lost");
		Service.InductanceBase.InductanceLost = 0;
		break;

	case Car_Stop:
		OLED_Init();
		OLED_Print(Position(Line1), "terminal point");
		break;

	case No_Mode:
		OLED_Init();
		OLED_Print(Position(Line1), "Mode Error");
		break;

	case user_Stop:
		OLED_Init();
		OLED_Print(Position(Line1), "user stop");
		OLED_Print(Position(Line2), "the Car");
		break;

	case hardfault:
		OLED_Init();
		OLED_Print(Position(Line1), "hardfault");
		OLED_Print(Position(Line2), "error");
		OLED_Print(Position(Line3), "restart system");
		break;

	default:
		OLED_Init();
		OLED_Print(Position(Line1), "Unknown Error");
		break;
	}

	while (!gpio_get(Key1));
	DELAY_MS(10);
	while (!gpio_get(Key1));
	enable_irq(LPTMR_IRQn);									//开启低功耗定时计数器中断
}

/*============================================
函数名：abs(char a)
作用：abs取绝对值函数
==========================================*/

char CharAbs(char a)
{
	if (a < 0)
		a = -a;
	return a;
}

/*============================================
函数名：init_LED()
作用：LED初始化函数
==========================================*/

void init_LED()
{
	led_init(LED0);
	led_init(LED1);
	led_init(LED2);
	led_init(LED3);
}

/*============================================
函数名：LED_Interface()
作用：LED显示函数
==========================================*/

void LED_Interface()
{
	if (Speed.Left.Out_Speed > Speed.Right.Out_Speed)
	{
		led(LED3, LED_ON);
		led(LED0, LED_OFF);
	}
	else if (Speed.Right.Out_Speed > Speed.Left.Out_Speed)
	{
		led(LED3, LED_OFF);
		led(LED0, LED_ON);
	}
}