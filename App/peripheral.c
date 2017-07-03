#include "include.h"
#include "peripheral.h"
#include "stdlib.h"
#include "string.h"
/*============================================
��������Stop_Car_Init()
���ã�ͣ������ʼ��
==========================================*/

void Stop_Car_Init()
{
	gpio_init(REED, GPI, 0);                       //�ɻɹܳ�ʼ��
}

/*============================================
��������Stop_Car()
���ã�ͣ�����
==========================================*/

void Stop_Car()
{
	if (gpio_get(REED) == 0)
	{
		System_Error(Car_Stop);
	}
}

/*============================================
��������Send_Data()
���ã��������ڷ���
==========================================*/

void Send_Data()
{
	if (Service.BlueToothBase.AllowedSendData)
	{
		char var[AMP_MAX];
		for (counter i = 0; i < AMP_MAX; i++)
		{
			var[i] = Road_Data[i].AD_Value_fixed;						//����λ�����͵�й�һ�����ֵ
		}
		vcan_sendware(var, sizeof(var));							//���͵���λ����ע�ⷢ��Э�飬���Ͷ˿�
	}
}

/*============================================
��������Receive_Data()
���ã��������ڽ���
==========================================*/

void Receive_Data()
{
	if (uart_querystr(Bluetooth, Service.BlueToothBase.ReceiveArea, 19))
	{
		led(LED1, LED_ON);
		printf("Reseive:%s", Service.BlueToothBase.ReceiveArea);
		if (hasData("S"))
		{
			System_Error(user_Stop);
		}
		else if (hasData("..."))
		{

		}
		memset(Service.BlueToothBase.ReceiveArea, 0, 20);		//���������ֹһЩ��ֵ����
	}
	else
		led(LED1, LED_OFF);
}

/*============================================
��������Key_Init()
���ã���ʼ������
==========================================*/

void Init_Key()
{
	gpio_init(Key1, GPI, 0);                       //������ʼ��
	gpio_init(Key2, GPI, 0);						//������ʼ��
	gpio_init(Key3, GPI, 0);                      //������ʼ��
	gpio_init(Key4, GPI, 0);                      //������ʼ��
	//gpio_init(key5, GPI, 0);					    //������ʼ��
}

/*============================================
��������OLED_Interface()
���ã�OLED��ʾ����ǰ����
==========================================*/

void OLED_Interface()
{
	OLED_Print(Position(Line1), "Energy Star");
	OLED_Print(Position(Line2), "inductance");
	OLED_Print(Position(Line3), "debug");
	OLED_Print(Position(Line4), "relese");
	Service.RunMode = Debug_Mode;							//ģʽ�жϱ�־λ
	OLED_Print(100, 2 * Service.RunMode, "<-");
	while (1)
	{
		OLED_Print(100, 2 * Service.RunMode, "<-");
		DELAY_MS(100);										//ˢ��ָ��λ��
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

				case Debug_Mode:
					Debug_Init();
					Service.MotorBase.AllowRun = true;
					Service.MotorBase.GetSpeedAbs = true;
					break;

				case Release_Mode:
					Service.MotorBase.AllowRun = true;
					Service.MotorBase.GetSpeedAbs = true;
					OLED_CLS();
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
				OLED_Print(Position(Line3), "debug");
				OLED_Print(Position(Line4), "relese");
				Service.RunMode++;
				if (Service.RunMode >= Max_Mode)
					Service.RunMode = inductance_Mode;
			}
		}
	}
}

/*============================================
��������DeBug_Interface()
���ã�OLED��ʾ����ʱ����
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
		sprintf(temp, "FL%dFR%d", Road_Data[FRONT_LEFT].AD_Value_fixed, Road_Data[FRONT_RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "L%dR%d", Road_Data[LEFT].AD_Value_fixed, Road_Data[RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "M%d", Road_Data[MIDDLE].AD_Value_fixed);
		OLED_Print(Position(Line4), temp);
		break;

	case Speed_Interface:
		OLED_CLS();																														//������
		sprintf(temp, "out L%dR%d", Speed.Left.Out_Speed, Speed.Right.Out_Speed);					//����������
		OLED_Print(Position(Line1), temp);
		sprintf(temp, "now L%dR%d", Speed.Left.Now_Speed, Speed.Right.Now_Speed);				//��ǰ�ٶ�
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "L%dR%d", Speed.Left.Out_Speed, Speed.Right.Out_Speed);							//Ŀ���ٶ�
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "%d %d", Speed.Base.Aim_Speed, Speed.Base.Now_Speed);																//�趨�ٶ�
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

	case Fuzzy_interface:
		OLED_CLS();
		sprintf(temp, "Fuzzy%d", (int)Fuzzy_Direction.Position.eAngle * 100);
		OLED_Print(Position(Line1), temp);
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
��������Debug_Init()
���ã�����ģʽʱ��ʼ���������
==========================================*/

void Debug_Init()
{
	Service.OLEDbase.OLED_Renew = 0;
	Service.OLEDbase.OLED_Interface = Speed_Interface;
	Service.BlueToothBase.AllowedReceiveData = true;
	Service.BlueToothBase.AllowedSendData = true;
	init_LED();
	OLED_CLS();
}

/*============================================
��������System_Error(error Error_Number)
���ã�����ģʽϵͳ�������ͣ��
==========================================*/

void System_Error(error Error_Number)
{
	disable_irq(LPTMR_IRQn);									//��ֹ�͹��Ķ�ʱ�������ж�
	Speed.Right.Out_Speed = 0;								//�����ٶ�Ϊ0
	Speed.Left.Out_Speed = 0;								//�����ٶ�Ϊ0
	Motor_Control();												//���Ƶ��

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

	default:
		OLED_Init();
		OLED_Print(Position(Line1), "Unknown Error");
		break;
	}

	while (!gpio_get(Key1));
	DELAY_MS(10);
	while (!gpio_get(Key1));
	enable_irq(LPTMR_IRQn);									//�����͹��Ķ�ʱ�������ж�
}

/*============================================
��������abs(char a)
���ã�absȡ����ֵ����
==========================================*/

char CharAbs(char a)
{
	if (a < 0)
		a = -a;
	return a;
}

/*============================================
��������init_LED()
���ã�LED��ʼ������
==========================================*/

void init_LED()
{
	led_init(LED0);
	led_init(LED1);
	led_init(LED2);
	led_init(LED3);
}

/*============================================
��������LED_Interface()
���ã�LED��ʾ����
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

	if (Fuzzy_Direction.isMatched)
	{
		led(LED2, LED_ON);
	}
	else
	{
		led(LED2, LED_OFF);
	}
}