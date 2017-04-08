#include "include.h"
#include "peripheral.h"

/*============================================
��������Stop_Car_Init()
���ã�ͣ������ʼ��
==========================================*/

/*void Stop_Car_Init()
{//����������ػ����½���
	port_init(CAR_STOP, ALT1 | IRQ_FALLING | PULLUP);			//��ʼ��ͣ���ɻɹܣ��½����ж�
	set_vector_handler(PORTE_VECTORn, Stop_Car);				//����ͣ���ж�����
	enable_irq(PORTE_IRQn);										//ʹ������
}*/

/*============================================
��������Stop_Car()
���ã�ͣ�����
==========================================*/

/*void Stop_Car()
{
	if (PORTA_ISFR & (1 << CAR_STOP_NUM))						//ȷ���жϹܽź�
	{
		PORTA_ISFR = (1 << CAR_STOP_NUM);						//����жϱ�־λ

		disable_irq(LPTMR_IRQn);								//��ֹ�͹��Ķ�ʱ�������ж�
		Right_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
		Left_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
		ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);					//������
		ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);					//������
		ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);					//������
		ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0);					//������
	}
}*/

/*============================================
��������Send_Data()
���ã��������ڷ���
==========================================*/

void Send_Data()
{
	char var[AMP_MAX];
	for (counter i = 0; i < AMP_MAX; i++)
	{
		var[i] = Road_Data[i].AD_Value_fixed;						//����λ�����͵�й�һ�����ֵ
	}
	vcan_sendware(var, sizeof(var));							//���͵���λ����ע�ⷢ��Э�飬���Ͷ˿�
	printf("Out_Speed %d %d\n ", Left_Speed.Out_Speed, Right_Speed.Out_Speed);
	printf("NowSpeed %d %d\n", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
	printf("AimSpeed %d %d\n", Left_Speed.Aim_Speed, Right_Speed.Aim_Speed);
	printf("p=%f %f\n", Left_Speed.P, Right_Speed.P);
	printf("I=%f %f\n", Left_Speed.I, Right_Speed.I);
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
	OLED_Print(15, 0, "718����ʵ����");
	OLED_Print(27, 2, "untitled��");
	OLED_Print(Position(Line3), "����ģʽ   <-");
	OLED_Print(Position(Line4), "����ģʽ");
	mode flag = 0;											//ģʽ�жϱ�־λ

	while (true)
	{
          DELAY_MS(100);
		if (gpio_get(Key1) != 0)							//ȷ�ϰ�ť
		{
			DELAY_MS(10);									//������ʱ
			if (gpio_get(Key1) != 0)
			{
                          OLED_CLS();
						  if (0 == flag % 2)
						  {
							  Debug_Init();
						  }
			}
			break;
		}

		if (gpio_get(Key4) != 0)						//ѡ��ť
		{
			DELAY_MS(10);								//������ʱ
			if (gpio_get(Key4) != 0)
			{
				flag++;
				OLED_CLS();
				OLED_Print(15, 0, "718����ʵ����");
				OLED_Print(27, 2, "untitled��");
				if (0 == flag % 2)
				{
					OLED_Print(Position(Line3), "����ģʽ   <-");
					OLED_Print(Position(Line4), "����ģʽ");
				}
				else
				{
					OLED_Print(Position(Line3), "����ģʽ");
					OLED_Print(Position(Line4), "����ģʽ   <-");
				}
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
	data temp[10];
	if (Service.flag == Inductance_Interface)
	{
		OLED_CLS();
		OLED_Print(Position(Line1), "��е���");
		sprintf(temp, "    FL=%d FR=%d", Road_Data[FRONT_LEFT].AD_Value_fixed, Road_Data[FRONT_RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "L=%d R=%d", Road_Data[LEFT].AD_Value_fixed, Road_Data[RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "M    %d", Road_Data[MIDDLE].AD_Value_fixed);
		OLED_Print(Position(Line4), temp);
	}
	else if (Service.flag == Speed_Interface)
	{
		OLED_CLS();																															//������
		sprintf(temp, "out=L%d R%d", Left_Speed.Out_Speed, Right_Speed.Out_Speed);					//����������
		OLED_Print(Position(Line1), temp);
		sprintf(temp, "now=L%d R%d", Left_Speed.Now_Speed, Right_Speed.Now_Speed);				//��ǰ�ٶ�
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "aim=L%d R%d", Left_Speed.Aim_Speed, Right_Speed.Aim_Speed);					//Ŀ���ٶ�
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "set=L%d R%d", Left_Speed.Go_Speed, Right_Speed.Go_Speed);						//�趨�ٶ�
		OLED_Print(Position(Line4), temp);
	}
	else if (Service.flag == PID_Interface)
	{
		OLED_CLS();
		sprintf(temp, "P %.2f %.2f", Left_Speed.P, Right_Speed.P);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "I %.2f %.2f", Left_Speed.I, Right_Speed.I);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "D %.1f %.1f", Left_Speed.D, Right_Speed.D);
		OLED_Print(Position(Line4), temp);
	}

	if (gpio_get(Key2) != 0)
	{
		Left_Speed.Go_Speed += 10;
		Right_Speed.Go_Speed += 10;
	}
	if (gpio_get(Key4) != 0)
	{
		Left_Speed.Go_Speed -= 10;
		Right_Speed.Go_Speed -= 10;

		if (Left_Speed.Go_Speed < 0)
			Left_Speed.Go_Speed = 0;
		if (Right_Speed.Go_Speed < 0)
			Right_Speed.Go_Speed = 0;
	}
	
	if (gpio_get(Key1) != 0)
	{
		Service.flag++;
		if (Service.flag >=MAX_Interface)
			Service.flag = 0;
	}
}

/*============================================
��������Debug_Init()
���ã�����ģʽʱ��ʼ���������
==========================================*/

void Debug_Init()
{
	Service.isDebug = true;
	Service.flag = 0;
}

/*============================================
��������System_Error(error Error_Number)
���ã�����ģʽϵͳ�������ͣ��
==========================================*/

void System_Error(error Error_Number)
{
	disable_irq(LPTMR_IRQn);									//��ֹ�͹��Ķ�ʱ�������ж�
	Right_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
	Left_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
	Motor_Control();												//���Ƶ��
	if (Error_Number == Motor_Stop)
	{
		OLED_Init();
		OLED_Print(0, 0, "�����ת");
	}

        while(1);
}

/*============================================
��������Debug()
���ã�����ģʽ��ʱ�ж�
==========================================*/

void Debug()
{
		DeBug_Interface();
		Send_Data();
}