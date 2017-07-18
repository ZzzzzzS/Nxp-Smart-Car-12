#include "include.h"


/*============================================
�������� Speed_Analyse()
���ã�ȫ�ֿ����ٶ�
==========================================*/

void Speed_Control()
{
	if (count >= 20)
	{
		Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
		count = 0;
	}
	Motor_PID();															//�Ե����������ʽPID����
	Speed_Comput();													//���뷽�򻷿���
	//Speed_Stable();														//�ٶ��˲�ʹϵͳ�ȶ�
	Speed_Chack();														//����ٶȺϷ��ԣ���ֹ��ת��
	Motor_Control();													//��������ٶ�
}

/*============================================
��������Motor_Init()
���ã���ʼ��PWMʹ�ܡ���ʼ��FTM0
==========================================*/
void Motor_Init()
{
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM_BACK, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM_BACK, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
}

/*============================================
��������Motor_Control()
����:���PWM�������Ƶ��ת��
==========================================*/

void Motor_Control()
{
	if (Service.MotorBase.AllowRun)																			//�ж��Ƿ�������ת��
	{
		if (Speed.Right.Out_Speed >= 0)
		{
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM, Speed.Right.Out_Speed);		//�����ת
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM_BACK, 0);
		}
		else if (Speed.Right.Out_Speed < 0)																//�����ת
		{
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM, 0);
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM_BACK, -Speed.Right.Out_Speed);
		}

		if (Speed.Left.Out_Speed >= 0)																		//�����ת
		{
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM, Speed.Left.Out_Speed);
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM_BACK, 0);
		}
		else if (Speed.Left.Out_Speed < 0)																//�����ת
		{
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM, 0);
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM_BACK, -Speed.Left.Out_Speed);
		}
	}
  /*ftm_pwm_init(MOTOR_FTM, LEFT_PWM, MOTOR_HZ, 50);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM_BACK, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM, MOTOR_HZ, 50);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM_BACK, MOTOR_HZ, 0);      //��ʼ�� ��� PWM*/
}

/*============================================
��������Motor_PID_Init()
����:��ʼ��PID
==========================================*/

void Motor_PID_Init()
{
	Speed.Base.Aim_Speed = 18;						//����Ĭ�ϳ�ʼ�ٶ�,�ڴ˴����ٶ���Ч

	Speed.Left.Now_Speed = Speed.Base.Aim_Speed;		//���ó�ʼ��ǰ�ٶȣ���ֹ���δ�ϵ�ʱPID�����쳣
	Speed.Right.Now_Speed = Speed.Base.Aim_Speed;	//���ó�ʼ��ǰ�ٶȣ���ֹ���δ�ϵ�ʱPID�����쳣

	Speed.Base.Error_Speed[0] = 0;					//���������س�ʼ��
	Speed.Base.Error_Speed[1] = 0;					//���������س�ʼ��
	Speed.Base.Error_Speed[2] = 0;					//���������س�ʼ��				

	Speed.Base.P = 0.05;
	Speed.Base.I = 0.02;
	Speed.Base.D = 0;

}

/*============================================
��������Motor_PID()
����:�Ե����������ʽPID����
==========================================*/

void Motor_PID()
{
	
	char I_flag = 1;										//���ֱ��������־λ
															/*****PID���ں��Ĳ���*****/
	Speed.Base.Error_Speed[Now_Error] = Speed.Base.Aim_Speed - Speed.Base.Now_Speed;

	if (Speed.Base.Error_Speed[Now_Error] > 20)																					//���ֱ�������
		I_flag = 0;
	else
		I_flag = 1;

	Speed.Base.IncrementSpeed = Speed.Base.P * Speed.Base.Error_Speed[Now_Error];																							//P
	Speed.Base.IncrementSpeed += I_flag*Speed.Base.I*(Speed.Base.Error_Speed[Now_Error] + Speed.Base.Error_Speed[last_Error]);							//I
	Speed.Base.IncrementSpeed += Speed.Base.D*(Speed.Base.Error_Speed[Now_Error] - 2 * Speed.Base.Error_Speed[last_Error] + Speed.Base.Error_Speed[lastest_Error]);	//D

	Speed.Base.Error_Speed[lastest_Error] = Speed.Base.Error_Speed[last_Error];
	Speed.Base.Error_Speed[last_Error] = Speed.Base.Error_Speed[Now_Error];

	Speed.Base.PID_Out_Speed += Speed.Base.IncrementSpeed;

	if (Speed.Base.PID_Out_Speed >= MAX_SPEED)
		Speed.Base.PID_Out_Speed = MAX_SPEED;
	else if (Speed.Base.PID_Out_Speed <= MIN_SPEED)
		Speed.Base.PID_Out_Speed = MIN_SPEED;

}

/*============================================
��������Speed_Comput()
����:��������򻷺��ٶȻ��ĺ��ٶ�
==========================================*/

void Speed_Comput()
{
	Speed.Left.Out_Speed = Speed.Left.Turn_Speed + Speed.Base.PID_Out_Speed*1.2;
	Speed.Right.Out_Speed = Speed.Right.Turn_Speed + Speed.Base.PID_Out_Speed*0.9;
}

/*============================================
�������� Get_Motor_Speed_Init()
����:��ʼ��FTM���������Ի�ȡ��ǰ�ٶ�
==========================================*/

void Get_Motor_Speed_Init()
{
	ftm_quad_init(FTM1);									//FTM1 ���������ʼ�������õĹܽſɲ� port_cfg.h ��
	ftm_quad_init(FTM2);									//FTM2 ���������ʼ��
}

/*============================================
��������Speed_Stable()
���ã����ٶȿ�������ƽ����X��������������ɣ����ӳ�ģ�ȶ���
==========================================*/
/*============================================
�˲�˵����
[0][1][2][3]
������        ������
��Ȩֵ        ��Ȩֵ
==========================================*/

void Speed_Stable()
{
	for (counter i = 0; i < Stable_Times - 1; i++)
	{
		Speed.Left.Speed_Old[i] = Speed.Left.Speed_Old[i + 1];
		Speed.Right.Speed_Old[i] = Speed.Right.Speed_Old[i + 1];
	}
	Speed.Left.Speed_Old[Stable_Times - 1] = Speed.Left.Out_Speed;
	Speed.Right.Speed_Old[Stable_Times - 1] = Speed.Right.Out_Speed;

	char sum = 0;
	Speed.Left.Out_Speed = 0;
	Speed.Right.Out_Speed = 0;
	for (counter i = 0; i < Stable_Times; i++)
	{
		sum += i;
		Speed.Left.Out_Speed += Speed.Left.Speed_Old[i] * i;
		Speed.Right.Out_Speed += Speed.Right.Speed_Old[i] * i;
	}
	Speed.Left.Out_Speed /= sum;
	Speed.Right.Out_Speed /= sum;
}

/*============================================
�������� Get_Motor_Speed()
����:FTM���������ȡ��ǰ�ٶ�
==========================================*/

void Get_Motor_Speed()
{
	Speed.Left.Now_Speed = ftm_quad_get(FTM1);				//��ȡ��������������
	Speed.Right.Now_Speed = ftm_quad_get(FTM2);				//��ȡ��������������
	
	Speed.Left.Now_Speed = -Speed.Left.Now_Speed;
	Speed.Base.Now_Speed = (Speed.Left.Now_Speed + Speed.Right.Now_Speed) / 2;//����ƽ���ٶ�

	ftm_quad_clean(FTM1);														//����������������
	ftm_quad_clean(FTM2);														//����������������
}

/*============================================
�������� Speed_Chack()
����:�ٶȺϷ��Լ�⣬��ֹ��ת��
==========================================*/

void Speed_Chack()
{
	if (Speed.Left.Out_Speed >= MAX_SPEED)					//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Speed.Left.Out_Speed = MAX_SPEED;
	}
	else if (Speed.Left.Out_Speed <= MIN_SPEED)				//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Speed.Left.Out_Speed = MIN_SPEED;
	}
	if (Speed.Right.Out_Speed >= MAX_SPEED)					//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Speed.Right.Out_Speed = MAX_SPEED;
	}
	else if (Speed.Right.Out_Speed <= MIN_SPEED)			//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Speed.Right.Out_Speed = MIN_SPEED;
	}

	if ((Speed.Right.Now_Speed < 1) && (Speed.Right.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//���ϵͳ����
	{
		if ((Speed.Left.Now_Speed < 1) && (Speed.Left.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//���ϵͳ����
		{
			//System_Error(Motor_Stop);
		}
	}
}