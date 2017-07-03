#include "include.h"


/*============================================
�������� Speed_Analyse()
���ã�ȫ�ֿ����ٶ�
==========================================*/

void Speed_Control()
{
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	//FuzzyPID();															//��PID����ģ������
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

	Speed.Base.P = 1;
	Speed.Base.I = 0.1;
	Speed.Base.D = 3;
}

/*============================================
��������Motor_PID()
����:�Ե����������ʽPID����
==========================================*/

void Motor_PID()
{
	char I_flag = 1;										//���ֱ��������־λ4
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
	Speed.Left.Out_Speed = Speed.Left.Turn_Speed + Speed.Base.PID_Out_Speed;
	Speed.Right.Out_Speed = Speed.Right.Turn_Speed + Speed.Base.PID_Out_Speed;
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
	Speed.Left.Now_Speed = ftm_quad_get(FTM2);				//��ȡ��������������
	Speed.Right.Now_Speed = ftm_quad_get(FTM1);				//��ȡ��������������
        
    Speed.Left.Now_Speed=-Speed.Left.Now_Speed;                             //���߷���

	if (Service.MotorBase.GetSpeedAbs)
	{
		if (Speed.Left.Now_Speed < 0)
			Speed.Left.Now_Speed = -Speed.Left.Now_Speed;
		if (Speed.Right.Now_Speed < 0)
			Speed.Right.Now_Speed = -Speed.Right.Now_Speed;
	}
		
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

	if ((Speed.Right.Now_Speed < 3) && (Speed.Right.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//���ϵͳ����
	{
		if ((Speed.Left.Now_Speed < 3) && (Speed.Left.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//���ϵͳ����
		{
			//System_Error(Motor_Stop);
		}
	}
}

/*============================================
�������� FuzzyPID()
����:ģ������PID
==========================================*/

void FuzzyPID()
{
	Speed.Base.P = FuzzyKp(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);			//����ģ�����Ƶ�P

	Speed.Base.I = FuzzyKi(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);			//����ģ�����Ƶ�I

	Speed.Base.D = FuzzyKd(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);		//����ģ�����Ƶ�D
}

/*============================================
�������� FuzzyKp()
����:ģ������PID Pֵ
==========================================*/

double FuzzyKp(int16 e, double ec)
{
	double Kp_calcu;																//ģ���������������
	unsigned char num, pe, pec;														//����ʣ����仯�ʵ���
	const char  eRule[7] = { -50,-25,-10,0.0,10,25,50 };							//���E��ģ������
	const char  ecRule[7] = { -50,-25,-10,0.0,10,25,50 };							//���仯��EC��ģ������
	double eFuzzy[2] = { 0.0,0.0 };													//���������E�������̶�
	double ecFuzzy[2] = { 0.0,0.0 };												//���������仯��EC�������̶�
	const double  kpRule[4] = { 0.05,0.05,0.1,0.1 };								//Kp��ģ���Ӽ�
	double KpFuzzy[4] = { 0.0,0.0,0.0,0.0 };										//������Kp�������̶�
	const unsigned char  KpRule[7][7] =					  							//Kp��ģ�����Ʊ�
	{
		{ 3,3,3,3,3,3,3 },
		{ 2,2,2,2,1,2,2 },
		{ 1,1,1,1,1,1,1 },
		{ 1,1,0,1,0,1,1 },
		{ 0,0,1,0,0,1,0 },
		{ 0,1,0,1,0,0,2 },
		{ 3,3,3,3,3,3,3 }
	};
	/*****���E������������*****/
	if (e<eRule[0])
	{
		eFuzzy[0] = 1.0;
		pe = 0;
	}
	else if (eRule[0] <= e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1] - e) / (eRule[1] - eRule[0]);
		pe = 0;
	}
	else if (eRule[1] <= e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] - e) / (eRule[2] - eRule[1]);
		pe = 1;
	}
	else if (eRule[2] <= e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] - e) / (eRule[3] - eRule[2]);
		pe = 2;
	}
	else if (eRule[3] <= e && e<eRule[4])
	{
		eFuzzy[0] = (eRule[4] - e) / (eRule[4] - eRule[3]);
		pe = 3;
	}
	else if (eRule[4] <= e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5] - e) / (eRule[5] - eRule[4]);
		pe = 4;
	}
	else if (eRule[5] <= e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6] - e) / (eRule[6] - eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] = 0.0;
		pe = 5;
	}
	eFuzzy[1] = 1.0 - eFuzzy[0];
	/*****���仯��EC������������*****/
	if (ec<ecRule[0])
	{
		ecFuzzy[0] = 1.0;
		pec = 0;
	}
	else if (ecRule[0] <= ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec) / (ecRule[1] - ecRule[0]);
		pec = 0;
	}
	else if (ecRule[1] <= ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec) / (ecRule[2] - ecRule[1]);
		pec = 1;
	}
	else if (ecRule[2] <= ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec) / (ecRule[3] - ecRule[2]);
		pec = 2;
	}
	else if (ecRule[3] <= ec && ec<ecRule[4])
	{
		ecFuzzy[0] = (ecRule[4] - ec) / (ecRule[4] - ecRule[3]);
		pec = 3;
	}
	else if (ecRule[4] <= ec && ec<ecRule[5])
	{
		ecFuzzy[0] = (ecRule[5] - ec) / (ecRule[5] - ecRule[4]);
		pec = 4;
	}
	else if (ecRule[5] <= ec && ec<ecRule[6])
	{
		ecFuzzy[0] = (ecRule[6] - ec) / (ecRule[6] - ecRule[5]);
		pec = 5;
	}
	else
	{
		ecFuzzy[0] = 0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
	/*********��ѯģ�������*********/
	num = KpRule[pe][pec];
	KpFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KpRule[pe][pec + 1];
	KpFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KpRule[pe + 1][pec];
	KpFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KpRule[pe + 1][pec + 1];
	KpFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/*********��Ȩƽ����ģ��*********/
	Kp_calcu = KpFuzzy[0] * kpRule[0] + KpFuzzy[1] * kpRule[1] + KpFuzzy[2] * kpRule[2] + KpFuzzy[3] * kpRule[3];
	return Kp_calcu;
}

/*============================================
�������� FuzzyKi()
����:ģ������PID Iֵ
==========================================*/

double FuzzyKi(int e, double ec)
{	//ע�Ͳμ�FuzzyKp
	double Ki_calcu;
	unsigned char num, pe, pec;
	const char eRule[7] = { -50,-25,-10,0.0,10,25,50 };
	const char ecRule[7] = { -50,-25,-10,0.0,10,25,50 };
	double eFuzzy[2] = { 0.0,0.0 };
	double ecFuzzy[2] = { 0.0,0.0 };
	const double kiRule[4] = { 0.00,0.01,0.02,0.03 };
	double KiFuzzy[4] = { 0.0,0.0,0.0,0.0 };
	const unsigned char KiRule[7][7] =
	{
		/*{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{2,0,0,0,0,0,1},
		{3,3,3,3,3,3,3}*/
		{ 3,3,3,2,2,2,2 },
		{ 2,2,2,1,1,1,1 },
		{ 1,1,2,1,1,2,1 },
		{ 1,1,0,1,0,1,1 },
		{ 1,1,0,0,0,1,1 },
		{ 2,2,1,0 ,1,1,1 },
		{ 3,3,3,3,2,3,2 }
	};
	/*****���������������*****/
	if (e<eRule[0])
	{
		eFuzzy[0] = 1.0;
		pe = 0;
	}
	else if (eRule[0] <= e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1] - e) / (eRule[1] - eRule[0]);
		pe = 0;
	}
	else if (eRule[1] <= e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] - e) / (eRule[2] - eRule[1]);
		pe = 1;
	}
	else if (eRule[2] <= e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] - e) / (eRule[3] - eRule[2]);
		pe = 2;
	}
	else if (eRule[3] <= e && e<eRule[4])
	{
		eFuzzy[0] = (eRule[4] - e) / (eRule[4] - eRule[3]);
		pe = 3;
	}
	else if (eRule[4] <= e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5] - e) / (eRule[5] - eRule[4]);
		pe = 4;
	}
	else if (eRule[5] <= e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6] - e) / (eRule[6] - eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] = 0.0;
		pe = 5;
	}
	eFuzzy[1] = 1.0 - eFuzzy[0];
	/*****���仯������������*****/
	if (ec<ecRule[0])
	{
		ecFuzzy[0] = 1.0;
		pec = 0;
	}
	else if (ecRule[0] <= ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec) / (ecRule[1] - ecRule[0]);
		pec = 0;
	}
	else if (ecRule[1] <= ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec) / (ecRule[2] - ecRule[1]);
		pec = 1;
	}
	else if (ecRule[2] <= ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec) / (ecRule[3] - ecRule[2]);
		pec = 2;
	}
	else if (ecRule[3] <= ec && ec<ecRule[4])
	{
		ecFuzzy[0] = (ecRule[4] - ec) / (ecRule[4] - ecRule[3]);
		pec = 3;
	}
	else if (ecRule[4] <= ec && ec<ecRule[5])
	{
		ecFuzzy[0] = (ecRule[5] - ec) / (ecRule[5] - ecRule[4]);
		pec = 4;
	}
	else if (ecRule[5] <= ec && ec<ecRule[6])
	{
		ecFuzzy[0] = (ecRule[6] - ec) / (ecRule[6] - ecRule[5]);
		pec = 5;
	}
	else
	{
		ecFuzzy[0] = 0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
	/***********��ѯģ�������***************/
	num = KiRule[pe][pec];
	KiFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KiRule[pe][pec + 1];
	KiFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KiRule[pe + 1][pec];
	KiFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KiRule[pe + 1][pec + 1];
	KiFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/********��Ȩƽ������ģ��********/
	Ki_calcu = KiFuzzy[0] * kiRule[0] + KiFuzzy[1] * kiRule[1] + KiFuzzy[2] * kiRule[2] + KiFuzzy[3] * kiRule[3];
	return Ki_calcu;
}

/*============================================
�������� FuzzyKd()
����:ģ������PID Dֵ
==========================================*/

double FuzzyKd(int e, double ec)
{	//ע�Ͳμ�FuzzyKp
	double Kd_calcu;
	unsigned char num, pe, pec;
	const char eRule[7] = { -50,-25,-10,0.0,10,25,50 };
	const char ecRule[7] = { -50,-25,-10,0.0,10,25,50 };
	double eFuzzy[2] = { 0.0,0.0 };
	double ecFuzzy[2] = { 0.0,0.0 };
	const double kdRule[4] = { 0.05,0.1,0.15,0.2 };
	double KdFuzzy[4] = { 0.0,0.0,0.0,0.0 };
	const unsigned char KdRule[7][7] =
	{
		{ 3,3,3,2,2,2,2 },
		{ 2,2,2,1,1,1,1 },
		{ 1,1,2,1,1,2,1 },
		{ 1,1,0,1,0,1,1 },
		{ 1,1,0,0,0,1,1 },
		{ 2,2,1,0 ,1,1,1 },
		{ 3,3,3,3,2,3,2 }
	};
	/*****���������������*****/
	if (e<eRule[0])
	{
		eFuzzy[0] = 1.0;
		pe = 0;
	}
	else if (eRule[0] <= e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1] - e) / (eRule[1] - eRule[0]);
		pe = 0;
	}
	else if (eRule[1] <= e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] - e) / (eRule[2] - eRule[1]);
		pe = 1;
	}
	else if (eRule[2] <= e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] - e) / (eRule[3] - eRule[2]);
		pe = 2;
	}
	else if (eRule[3] <= e && e<eRule[4])
	{
		eFuzzy[0] = (eRule[4] - e) / (eRule[4] - eRule[3]);
		pe = 3;
	}
	else if (eRule[4] <= e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5] - e) / (eRule[5] - eRule[4]);
		pe = 4;
	}
	else if (eRule[5] <= e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6] - e) / (eRule[6] - eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] = 0.0;
		pe = 5;
	}
	eFuzzy[1] = 1.0 - eFuzzy[0];

	/*****���仯������������*****/
	if (ec<ecRule[0])
	{
		ecFuzzy[0] = 1.0;
		pec = 0;
	}
	else if (ecRule[0] <= ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec) / (ecRule[1] - ecRule[0]);
		pec = 0;
	}
	else if (ecRule[1] <= ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec) / (ecRule[2] - ecRule[1]);
		pec = 1;
	}
	else if (ecRule[2] <= ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec) / (ecRule[3] - ecRule[2]);
		pec = 2;
	}
	else if (ecRule[3] <= ec && ec<ecRule[4])
	{
		ecFuzzy[0] = (ecRule[4] - ec) / (ecRule[4] - ecRule[3]);
		pec = 3;
	}
	else if (ecRule[4] <= ec && ec<ecRule[5])
	{
		ecFuzzy[0] = (ecRule[5] - ec) / (ecRule[5] - ecRule[4]);
		pec = 4;
	}
	else if (ecRule[5] <= ec && ec<ecRule[6])
	{
		ecFuzzy[0] = (ecRule[6] - ec) / (ecRule[6] - ecRule[5]);
		pec = 5;
	}
	else
	{
		ecFuzzy[0] = 0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
	/***********��ѯģ�������*************/
	num = KdRule[pe][pec];
	KdFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KdRule[pe][pec + 1];
	KdFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KdRule[pe + 1][pec];
	KdFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KdRule[pe + 1][pec + 1];
	KdFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/********��Ȩƽ������ģ��********/
	Kd_calcu = KdFuzzy[0] * kdRule[0] + KdFuzzy[1] * kdRule[1] + KdFuzzy[2] * kdRule[2] + KdFuzzy[3] * kdRule[3];
	return Kd_calcu;
}