#include "include.h"


/*============================================
�������� Speed_Analyse()
���ã�ȫ�ֿ����ٶ�
==========================================*/

void Speed_Control()
{
	/*****PID����1*****/
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	//FuzzyPID();															//��PID����ģ������
	PID_LearnSelf();														//����ѧϰ�Զ�����PID
	Motor_PID();															//�Ե����������ʽPID����
	Speed_Chack();														//����ٶȺϷ��ԣ���ֹ��ת��
	Motor_Control();													//��������ٶ�
}

/*============================================
��������Motor_Init()
���ã���ʼ��PWMʹ�ܡ���ʼ��FTM0
==========================================*/
void Motor_Init()
{
	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 0);      //��ʼ�� ��� PWM
}

/*============================================
��������Motor_Control()
����:���PWM�������Ƶ��ת��
==========================================*/

void Motor_Control()
{

	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, Right_Speed.Out_Speed);	//������
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);						//������
	ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, Left_Speed.Out_Speed);	//������
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0);						//������

}

/*============================================
��������Motor_PID_Init()
����:��ʼ��PID
==========================================*/

void Motor_PID_Init()
{
	Left_Speed.Go_Speed = 100;					//����Ĭ�ϳ�ʼ�ٶ�
	Right_Speed.Go_Speed = 100;				//����Ĭ�ϳ�ʼ�ٶ�
        Left_Speed.Aim_Speed=Left_Speed.Go_Speed;
        Right_Speed.Aim_Speed=Right_Speed.Go_Speed;

	Left_Speed.Now_Speed = Left_Speed.Go_Speed;		//���ó�ʼ��ǰ�ٶȣ���ֹ���δ�ϵ�ʱPID�����쳣
	Right_Speed.Now_Speed = Left_Speed.Go_Speed;	//���ó�ʼ��ǰ�ٶȣ���ֹ���δ�ϵ�ʱPID�����쳣

	Left_Speed.Error_Speed = 0;					//���������س�ʼ��
	Right_Speed.Error_Speed = 0;				//���������س�ʼ��

	Left_Speed.err_last = 0;							//���������س�ʼ��
	Right_Speed.err_last = 0;						//���������س�ʼ��

	Left_Speed.err_next = 0;						//���������س�ʼ��
	Right_Speed.err_next = 0;						//���������س�ʼ��

	Left_Speed.P = 0.3;								//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.P = 0.3;								//����ģ�����ƺ�Ҫ�������ֵ

	Left_Speed.I = 0.015;								//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.I = 0.015;							//����ģ�����ƺ�Ҫ�������ֵ

	Left_Speed.D = 0;								//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.D = 0;								//����ģ�����ƺ�Ҫ�������ֵ

}

/*============================================
��������Motor_PID()
����:�Ե����������ʽPID����
==========================================*/

void Motor_PID()
{
	flag I_flag = 1;										//���ֱ��������־λ

	if (Left_Speed.Aim_Speed < 0)			//�ж��ٶȺϷ���
	{
		Left_Speed.Aim_Speed = 0;
	}
	if (Right_Speed.Aim_Speed < 0)			//�ж��ٶȺϷ���
	{
		Right_Speed.Aim_Speed = 0;
	}
	/*****PID���ں��Ĳ���*****/
	/****���ֿ���****/
	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//ȡ������ٶ�
	if (Left_Speed.Error_Speed > 100)																					//���ֱ�������
		I_flag = 0;
	else
		I_flag = 1;

	Left_Speed.IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);
	Left_Speed.IncrementSpeed += I_flag*Left_Speed.I*Left_Speed.Error_Speed;
	Left_Speed.IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	/****���ֿ���****/
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;			//ȡ������ٶ�
	if (Right_Speed.Error_Speed > 100)																				//���ֱ�������
		I_flag = 0;
	else
		I_flag = 1;

	Right_Speed.IncrementSpeed = Right_Speed.P*(Right_Speed.Error_Speed - Right_Speed.err_next);
	Right_Speed.IncrementSpeed += I_flag*Right_Speed.I*Right_Speed.Error_Speed;
	Right_Speed.IncrementSpeed += Right_Speed.D*(Right_Speed.Error_Speed - 2 * Right_Speed.err_next + Right_Speed.err_last);

	Left_Speed.err_last = Left_Speed.err_next;
	Right_Speed.err_last = Right_Speed.err_next;

	Left_Speed.err_next = Left_Speed.Error_Speed;
	Right_Speed.err_next = Right_Speed.Error_Speed;

	Left_Speed.PID_Out_Speed += Left_Speed.IncrementSpeed;
	Right_Speed.PID_Out_Speed += Right_Speed.IncrementSpeed;


	Left_Speed.Out_Speed = Left_Speed.PID_Out_Speed;													//��������������ٶ���ʱ����pid�����ĵ�ǰ�ٶ�
	Right_Speed.Out_Speed = Right_Speed.PID_Out_Speed;												//��������������ٶ���ʱ����pid�����ĵ�ǰ�ٶ�

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
�������� Get_Motor_Speed()
����:FTM���������ȡ��ǰ�ٶ�
==========================================*/

void Get_Motor_Speed()
{
	Left_Speed.Now_Speed = ftm_quad_get(FTM2);				//��ȡ��������������
	Right_Speed.Now_Speed = ftm_quad_get(FTM1);				//��ȡ��������������

	if (Right_Speed.Now_Speed < 0)										//ȡ����ֵ
	{
		Right_Speed.Now_Speed = -Right_Speed.Now_Speed;
	}
	if (Left_Speed.Now_Speed < 0)											//ȡ����ֵ
	{
		Left_Speed.Now_Speed = -Left_Speed.Now_Speed;
	}

	ftm_quad_clean(FTM1);														//����������������
	ftm_quad_clean(FTM2);														//����������������
}

/*============================================
�������� Speed_Chack()
����:�ٶȺϷ��Լ�⣬��ֹ��ת��
==========================================*/

void Speed_Chack()
{
	if (Left_Speed.Out_Speed >= MAX_SPEED)					//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Left_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Left_Speed.Out_Speed <= MIN_SPEED)				//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Left_Speed.Out_Speed = MIN_SPEED;
	}
	if (Right_Speed.Out_Speed >= MAX_SPEED)					//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Right_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Right_Speed.Out_Speed <= MIN_SPEED)			//�ж��ٶ��Ƿ�ᳬ���߽�
	{
		Right_Speed.Out_Speed = MIN_SPEED;
	}


	if ((Right_Speed.Now_Speed < 10) && (Right_Speed.Out_Speed > 70))	//���ϵͳ����
	{
		if ((Left_Speed.Now_Speed < 10) && (Left_Speed.Out_Speed > 70))	//���ϵͳ����
		{
			//System_Error(0);
		}
	}
}

/*============================================
�������� FuzzyPID()
����:ģ������PID
==========================================*/

void FuzzyPID()
{
	Left_Speed.P = FuzzyKp(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);			//����ģ�����Ƶ�P
	Right_Speed.P = FuzzyKp(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�P

	Left_Speed.I = FuzzyKi(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);			//����ģ�����Ƶ�I
	Right_Speed.I = FuzzyKi(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�I

	Left_Speed.D = FuzzyKd(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//����ģ�����Ƶ�D
	Right_Speed.D = FuzzyKd(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�D
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

/*============================================
��������PID_LearnSelf()
����:����ѧϰ����PIDֵ
==========================================*/
/*============================================
����˵���� ���PI����    ����200�����2000
������PI������(����λ��ʽ��ѧϰ��)
���ַ���+�ֶ�PI
���ݵ�Ƭ�������������ԣ��������������Բ��ƣ��������Err_speed-Err_speed_old��
ԭ����ݵ����϶���3�棬����PI������
==========================================*/
void PID_LearnSelf()
{
	int16 x1, x2, x3;
	x1 = Left_Speed.Error_Speed;
	Left_Speed.Intergate_Speed += Left_Speed.Error_Speed;
	x2 = Left_Speed.Intergate_Speed;
	x3 = Left_Speed.err_last - Left_Speed.Error_Speed;

	if (x2 >= 300)
		x2 = 300;
	else if (x2 <= -300)
		x2 = -300;

	if (x1*x3<0)
	{
		Left_Speed.P -= 0.001;
		if (x1*x2>0)
			Left_Speed.I += 0.001;
		else if (x1*x2 < 0)
			Left_Speed.I -= 0.001;
	}
	else if (x1*x3>0)
	{
		Left_Speed.P += 0.001;
		if (x1*x2>0)
			Left_Speed.I += 0.001;
		else if (x1*x2 < 0)
			Left_Speed.I -= 0.001;
	}

	if (Left_Speed.P < 0)
		Left_Speed.P = 0;
	if (Left_Speed.I < 0)
		Left_Speed.I = 0;


	x1 = Right_Speed.Error_Speed;
	Right_Speed.Intergate_Speed += Right_Speed.Error_Speed;
	x2 = Right_Speed.Intergate_Speed;
	x3 = Right_Speed.err_last - Right_Speed.Error_Speed;

	if (x2 >= 300)
		x2 = 300;
	else if (x2 <= -300)
		x2 = -300;

	if (x1*x3<0)
	{
		Right_Speed.P -= 0.001;
		if (x1*x2>0)
			Right_Speed.I += 0.001;
		else if (x1*x2 < 0)
			Right_Speed.I -= 0.001;
	}
	else if (x1*x3>0)
	{
		Right_Speed.P += 0.001;
		if (x1*x2>0)
			Right_Speed.I += 0.001;
		else if (x1*x2 < 0)
			Right_Speed.I -= 0.001;
	}

	if (Right_Speed.P < 0)
		Right_Speed.P = 0;
	if (Right_Speed.I < 0)
		Right_Speed.I = 0;

}