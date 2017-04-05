#include "include.h"


/*============================================
�������� Speed_Analyse()
���ã�ȫ�ֿ����ٶ�
==========================================*/

void Speed_Control()
{
	/*****PID����1*****/
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	//FuzzyPID();													//��PID����ģ������
	Motor_PID();													//�Ե����������ʽPID����
	Speed_Chack();												//����ٶȺϷ��ԣ���ֹ��ת��
	Motor_Control();												//��������ٶ�
}

/*============================================
��������Motor_Init()
���ã���ʼ��PWMʹ�ܡ���ʼ��FTM0
==========================================*/
void Motor_Init()
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
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
	Left_Speed.Go_Speed = 100;
	Right_Speed.Go_Speed = 100;

	Left_Speed.Now_Speed = 0;
	Right_Speed.Now_Speed = 0;

	Left_Speed.Error_Speed = 0;
	Right_Speed.Error_Speed = 0;

	Left_Speed.err_last = 0;
	Right_Speed.err_last = 0;

	Left_Speed.err_next = 0;
	Right_Speed.err_next = 0;

	Left_Speed.P = 0.5;										//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.P = 0.5;									//����ģ�����ƺ�Ҫ�������ֵ

	Left_Speed.I = 0.015;									//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.I = 0.015;									//����ģ�����ƺ�Ҫ�������ֵ

	Left_Speed.D = 0.2;										//����ģ�����ƺ�Ҫ�������ֵ
	Right_Speed.D = 0.2;									//����ģ�����ƺ�Ҫ�������ֵ

}

/*============================================
��������Motor_PID()
����:�Ե����������ʽPID����
==========================================*/

void Motor_PID()
{
	if (Left_Speed.Aim_Speed < 0)
	{
		Left_Speed.Aim_Speed = 0;
	}
	if (Right_Speed.Aim_Speed < 0)
	{
		Right_Speed.Aim_Speed = 0;
	}
	/*****PID���ں��Ĳ���*****/
	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//ȡ������ٶ�
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;

	Left_Speed.IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);		//����
	Left_Speed.IncrementSpeed += Left_Speed.I*Left_Speed.Error_Speed;
	Left_Speed.IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	Right_Speed.IncrementSpeed = Right_Speed.P*(Right_Speed.Error_Speed - Right_Speed.err_next);	//����
	Right_Speed.IncrementSpeed += Right_Speed.I*Right_Speed.Error_Speed;
	Right_Speed.IncrementSpeed += Right_Speed.D*(Right_Speed.Error_Speed - 2 * Right_Speed.err_next + Right_Speed.err_last);

	Left_Speed.err_last = Left_Speed.err_next;
	Right_Speed.err_last = Right_Speed.err_next;

	Left_Speed.err_next = Left_Speed.Error_Speed;
	Right_Speed.err_next = Right_Speed.Error_Speed;

	Left_Speed.PID_Out_Speed += Left_Speed.IncrementSpeed;
	Right_Speed.PID_Out_Speed += Right_Speed.IncrementSpeed;

	//ÿ�δ�Խ�������㣬���˱��ͳ���???

	Left_Speed.Out_Speed = Left_Speed.PID_Out_Speed;										//��������������ٶ���ʱ����pid�����ĵ�ǰ�ٶ�
	Right_Speed.Out_Speed = Right_Speed.PID_Out_Speed;

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

	if (Right_Speed.Now_Speed < 0)							//ȡ����ֵ
	{
		Right_Speed.Now_Speed = -Right_Speed.Now_Speed;
	}
	if (Left_Speed.Now_Speed < 0)							//ȡ����ֵ
	{
		Left_Speed.Now_Speed = -Left_Speed.Now_Speed;
	}
        
        //Right_Speed.Now_Speed*=(43/30);
       // Right_Speed.Now_Speed=(int)(Right_Speed.Now_Speed);//��Ӧ�����

	ftm_quad_clean(FTM1);									//����������������
	ftm_quad_clean(FTM2);									//����������������
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
	else if (Left_Speed.Out_Speed <= MIN_SPEED)
	{
		Left_Speed.Out_Speed = MIN_SPEED;
	}
	if (Right_Speed.Out_Speed >= MAX_SPEED)
	{
		Right_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Right_Speed.Out_Speed <= MIN_SPEED)
	{
		Right_Speed.Out_Speed = MIN_SPEED;
	}


	if ((Right_Speed.Now_Speed < 10) && (Right_Speed.Out_Speed > 70))
	{
		if ((Left_Speed.Now_Speed < 10) && (Left_Speed.Out_Speed > 70))
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
	Left_Speed.P = FuzzyKp(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//����ģ�����Ƶ�P
	Right_Speed.P = FuzzyKp(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�P

	Left_Speed.I = FuzzyKi(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//����ģ�����Ƶ�I
	Right_Speed.I = FuzzyKi(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�I

	Left_Speed.D = FuzzyKd(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//����ģ�����Ƶ�D
	Right_Speed.D = FuzzyKd(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//����ģ�����Ƶ�D
}

/*============================================
�������� FuzzyKp()
����:ģ������PID Pֵ
==========================================*/

double FuzzyKp(int16 e,double ec)
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

void PID_LearnSelf()
{

}

/*
/********************************************
����˵���� ���PI����    ����200�����2000
������PI������(����λ��ʽ��ѧϰ��)
���ַ���+�ֶ�PI
���ݵ�Ƭ�������������ԣ��������������Բ��ƣ��������Err_speed-Err_speed_old��
ԭ����ݵ����϶���3�棬����PI������

Ŀǰ��λ��ʽ��
��˫�޷�
�ο�����P109λ��ʽ����PI��������ͼ

**********************************************/
/*void PI_control(void)       //�ֿ����ƣ���һ�߶���һ����Ӱ��  
{
	//��ֵȷ�����ڳ��ϲ��ԣ���ϵͳѧϰһ��ʱ���֪��          
	Kp_left[q] = Kp_L;
	Ki_left[q] = Ki_L;
	Kp_right[q] = Kp_R;
	Ki_right[q] = Ki_R;



	x1 = Err_speed_left;                           //ת��ƫ��                          
	x2 = x2 + Err_speed_left;                        //ת��ƫ�����            
	x3 = left_speed_old - now_left_speed;            //ʵ��ת�ٱ仯�ʸ�ֵ      

	y1 = Err_speed_right;
	y2 = y2 + Err_speed_right;
	y3 = right_speed_old - now_right_speed;


	//�����޷�     С���
	/*
	if(x2>=300)  x2=300;
	else if(x2<=-300)  x2=-300;

	if(y2>=300) y2=300;
	else if(y2<=-300)  y2=-300;
	*/

	//ÿ�δ�Խ�������㣬���˱��ͳ���



	//���ַ���      ��
	/*if (abs(Err_speed_left) <= 30)     // 0.1~0.2Err
	{
		//�ֶ�PI      ��
		if (x1*x3<0)         //
		{
			Kp_left[q + 1] = Kp_left[q] - p1*0.0001;
			if (x1*x2>0)
				Ki_left[q + 1] = Ki_left[q] + i1*0.001;
			else
				Ki_left[q + 1] = Ki_left[q] - i1*0.001;
		}
		else if (x1*x3>0)
		{
			Kp_left[q + 1] = Kp_left[q] + p2*0.0001;
			if (x1*x2>0)
				Ki_left[q + 1] = Ki_left[q] + i1*0.001;
			else
				Ki_left[q + 1] = Ki_left[q] - i1*0.001;
		}
		else if (x1*x3 == 0)
		{
			Kp_left[q + 1] = Kp_left[q];
			Ki_left[q + 1] = Ki_left[q];
		}

		Kp_L = Kp_left[q + 1];
		Ki_L = Ki_left[q + 1];

		if (Kp_L<0) { Kp_L = 0; }
		if (Ki_L<0) { Ki_L = 0; }
		// if(Kp_L>13) {Kp_L=13;}
		// if(Ki_L>10) {Ki_L=10;}

	}
	else if (abs(Err_speed_left)>30)
	{
		Kp_L = 1900 / abs(x1);
		x2 = 0;
	}
	//ӦPI>=0

	//���ַ���      ��
	if (abs(Err_speed_right) <= 55)            // 0.1~0.2Err
	{
		//�ֶ�PI      ��
		if (y1*y3<0)                       //ƫ���ڼ�С
		{
			Kp_right[q + 1] = Kp_right[q] - p1*0.0001;
			if (y1*y2>0)
				Ki_right[q + 1] = Ki_right[q] + i1*0.001;
			else
				Ki_right[q + 1] = Ki_right[q] - i1*0.001;
		}
		else if (y1*y3>0)                   //ƫ��������
		{
			Kp_right[q + 1] = Kp_right[q] + p2*0.0001;
			if (y1*y2>0)
				Ki_right[q + 1] = Ki_right[q] + i1*0.001;
			else
				Ki_right[q + 1] = Ki_right[q] - i1*0.001;
		}
		else if (y1*y3 == 0)                 //��̬
		{
			Kp_right[q + 1] = Kp_right[q];
			Ki_right[q + 1] = Ki_right[q];
		}

		Kp_R = Kp_right[q + 1];
		Ki_R = Ki_right[q + 1];

		if (Kp_R<0) { Kp_R = 0; }
		if (Ki_R<0) { Ki_R = 0; }
		if (Kp_R>13) { Kp_R = 13; }
		if (Ki_R>10) { Ki_R = 10; }

	}

	else if (abs(Err_speed_right)>55)
	{
		Kp_R = 1900 / (abs(y1));    //�Զ�����P
		y2 = 0;
	}



	//˫�޷�����




	MOTOR_pwm_left = (int16)(Kp_L*x1 + Ki_L*x2);
	MOTOR_pwm_right = (int16)(Kp_R*y1 + Ki_R*y2);

	if (MOTOR_pwm_left>1900)           //����޷� ��
	{
		MOTOR_pwm_left = 1900;
	}
	else
		if (MOTOR_pwm_left<-1900)
		{
			MOTOR_pwm_left = -1900;
		}

	if (MOTOR_pwm_right>1900)          //����޷� you
	{
		MOTOR_pwm_right = 1900;
	}
	else
		if (MOTOR_pwm_right<-1900)
		{
			MOTOR_pwm_right = -1900;
		}


	//��ֵ
	if (MOTOR_pwm_left >= 0)                //��ת
	{
		MOTOR_GO_left(MOTOR_pwm_left);     //��ֵ���
	}
	if (MOTOR_pwm_left<0)                 //��ת
	{
		MOTOR_BACK_left(abs(MOTOR_pwm_left));
	}

	if (MOTOR_pwm_right >= 0)
	{
		MOTOR_GO_right(MOTOR_pwm_right);   //��ֵ�ұ�
	}
	if (MOTOR_pwm_right<0)
	{
		MOTOR_BACK_right(abs(MOTOR_pwm_right));
	}

	//ƫ��洢  x2����
	//Err_speed_old_left=Err_speed_left;
	//Err_speed_old_right=Err_speed_right;

	//ʵ���ٶȴ洢
	left_speed_old = now_left_speed;
	right_speed_old = now_right_speed;

	//ѧϰ3��   ʵ��һֱѧ
	q++;
	if (q>2)
		q = 0;
}*/
