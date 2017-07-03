#include "include.h"


/*============================================
函数名： Speed_Analyse()
作用：全局控制速度
==========================================*/

void Speed_Control()
{
	Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
	//FuzzyPID();															//对PID参数模糊控制
	Motor_PID();															//对电机进行增量式PID调节
	Speed_Comput();													//加入方向环控制
	//Speed_Stable();														//速度滤波使系统稳定
	Speed_Chack();														//检测速度合法性，防止堵转等
	Motor_Control();													//输出最终速度
}

/*============================================
函数名：Motor_Init()
作用：初始化PWM使能。初始化FTM0
==========================================*/
void Motor_Init()
{
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM_BACK, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM_BACK, MOTOR_HZ, 0);      //初始化 电机 PWM
}

/*============================================
函数名：Motor_Control()
作用:输出PWM波，控制电机转速
==========================================*/

void Motor_Control()
{
	if (Service.MotorBase.AllowRun)																			//判断是否允许电机转动
	{
		if (Speed.Right.Out_Speed >= 0)
		{
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM, Speed.Right.Out_Speed);		//电机正转
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM_BACK, 0);
		}
		else if (Speed.Right.Out_Speed < 0)																//电机反转
		{
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM, 0);
			ftm_pwm_duty(MOTOR_FTM, RIGHT_PWM_BACK, -Speed.Right.Out_Speed);
		}

		if (Speed.Left.Out_Speed >= 0)																		//电机正转
		{
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM, Speed.Left.Out_Speed);
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM_BACK, 0);
		}
		else if (Speed.Left.Out_Speed < 0)																//电机反转
		{
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM, 0);
			ftm_pwm_duty(MOTOR_FTM, LEFT_PWM_BACK, -Speed.Left.Out_Speed);
		}
	}
}

/*============================================
函数名：Motor_PID_Init()
作用:初始化PID
==========================================*/

void Motor_PID_Init()
{
	Speed.Base.Aim_Speed = 18;						//设置默认初始速度,在此处改速度无效

	Speed.Left.Now_Speed = Speed.Base.Aim_Speed;		//设置初始当前速度，防止电机未上电时PID工作异常
	Speed.Right.Now_Speed = Speed.Base.Aim_Speed;	//设置初始当前速度，防止电机未上电时PID工作异常

	Speed.Base.Error_Speed[0] = 0;					//电机控制相关初始化
	Speed.Base.Error_Speed[1] = 0;					//电机控制相关初始化
	Speed.Base.Error_Speed[2] = 0;					//电机控制相关初始化				

	Speed.Base.P = 1;
	Speed.Base.I = 0.1;
	Speed.Base.D = 3;
}

/*============================================
函数名：Motor_PID()
作用:对电机进行增量式PID控制
==========================================*/

void Motor_PID()
{
	char I_flag = 1;										//积分变量分离标志位4
					/*****PID调节核心部分*****/
	Speed.Base.Error_Speed[Now_Error] = Speed.Base.Aim_Speed - Speed.Base.Now_Speed;

	if (Speed.Base.Error_Speed[Now_Error] > 20)																					//积分变量分离
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
函数名：Speed_Comput()
作用:计算出方向环和速度环的合速度
==========================================*/

void Speed_Comput()
{
	Speed.Left.Out_Speed = Speed.Left.Turn_Speed + Speed.Base.PID_Out_Speed;
	Speed.Right.Out_Speed = Speed.Right.Turn_Speed + Speed.Base.PID_Out_Speed;
}

/*============================================
函数名： Get_Motor_Speed_Init()
作用:初始化FTM正交解码以获取当前速度
==========================================*/

void Get_Motor_Speed_Init()
{
	ftm_quad_init(FTM1);									//FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
	ftm_quad_init(FTM2);									//FTM2 正交解码初始化
}

/*============================================
函数名：Speed_Stable()
作用：将速度控制周期平均到X个控制周期内完成，增加车模稳定性
==========================================*/
/*============================================
滤波说明：
[0][1][2][3]
旧数据        新数据
低权值        高权值
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
函数名： Get_Motor_Speed()
作用:FTM正交解码获取当前速度
==========================================*/

void Get_Motor_Speed()
{
	Speed.Left.Now_Speed = ftm_quad_get(FTM2);				//获取正交解码脉冲数
	Speed.Right.Now_Speed = ftm_quad_get(FTM1);				//获取正交解码脉冲数
        
    Speed.Left.Now_Speed=-Speed.Left.Now_Speed;                             //接线反向

	if (Service.MotorBase.GetSpeedAbs)
	{
		if (Speed.Left.Now_Speed < 0)
			Speed.Left.Now_Speed = -Speed.Left.Now_Speed;
		if (Speed.Right.Now_Speed < 0)
			Speed.Right.Now_Speed = -Speed.Right.Now_Speed;
	}
		
	Speed.Base.Now_Speed = (Speed.Left.Now_Speed + Speed.Right.Now_Speed) / 2;//计算平均速度

	ftm_quad_clean(FTM1);														//清正交解码脉冲数
	ftm_quad_clean(FTM2);														//清正交解码脉冲数
}

/*============================================
函数名： Speed_Chack()
作用:速度合法性检测，防止堵转等
==========================================*/

void Speed_Chack()
{
	if (Speed.Left.Out_Speed >= MAX_SPEED)					//判断速度是否会超过边界
	{
		Speed.Left.Out_Speed = MAX_SPEED;
	}
	else if (Speed.Left.Out_Speed <= MIN_SPEED)				//判断速度是否会超过边界
	{
		Speed.Left.Out_Speed = MIN_SPEED;
	}
	if (Speed.Right.Out_Speed >= MAX_SPEED)					//判断速度是否会超过边界
	{
		Speed.Right.Out_Speed = MAX_SPEED;
	}
	else if (Speed.Right.Out_Speed <= MIN_SPEED)			//判断速度是否会超过边界
	{
		Speed.Right.Out_Speed = MIN_SPEED;
	}

	if ((Speed.Right.Now_Speed < 3) && (Speed.Right.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//检测系统错误
	{
		if ((Speed.Left.Now_Speed < 3) && (Speed.Left.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//检测系统错误
		{
			//System_Error(Motor_Stop);
		}
	}
}

/*============================================
函数名： FuzzyPID()
作用:模糊控制PID
==========================================*/

void FuzzyPID()
{
	Speed.Base.P = FuzzyKp(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);			//计算模糊控制的P

	Speed.Base.I = FuzzyKi(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);			//计算模糊控制的I

	Speed.Base.D = FuzzyKd(Speed.Base.Error_Speed[Now_Error], Speed.Base.IncrementSpeed);		//计算模糊控制的D
}

/*============================================
函数名： FuzzyKp()
作用:模糊控制PID P值
==========================================*/

double FuzzyKp(int16 e, double ec)
{
	double Kp_calcu;																//模糊控制最终输出量
	unsigned char num, pe, pec;														//误差率，误差变化率的域
	const char  eRule[7] = { -50,-25,-10,0.0,10,25,50 };							//误差E的模糊论域
	const char  ecRule[7] = { -50,-25,-10,0.0,10,25,50 };							//误差变化率EC的模糊论域
	double eFuzzy[2] = { 0.0,0.0 };													//隶属于误差E的隶属程度
	double ecFuzzy[2] = { 0.0,0.0 };												//隶属于误差变化率EC的隶属程度
	const double  kpRule[4] = { 0.05,0.05,0.1,0.1 };								//Kp的模糊子集
	double KpFuzzy[4] = { 0.0,0.0,0.0,0.0 };										//隶属于Kp的隶属程度
	const unsigned char  KpRule[7][7] =					  							//Kp的模糊控制表
	{
		{ 3,3,3,3,3,3,3 },
		{ 2,2,2,2,1,2,2 },
		{ 1,1,1,1,1,1,1 },
		{ 1,1,0,1,0,1,1 },
		{ 0,0,1,0,0,1,0 },
		{ 0,1,0,1,0,0,2 },
		{ 3,3,3,3,3,3,3 }
	};
	/*****误差E隶属函数描述*****/
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
	/*****误差变化率EC隶属函数描述*****/
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
	/*********查询模糊规则表*********/
	num = KpRule[pe][pec];
	KpFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KpRule[pe][pec + 1];
	KpFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KpRule[pe + 1][pec];
	KpFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KpRule[pe + 1][pec + 1];
	KpFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/*********加权平均解模糊*********/
	Kp_calcu = KpFuzzy[0] * kpRule[0] + KpFuzzy[1] * kpRule[1] + KpFuzzy[2] * kpRule[2] + KpFuzzy[3] * kpRule[3];
	return Kp_calcu;
}

/*============================================
函数名： FuzzyKi()
作用:模糊控制PID I值
==========================================*/

double FuzzyKi(int e, double ec)
{	//注释参见FuzzyKp
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
	/*****误差隶属函数描述*****/
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
	/*****误差变化隶属函数描述*****/
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
	/***********查询模糊规则表***************/
	num = KiRule[pe][pec];
	KiFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KiRule[pe][pec + 1];
	KiFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KiRule[pe + 1][pec];
	KiFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KiRule[pe + 1][pec + 1];
	KiFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/********加权平均法解模糊********/
	Ki_calcu = KiFuzzy[0] * kiRule[0] + KiFuzzy[1] * kiRule[1] + KiFuzzy[2] * kiRule[2] + KiFuzzy[3] * kiRule[3];
	return Ki_calcu;
}

/*============================================
函数名： FuzzyKd()
作用:模糊控制PID D值
==========================================*/

double FuzzyKd(int e, double ec)
{	//注释参见FuzzyKp
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
	/*****误差隶属函数描述*****/
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

	/*****误差变化隶属函数描述*****/
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
	/***********查询模糊规则表*************/
	num = KdRule[pe][pec];
	KdFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
	num = KdRule[pe][pec + 1];
	KdFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
	num = KdRule[pe + 1][pec];
	KdFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
	num = KdRule[pe + 1][pec + 1];
	KdFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
	/********加权平均法解模糊********/
	Kd_calcu = KdFuzzy[0] * kdRule[0] + KdFuzzy[1] * kdRule[1] + KdFuzzy[2] * kdRule[2] + KdFuzzy[3] * kdRule[3];
	return Kd_calcu;
}