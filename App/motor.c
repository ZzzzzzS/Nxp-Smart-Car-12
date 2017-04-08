#include "include.h"


/*============================================
函数名： Speed_Analyse()
作用：全局控制速度
==========================================*/

void Speed_Control()
{
	/*****PID方案1*****/
	Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
	//FuzzyPID();															//对PID参数模糊控制
	PID_LearnSelf();														//机器学习自动调节PID
	Motor_PID();															//对电机进行增量式PID调节
	Speed_Chack();														//检测速度合法性，防止堵转等
	Motor_Control();													//输出最终速度
}

/*============================================
函数名：Motor_Init()
作用：初始化PWM使能。初始化FTM0
==========================================*/
void Motor_Init()
{
	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 0);      //初始化 电机 PWM
}

/*============================================
函数名：Motor_Control()
作用:输出PWM波，控制电机转速
==========================================*/

void Motor_Control()
{

	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, Right_Speed.Out_Speed);	//电机输出
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);						//电机输出
	ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, Left_Speed.Out_Speed);	//电机输出
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0);						//电机输出

}

/*============================================
函数名：Motor_PID_Init()
作用:初始化PID
==========================================*/

void Motor_PID_Init()
{
	Left_Speed.Go_Speed = 100;					//设置默认初始速度
	Right_Speed.Go_Speed = 100;				//设置默认初始速度
        Left_Speed.Aim_Speed=Left_Speed.Go_Speed;
        Right_Speed.Aim_Speed=Right_Speed.Go_Speed;

	Left_Speed.Now_Speed = Left_Speed.Go_Speed;		//设置初始当前速度，防止电机未上电时PID工作异常
	Right_Speed.Now_Speed = Left_Speed.Go_Speed;	//设置初始当前速度，防止电机未上电时PID工作异常

	Left_Speed.Error_Speed = 0;					//电机控制相关初始化
	Right_Speed.Error_Speed = 0;				//电机控制相关初始化

	Left_Speed.err_last = 0;							//电机控制相关初始化
	Right_Speed.err_last = 0;						//电机控制相关初始化

	Left_Speed.err_next = 0;						//电机控制相关初始化
	Right_Speed.err_next = 0;						//电机控制相关初始化

	Left_Speed.P = 0.3;								//开启模糊控制后不要调节这个值
	Right_Speed.P = 0.3;								//开启模糊控制后不要调节这个值

	Left_Speed.I = 0.015;								//开启模糊控制后不要调节这个值
	Right_Speed.I = 0.015;							//开启模糊控制后不要调节这个值

	Left_Speed.D = 0;								//开启模糊控制后不要调节这个值
	Right_Speed.D = 0;								//开启模糊控制后不要调节这个值

}

/*============================================
函数名：Motor_PID()
作用:对电机进行增量式PID控制
==========================================*/

void Motor_PID()
{
	flag I_flag = 1;										//积分变量分离标志位

	if (Left_Speed.Aim_Speed < 0)			//判断速度合法性
	{
		Left_Speed.Aim_Speed = 0;
	}
	if (Right_Speed.Aim_Speed < 0)			//判断速度合法性
	{
		Right_Speed.Aim_Speed = 0;
	}
	/*****PID调节核心部分*****/
	/****左轮控制****/
	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//取得误差速度
	if (Left_Speed.Error_Speed > 100)																					//积分变量分离
		I_flag = 0;
	else
		I_flag = 1;

	Left_Speed.IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);
	Left_Speed.IncrementSpeed += I_flag*Left_Speed.I*Left_Speed.Error_Speed;
	Left_Speed.IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	/****右轮控制****/
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;			//取得误差速度
	if (Right_Speed.Error_Speed > 100)																				//积分变量分离
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


	Left_Speed.Out_Speed = Left_Speed.PID_Out_Speed;													//左右轮最终输出速度暂时等于pid处理后的当前速度
	Right_Speed.Out_Speed = Right_Speed.PID_Out_Speed;												//左右轮最终输出速度暂时等于pid处理后的当前速度

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
函数名： Get_Motor_Speed()
作用:FTM正交解码获取当前速度
==========================================*/

void Get_Motor_Speed()
{
	Left_Speed.Now_Speed = ftm_quad_get(FTM2);				//获取正交解码脉冲数
	Right_Speed.Now_Speed = ftm_quad_get(FTM1);				//获取正交解码脉冲数

	if (Right_Speed.Now_Speed < 0)										//取绝对值
	{
		Right_Speed.Now_Speed = -Right_Speed.Now_Speed;
	}
	if (Left_Speed.Now_Speed < 0)											//取绝对值
	{
		Left_Speed.Now_Speed = -Left_Speed.Now_Speed;
	}

	ftm_quad_clean(FTM1);														//清正交解码脉冲数
	ftm_quad_clean(FTM2);														//清正交解码脉冲数
}

/*============================================
函数名： Speed_Chack()
作用:速度合法性检测，防止堵转等
==========================================*/

void Speed_Chack()
{
	if (Left_Speed.Out_Speed >= MAX_SPEED)					//判断速度是否会超过边界
	{
		Left_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Left_Speed.Out_Speed <= MIN_SPEED)				//判断速度是否会超过边界
	{
		Left_Speed.Out_Speed = MIN_SPEED;
	}
	if (Right_Speed.Out_Speed >= MAX_SPEED)					//判断速度是否会超过边界
	{
		Right_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Right_Speed.Out_Speed <= MIN_SPEED)			//判断速度是否会超过边界
	{
		Right_Speed.Out_Speed = MIN_SPEED;
	}


	if ((Right_Speed.Now_Speed < 10) && (Right_Speed.Out_Speed > 70))	//检测系统错误
	{
		if ((Left_Speed.Now_Speed < 10) && (Left_Speed.Out_Speed > 70))	//检测系统错误
		{
			//System_Error(0);
		}
	}
}

/*============================================
函数名： FuzzyPID()
作用:模糊控制PID
==========================================*/

void FuzzyPID()
{
	Left_Speed.P = FuzzyKp(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);			//计算模糊控制的P
	Right_Speed.P = FuzzyKp(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的P

	Left_Speed.I = FuzzyKi(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);			//计算模糊控制的I
	Right_Speed.I = FuzzyKi(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的I

	Left_Speed.D = FuzzyKd(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//计算模糊控制的D
	Right_Speed.D = FuzzyKd(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的D
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

/*============================================
函数名：PID_LearnSelf()
作用:机器学习修正PID值
==========================================*/
/*============================================
功能说明： 电机PI调节    反馈200，输出2000
智能型PI调节器(采用位置式，学习型)
积分分离+分段PI
根据单片机及编码器特性，积分量化误差可以不计，具体根据Err_speed-Err_speed_old看
原理根据电力拖动第3版，数字PI调节器
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