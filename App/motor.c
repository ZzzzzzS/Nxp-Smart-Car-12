#include "include.h"


/*============================================
函数名： Speed_Analyse()
作用：全局控制速度
==========================================*/

void Speed_Control()
{
	/*****PID方案1*****/
	Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
	//FuzzyPID();													//对PID参数模糊控制
	Motor_PID();													//对电机进行增量式PID调节
	Speed_Chack();												//检测速度合法性，防止堵转等
	Motor_Control();												//输出最终速度
}

/*============================================
函数名：Motor_Init()
作用：初始化PWM使能。初始化FTM0
==========================================*/
void Motor_Init()
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
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

	Left_Speed.P = 0.5;										//开启模糊控制后不要调节这个值
	Right_Speed.P = 0.5;									//开启模糊控制后不要调节这个值

	Left_Speed.I = 0.015;									//开启模糊控制后不要调节这个值
	Right_Speed.I = 0.015;									//开启模糊控制后不要调节这个值

	Left_Speed.D = 0.2;										//开启模糊控制后不要调节这个值
	Right_Speed.D = 0.2;									//开启模糊控制后不要调节这个值

}

/*============================================
函数名：Motor_PID()
作用:对电机进行增量式PID控制
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
	/*****PID调节核心部分*****/
	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//取得误差速度
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;

	Left_Speed.IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);		//左轮
	Left_Speed.IncrementSpeed += Left_Speed.I*Left_Speed.Error_Speed;
	Left_Speed.IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	Right_Speed.IncrementSpeed = Right_Speed.P*(Right_Speed.Error_Speed - Right_Speed.err_next);	//右轮
	Right_Speed.IncrementSpeed += Right_Speed.I*Right_Speed.Error_Speed;
	Right_Speed.IncrementSpeed += Right_Speed.D*(Right_Speed.Error_Speed - 2 * Right_Speed.err_next + Right_Speed.err_last);

	Left_Speed.err_last = Left_Speed.err_next;
	Right_Speed.err_last = Right_Speed.err_next;

	Left_Speed.err_next = Left_Speed.Error_Speed;
	Right_Speed.err_next = Right_Speed.Error_Speed;

	Left_Speed.PID_Out_Speed += Left_Speed.IncrementSpeed;
	Right_Speed.PID_Out_Speed += Right_Speed.IncrementSpeed;

	//每次穿越积分清零，防退饱和超调???

	Left_Speed.Out_Speed = Left_Speed.PID_Out_Speed;										//左右轮最终输出速度暂时等于pid处理后的当前速度
	Right_Speed.Out_Speed = Right_Speed.PID_Out_Speed;

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

	if (Right_Speed.Now_Speed < 0)							//取绝对值
	{
		Right_Speed.Now_Speed = -Right_Speed.Now_Speed;
	}
	if (Left_Speed.Now_Speed < 0)							//取绝对值
	{
		Left_Speed.Now_Speed = -Left_Speed.Now_Speed;
	}
        
        //Right_Speed.Now_Speed*=(43/30);
       // Right_Speed.Now_Speed=(int)(Right_Speed.Now_Speed);//适应大齿轮

	ftm_quad_clean(FTM1);									//清正交解码脉冲数
	ftm_quad_clean(FTM2);									//清正交解码脉冲数
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
函数名： FuzzyPID()
作用:模糊控制PID
==========================================*/

void FuzzyPID()
{
	Left_Speed.P = FuzzyKp(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//计算模糊控制的P
	Right_Speed.P = FuzzyKp(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的P

	Left_Speed.I = FuzzyKi(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//计算模糊控制的I
	Right_Speed.I = FuzzyKi(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的I

	Left_Speed.D = FuzzyKd(Left_Speed.Error_Speed, Left_Speed.IncrementSpeed);		//计算模糊控制的D
	Right_Speed.D = FuzzyKd(Right_Speed.Error_Speed, Right_Speed.IncrementSpeed);	//计算模糊控制的D
}

/*============================================
函数名： FuzzyKp()
作用:模糊控制PID P值
==========================================*/

double FuzzyKp(int16 e,double ec)
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
作用:自主学习修正PID值
==========================================*/

void PID_LearnSelf()
{

}

/*
/********************************************
功能说明： 电机PI调节    反馈200，输出2000
智能型PI调节器(采用位置式，学习型)
积分分离+分段PI
根据单片机及编码器特性，积分量化误差可以不计，具体根据Err_speed-Err_speed_old看
原理根据电力拖动第3版，数字PI调节器

目前：位置式，
需双限幅
参考电拖P109位置式数字PI调节器框图

**********************************************/
/*void PI_control(void)       //分开控制，调一边对另一边无影响  
{
	//初值确定，在车上测试，等系统学习一段时间可知道          
	Kp_left[q] = Kp_L;
	Ki_left[q] = Ki_L;
	Kp_right[q] = Kp_R;
	Ki_right[q] = Ki_R;



	x1 = Err_speed_left;                           //转速偏差                          
	x2 = x2 + Err_speed_left;                        //转速偏差积分            
	x3 = left_speed_old - now_left_speed;            //实际转速变化率负值      

	y1 = Err_speed_right;
	y2 = y2 + Err_speed_right;
	y3 = right_speed_old - now_right_speed;


	//积分限幅     小点好
	/*
	if(x2>=300)  x2=300;
	else if(x2<=-300)  x2=-300;

	if(y2>=300) y2=300;
	else if(y2<=-300)  y2=-300;
	*/

	//每次穿越积分清零，防退饱和超调



	//积分分离      左
	/*if (abs(Err_speed_left) <= 30)     // 0.1~0.2Err
	{
		//分段PI      左
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
	//应PI>=0

	//积分分离      右
	if (abs(Err_speed_right) <= 55)            // 0.1~0.2Err
	{
		//分段PI      右
		if (y1*y3<0)                       //偏差在减小
		{
			Kp_right[q + 1] = Kp_right[q] - p1*0.0001;
			if (y1*y2>0)
				Ki_right[q + 1] = Ki_right[q] + i1*0.001;
			else
				Ki_right[q + 1] = Ki_right[q] - i1*0.001;
		}
		else if (y1*y3>0)                   //偏差在增大
		{
			Kp_right[q + 1] = Kp_right[q] + p2*0.0001;
			if (y1*y2>0)
				Ki_right[q + 1] = Ki_right[q] + i1*0.001;
			else
				Ki_right[q + 1] = Ki_right[q] - i1*0.001;
		}
		else if (y1*y3 == 0)                 //稳态
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
		Kp_R = 1900 / (abs(y1));    //自动调整P
		y2 = 0;
	}



	//双限幅程序




	MOTOR_pwm_left = (int16)(Kp_L*x1 + Ki_L*x2);
	MOTOR_pwm_right = (int16)(Kp_R*y1 + Ki_R*y2);

	if (MOTOR_pwm_left>1900)           //输出限幅 左
	{
		MOTOR_pwm_left = 1900;
	}
	else
		if (MOTOR_pwm_left<-1900)
		{
			MOTOR_pwm_left = -1900;
		}

	if (MOTOR_pwm_right>1900)          //输出限幅 you
	{
		MOTOR_pwm_right = 1900;
	}
	else
		if (MOTOR_pwm_right<-1900)
		{
			MOTOR_pwm_right = -1900;
		}


	//赋值
	if (MOTOR_pwm_left >= 0)                //正转
	{
		MOTOR_GO_left(MOTOR_pwm_left);     //赋值左边
	}
	if (MOTOR_pwm_left<0)                 //反转
	{
		MOTOR_BACK_left(abs(MOTOR_pwm_left));
	}

	if (MOTOR_pwm_right >= 0)
	{
		MOTOR_GO_right(MOTOR_pwm_right);   //赋值右边
	}
	if (MOTOR_pwm_right<0)
	{
		MOTOR_BACK_right(abs(MOTOR_pwm_right));
	}

	//偏差存储  x2存了
	//Err_speed_old_left=Err_speed_left;
	//Err_speed_old_right=Err_speed_right;

	//实际速度存储
	left_speed_old = now_left_speed;
	right_speed_old = now_right_speed;

	//学习3次   实际一直学
	q++;
	if (q>2)
		q = 0;
}*/
