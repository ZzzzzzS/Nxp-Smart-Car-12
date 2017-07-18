#include "include.h"


/*============================================
函数名： Speed_Analyse()
作用：全局控制速度
==========================================*/

void Speed_Control()
{
	if (count >= 20)
	{
		Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
		count = 0;
	}
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
  /*ftm_pwm_init(MOTOR_FTM, LEFT_PWM, MOTOR_HZ, 50);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, LEFT_PWM_BACK, MOTOR_HZ, 0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM, MOTOR_HZ, 50);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, RIGHT_PWM_BACK, MOTOR_HZ, 0);      //初始化 电机 PWM*/
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

	Speed.Base.P = 0.05;
	Speed.Base.I = 0.02;
	Speed.Base.D = 0;

}

/*============================================
函数名：Motor_PID()
作用:对电机进行增量式PID控制
==========================================*/

void Motor_PID()
{
	
	char I_flag = 1;										//积分变量分离标志位
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
	Speed.Left.Out_Speed = Speed.Left.Turn_Speed + Speed.Base.PID_Out_Speed*1.2;
	Speed.Right.Out_Speed = Speed.Right.Turn_Speed + Speed.Base.PID_Out_Speed*0.9;
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
	Speed.Left.Now_Speed = ftm_quad_get(FTM1);				//获取正交解码脉冲数
	Speed.Right.Now_Speed = ftm_quad_get(FTM2);				//获取正交解码脉冲数
	
	Speed.Left.Now_Speed = -Speed.Left.Now_Speed;
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

	if ((Speed.Right.Now_Speed < 1) && (Speed.Right.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//检测系统错误
	{
		if ((Speed.Left.Now_Speed < 1) && (Speed.Left.Out_Speed > 90) && (Service.MotorBase.AllowRun))	//检测系统错误
		{
			//System_Error(Motor_Stop);
		}
	}
}