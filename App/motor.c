#include "include.h"



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

    //IO管脚配置
    /*gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);*/
}

 /*============================================
 函数名：Motor_Control()
 作用:输出PWM波，控制电机转速
 ==========================================*/

void Motor_Control()
{
	if (Left_Speed.Out_Speed > MAX_SPEED)					//判断速度是否会超过边界
	{
		Left_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Left_Speed.Out_Speed < MIN_SPEED)
	{
		Left_Speed.Out_Speed = MIN_SPEED;
	}
	if (Right_Speed.Out_Speed > MAX_SPEED)
	{
		Right_Speed.Out_Speed = MAX_SPEED;
	}
	else if (Right_Speed.Out_Speed < MIN_SPEED)
	{
		Right_Speed.Out_Speed = MIN_SPEED;
	}
        
	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, Right_Speed.Out_Speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);
	ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, Left_Speed.Out_Speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0);
     //适应山外电驱
        
}

/*============================================
函数名：Motor_PID_Init()
作用:初始化PID
==========================================*/

void Motor_PID_Init()
{
	Left_Speed.Aim_Speed = 10;
	Right_Speed.Aim_Speed = 10;

	Left_Speed.Go_Speed = 10;
	Left_Speed.Go_Speed = 10;

	Left_Speed.Now_Speed = 0;
	Right_Speed.Now_Speed = 0;

	Left_Speed.Error_Speed = 0;
	Right_Speed.Error_Speed = 0;

	Left_Speed.err_last = 0;
	Right_Speed.err_last = 0;

	Left_Speed.err_next = 0;
	Right_Speed.err_next = 0;

	Left_Speed.P = 0.2;
	Right_Speed.P = 0.2;

	Left_Speed.I = 0.15;
	Right_Speed.I = 0.15;

	Left_Speed.D = 0.2;
	Right_Speed.D = 0.2;

}

/*============================================
函数名：Motor_PID()
作用:对电机进行增量式PID控制
==========================================*/

void Motor_PID()
{
	if (Left_Speed.Aim_Speed >= 100)//合法性检测
	{
		Left_Speed.Aim_Speed = 99;
	}
	else if (Left_Speed.Aim_Speed <= 0)
	{
		Left_Speed.Aim_Speed = 1;
	}
	if (Right_Speed.Aim_Speed >= 100)
	{
		Right_Speed.Aim_Speed = 99;
	}
	else if (Right_Speed.Aim_Speed <= 0)
	{
		Right_Speed.Aim_Speed = 1;
	}



	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//取得误差速度
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;

	float Left_IncrementSpeed, Right_IncrementSpeed;										//增量式PID核心公式
	Left_IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);		//左轮
	Left_IncrementSpeed += Left_Speed.I*Left_Speed.Error_Speed;
	Left_IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	Right_IncrementSpeed = Right_Speed.P*(Right_Speed.Error_Speed - Right_Speed.err_next);	//右轮
	Right_IncrementSpeed += Right_Speed.I*Right_Speed.Error_Speed;
	Right_IncrementSpeed += Right_Speed.D*(Right_Speed.Error_Speed - 2 * Right_Speed.err_next + Right_Speed.err_last);

	Left_Speed.err_last = Left_Speed.err_next;
	Right_Speed.err_last = Right_Speed.err_next;

	Left_Speed.err_next = Left_Speed.Error_Speed;
	Right_Speed.err_next = Right_Speed.Error_Speed;

	Left_Speed.PID_Out_Speed += Left_IncrementSpeed;
	Right_Speed.PID_Out_Speed += Right_IncrementSpeed;

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
	Left_Speed.Now_Speed = ftm_quad_get(FTM1);				//获取正交解码脉冲数
	Right_Speed.Now_Speed = ftm_quad_get(FTM2);
	ftm_quad_clean(FTM1);									//清正交解码脉冲数
	ftm_quad_clean(FTM2);	
}