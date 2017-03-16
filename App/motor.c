#include "include.h"



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

    //IO�ܽ�����
    /*gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);*/
}

 /*============================================
 ��������Motor_Control()
 ����:���PWM�������Ƶ��ת��
 ==========================================*/

void Motor_Control()
{
	if (Left_Speed.Out_Speed > MAX_SPEED)					//�ж��ٶ��Ƿ�ᳬ���߽�
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
     //��Ӧɽ�����
        
}

/*============================================
��������Motor_PID_Init()
����:��ʼ��PID
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
��������Motor_PID()
����:�Ե����������ʽPID����
==========================================*/

void Motor_PID()
{
	if (Left_Speed.Aim_Speed >= 100)//�Ϸ��Լ��
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



	Left_Speed.Error_Speed = Left_Speed.Aim_Speed - Left_Speed.Now_Speed;					//ȡ������ٶ�
	Right_Speed.Error_Speed = Right_Speed.Aim_Speed - Right_Speed.Now_Speed;

	float Left_IncrementSpeed, Right_IncrementSpeed;										//����ʽPID���Ĺ�ʽ
	Left_IncrementSpeed = Left_Speed.P*(Left_Speed.Error_Speed - Left_Speed.err_next);		//����
	Left_IncrementSpeed += Left_Speed.I*Left_Speed.Error_Speed;
	Left_IncrementSpeed += Left_Speed.D*(Left_Speed.Error_Speed - 2 * Left_Speed.err_next + Left_Speed.err_last);

	Right_IncrementSpeed = Right_Speed.P*(Right_Speed.Error_Speed - Right_Speed.err_next);	//����
	Right_IncrementSpeed += Right_Speed.I*Right_Speed.Error_Speed;
	Right_IncrementSpeed += Right_Speed.D*(Right_Speed.Error_Speed - 2 * Right_Speed.err_next + Right_Speed.err_last);

	Left_Speed.err_last = Left_Speed.err_next;
	Right_Speed.err_last = Right_Speed.err_next;

	Left_Speed.err_next = Left_Speed.Error_Speed;
	Right_Speed.err_next = Right_Speed.Error_Speed;

	Left_Speed.PID_Out_Speed += Left_IncrementSpeed;
	Right_Speed.PID_Out_Speed += Right_IncrementSpeed;

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
	Left_Speed.Now_Speed = ftm_quad_get(FTM1);				//��ȡ��������������
	Right_Speed.Now_Speed = ftm_quad_get(FTM2);
	ftm_quad_clean(FTM1);									//����������������
	ftm_quad_clean(FTM2);	
}