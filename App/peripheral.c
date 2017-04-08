#include "include.h"
#include "peripheral.h"

/*============================================
函数名：Stop_Car_Init()
作用：停车检测初始化
==========================================*/

/*void Stop_Car_Init()
{//不清楚上升沿还是下降沿
	port_init(CAR_STOP, ALT1 | IRQ_FALLING | PULLUP);			//初始化停车干簧管，下降沿中断
	set_vector_handler(PORTE_VECTORn, Stop_Car);				//设置停车中断向量
	enable_irq(PORTE_IRQn);										//使能向量
}*/

/*============================================
函数名：Stop_Car()
作用：停车检测
==========================================*/

/*void Stop_Car()
{
	if (PORTA_ISFR & (1 << CAR_STOP_NUM))						//确定中断管脚号
	{
		PORTA_ISFR = (1 << CAR_STOP_NUM);						//清除中断标志位

		disable_irq(LPTMR_IRQn);								//禁止低功耗定时计数器中断
		Right_Speed.Out_Speed = 0;								//调整速度为0
		Left_Speed.Out_Speed = 0;								//调整速度为0
		ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);					//电机输出
		ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);					//电机输出
		ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);					//电机输出
		ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0);					//电机输出
	}
}*/

/*============================================
函数名：Send_Data()
作用：蓝牙串口发送
==========================================*/

void Send_Data()
{
	char var[AMP_MAX];
	for (counter i = 0; i < AMP_MAX; i++)
	{
		var[i] = Road_Data[i].AD_Value_fixed;						//向上位机发送电感归一化后的值
	}
	vcan_sendware(var, sizeof(var));							//发送到上位机，注意发送协议，发送端口
	printf("Out_Speed %d %d\n ", Left_Speed.Out_Speed, Right_Speed.Out_Speed);
	printf("NowSpeed %d %d\n", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
	printf("AimSpeed %d %d\n", Left_Speed.Aim_Speed, Right_Speed.Aim_Speed);
	printf("p=%f %f\n", Left_Speed.P, Right_Speed.P);
	printf("I=%f %f\n", Left_Speed.I, Right_Speed.I);
}

/*============================================
函数名：Key_Init()
作用：初始化按键
==========================================*/

void Init_Key()
{
	gpio_init(Key1, GPI, 0);                       //按键初始化
	gpio_init(Key2, GPI, 0);						//按键初始化
	gpio_init(Key3, GPI, 0);                      //按键初始化
	gpio_init(Key4, GPI, 0);                      //按键初始化
	//gpio_init(key5, GPI, 0);					    //按键初始化
}

/*============================================
函数名：OLED_Interface()
作用：OLED显示发车前界面
==========================================*/

void OLED_Interface()
{
	OLED_Print(15, 0, "718创新实验室");
	OLED_Print(27, 2, "untitled组");
	OLED_Print(Position(Line3), "调试模式   <-");
	OLED_Print(Position(Line4), "发布模式");
	mode flag = 0;											//模式判断标志位

	while (true)
	{
          DELAY_MS(100);
		if (gpio_get(Key1) != 0)							//确认按钮
		{
			DELAY_MS(10);									//消抖延时
			if (gpio_get(Key1) != 0)
			{
                          OLED_CLS();
						  if (0 == flag % 2)
						  {
							  Debug_Init();
						  }
			}
			break;
		}

		if (gpio_get(Key4) != 0)						//选择按钮
		{
			DELAY_MS(10);								//消抖延时
			if (gpio_get(Key4) != 0)
			{
				flag++;
				OLED_CLS();
				OLED_Print(15, 0, "718创新实验室");
				OLED_Print(27, 2, "untitled组");
				if (0 == flag % 2)
				{
					OLED_Print(Position(Line3), "调试模式   <-");
					OLED_Print(Position(Line4), "发布模式");
				}
				else
				{
					OLED_Print(Position(Line3), "调试模式");
					OLED_Print(Position(Line4), "发布模式   <-");
				}
			}
		}
	}
}

/*============================================
函数名：DeBug_Interface()
作用：OLED显示调试时界面
==========================================*/

void DeBug_Interface()
{
	data temp[10];
	if (Service.flag == Inductance_Interface)
	{
		OLED_CLS();
		OLED_Print(Position(Line1), "电感调试");
		sprintf(temp, "    FL=%d FR=%d", Road_Data[FRONT_LEFT].AD_Value_fixed, Road_Data[FRONT_RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "L=%d R=%d", Road_Data[LEFT].AD_Value_fixed, Road_Data[RIGHT].AD_Value_fixed);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "M    %d", Road_Data[MIDDLE].AD_Value_fixed);
		OLED_Print(Position(Line4), temp);
	}
	else if (Service.flag == Speed_Interface)
	{
		OLED_CLS();																															//先清屏
		sprintf(temp, "out=L%d R%d", Left_Speed.Out_Speed, Right_Speed.Out_Speed);					//电机输出功率
		OLED_Print(Position(Line1), temp);
		sprintf(temp, "now=L%d R%d", Left_Speed.Now_Speed, Right_Speed.Now_Speed);				//当前速度
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "aim=L%d R%d", Left_Speed.Aim_Speed, Right_Speed.Aim_Speed);					//目标速度
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "set=L%d R%d", Left_Speed.Go_Speed, Right_Speed.Go_Speed);						//设定速度
		OLED_Print(Position(Line4), temp);
	}
	else if (Service.flag == PID_Interface)
	{
		OLED_CLS();
		sprintf(temp, "P %.2f %.2f", Left_Speed.P, Right_Speed.P);
		OLED_Print(Position(Line2), temp);
		sprintf(temp, "I %.2f %.2f", Left_Speed.I, Right_Speed.I);
		OLED_Print(Position(Line3), temp);
		sprintf(temp, "D %.1f %.1f", Left_Speed.D, Right_Speed.D);
		OLED_Print(Position(Line4), temp);
	}

	if (gpio_get(Key2) != 0)
	{
		Left_Speed.Go_Speed += 10;
		Right_Speed.Go_Speed += 10;
	}
	if (gpio_get(Key4) != 0)
	{
		Left_Speed.Go_Speed -= 10;
		Right_Speed.Go_Speed -= 10;

		if (Left_Speed.Go_Speed < 0)
			Left_Speed.Go_Speed = 0;
		if (Right_Speed.Go_Speed < 0)
			Right_Speed.Go_Speed = 0;
	}
	
	if (gpio_get(Key1) != 0)
	{
		Service.flag++;
		if (Service.flag >=MAX_Interface)
			Service.flag = 0;
	}
}

/*============================================
函数名：Debug_Init()
作用：调试模式时初始化调试组件
==========================================*/

void Debug_Init()
{
	Service.isDebug = true;
	Service.flag = 0;
}

/*============================================
函数名：System_Error(error Error_Number)
作用：调试模式系统错误紧急停车
==========================================*/

void System_Error(error Error_Number)
{
	disable_irq(LPTMR_IRQn);									//禁止低功耗定时计数器中断
	Right_Speed.Out_Speed = 0;								//调整速度为0
	Left_Speed.Out_Speed = 0;								//调整速度为0
	Motor_Control();												//控制电机
	if (Error_Number == Motor_Stop)
	{
		OLED_Init();
		OLED_Print(0, 0, "电机堵转");
	}

        while(1);
}

/*============================================
函数名：Debug()
作用：调试模式定时中断
==========================================*/

void Debug()
{
		DeBug_Interface();
		Send_Data();
}