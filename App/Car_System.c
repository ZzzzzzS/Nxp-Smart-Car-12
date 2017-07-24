#include "include.h"
#include "common.h"

/*============================================
函数名：System_Init()
作用：初始化整个系统
==========================================*/

int stopflag;

void Init_System()
{
	DisableInterrupts;																			//宏定义，禁止中断
	uart_init(Bluetooth, Bluetooth_Band);												//串口初始化
	ADC_Init();																					//ADC模数转换器初始化
	Init_Key();																						//初始化按键	
	Motor_Init();																					//电机初始化
	Motor_PID_Init();																			//电机PID控制初始化
	Get_Motor_Speed_Init();																//TPM解码初始化
	OLED_Init();																					//OLED初始化																				//停车检测初始化
	lptmr_timing_ms(2);																		//采用低功耗定时计数器，初始化定时计数器为定时模式，单位:ms
	set_vector_handler(LPTMR_VECTORn, LPTMR_IRQHandler);			//将系统控制主要中断函数加入到中断向量表中
	EnableInterrupts;																			//宏定义，允许中断
	disable_irq(LPTMR_IRQn);																//关闭低功耗定时计数器中断
	stopflag = 0;
}

/*============================================
函数名：Set_User_Information()
作用：设置用户参数
==========================================*/

void Set_User_Information()
{
	if (Service.RunMode == SlowMode)
	{
		Service.BlueToothBase.Information.speed = 30;
		Service.BlueToothBase.Information.P = 0.4;
		Service.BlueToothBase.Information.D = 60;

		Service.BlueToothBase.Information.MaxSpeed = 80;
		Service.BlueToothBase.Information.MinSpeed = -60;
		Service.BlueToothBase.Information.ToroidTurnTimes = 30;
		Service.BlueToothBase.Information.ToroidSpeed = 80;
        Service.BlueToothBase.Information.StopTimes=100;
	}
	else if (Service.RunMode == FastMode)
	{
		Service.BlueToothBase.Information.speed =40 ;
		Service.BlueToothBase.Information.P = 0.4;
		Service.BlueToothBase.Information.D = 60;

		Service.BlueToothBase.Information.MaxSpeed = 80;
		Service.BlueToothBase.Information.MinSpeed = -80;
		Service.BlueToothBase.Information.ToroidTurnTimes = 30;
		Service.BlueToothBase.Information.ToroidSpeed = 80;
                Service.BlueToothBase.Information.StopTimes=120;
	}
	
}

/*============================================
函数名：Get_System_Ready()
作用：跑前准备
==========================================*/

void Get_System_Ready()
{
	OLED_Interface();														//初始参数设置界面
	Set_User_Information();												//设置用户参数
	DELAY_MS(2000);
    Stop_Car_Init();
	enable_irq(LPTMR_IRQn);											//开启低功耗定时计数器中断，准备发车
}

/*============================================
函数名： LPTMR_IRQHandler()
作用：系统控制的主要周期性中断
==========================================*/

void LPTMR_IRQHandler()
{
  static unsigned char flag=0;
  flag++;
  if(flag>=25)
  {
    Get_Motor_Speed();
    flag=0;
  }
  if(Speed.Base.Now_Speed>100)
    led(LED0,LED_ON);
  else
    led(LED0,LED_OFF);
    Direction_Control();
    Speed_Control();
	stopflag++;
	if (stopflag > 2500)
	{
		Stop_Car();
		stopflag = 2600;
	}
    LPTMR_Flag_Clear();												//清除中断标志位，准备下一次中断
}

/*============================================
函数名：system_RunTime_Update()
作用：系统运行时执行的操作
==========================================*/

void system_RunTime_Update()
{
	DELAY_MS(200);													//两百毫秒执行一次，防止刷屏过快

	switch (Service.RunMode)										//判断不同模式执行不同操作
	{
	case FastMode:
		Send_Data();
		//Receive_Data();
		break;

	case inductance_Mode:
		DeBug_Interface();
		LED_Interface();
		Send_Data();
		Receive_Data();
		break;

	case SlowMode:
		Send_Data();
		//Receive_Data();
		break;

	default:
		Service.RunMode = inductance_Mode;
		System_Error(No_Mode);
		break;
	}
}