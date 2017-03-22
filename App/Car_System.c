#include "include.h"
#include "common.h"

/*============================================
函数名：System_Init()
作用：初始化整个系统
==========================================*/

void System_Init()
{
	DisableInterrupts;												//宏定义，禁止中断
	flash_init();													//flash初始化
	ADC_Init();														//ADC模数转换器初始化
	//eRule_Init_Fuzzy();											//方向模糊控制论域初始化
	Init_Key();														//初始化按键	
	Motor_Init();													//电机初始化
	Motor_PID_Init();												//电机PID控制初始化
	Get_Motor_Speed_Init();											//FTM正交解码初始化
	OLED_Init();													//OLED初始化
	//Stop_Car_Init();												//停车检测初始化
	lptmr_timing_ms(20);											//采用低功耗定时计数器，初始化定时计数器为定时模式，单位:ms
	set_vector_handler(LPTMR_VECTORn, LPTMR_IRQHandler);			//将系统控制主要中断函数加入到中断向量表中
	EnableInterrupts;												//宏定义，允许中断
	disable_irq(LPTMR_IRQn);										//关闭低功耗定时计数器中断
}

/*============================================
函数名：Get_System_Ready()
作用：跑前准备
==========================================*/

void Get_System_Ready()
{
	OLED_Interface();												//初始参数设置界面
	ADC_Weight_Init();												//初始化权重向前滤波法
	enable_irq(LPTMR_IRQn);											//开启低功耗定时计数器中断，准备发车
}

/*============================================
函数名： LPTMR_IRQHandler()
作用：系统控制的主要周期性中断
==========================================*/

void LPTMR_IRQHandler()
{
	Get_AD_Value();													//获取ADC数模转换器的值
	Direction_Control();											//获得目标转向角度
	Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
	//FuzzyPID();														//对PID参数模糊控制
	Motor_PID();													//对电机进行增量式PID调节
	Motor_Control();												//输出最终速度
	Send_Data();
	LPTMR_Flag_Clear();												//清除中断标志位，准备下一次中断
        
	
}

/*============================================
函数名：Debug_Init()
作用：调试模式时初始化调试组件
==========================================*/

void Debug_Init()
{
//这里暂时不需要写
}



