//先跑起来分支
#include "include.h"

void main()
{
	System_Init();
	Get_System_Ready();
	while (true)
	{
		if (Service.Debug == true)
		{
			//Send_Data();
		}
                
                	Get_AD_Value();													//获取ADC数模转换器的值
	Direction_Control();											//获得目标转向角度
	Get_Motor_Speed();												//获取FTM正交解码脉冲采集器的值
	Motor_PID();													//对电机进行增量式PID调节
	Motor_Control();												//输出最终速度
	//Stop_Car();														//停车检测

	//LPTMR_Flag_Clear();												//清除中断标志位，准备下一次中断
	}
}