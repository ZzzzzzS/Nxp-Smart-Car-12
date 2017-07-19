#include "include.h"
#include "data.h"

/*============================================
电感顺序：
	左 中 上左 上右 右
==========================================*/

/*============================================
函数名：ADC_Init()
作用:初始化ADC模数转换器,初始化权重滤波
==========================================*/
/*============================================
什么是权重向前滤波算法:
与之前采集的数据作加权平均数
作用:对采集的数据滤波消除偶然误差
==========================================*/

void ADC_Init()
{
	adc_init(AD1);													//初始化AMP1通道，PTB0
	adc_init(AD2);													//初始化AMP2通道，PTB1
	adc_init(AD3);									 				//初始化AMP3通道，PTB2
	adc_init(AD4);													//初始化AMP4通道，PTB3
}


/*============================================
函数名：Direction_Control()
作用:全局控制方向
==========================================*/

void Direction_Control()
{
	Get_AD_Value();
	if (!hasToroid())
	{
		Direction_Calculate();
	}
	Direction_PID();
}

/*============================================
函数名：Get_AD_Value()
作用:采集ADC模数转换器传回的数据
==========================================*/
/*============================================
权重向前滤波法队列说明：
    [0][1][2][3][4][5][6][7][8][9][10]
旧数据        新数据
低权值        高权值
==========================================*/

void Get_AD_Value()
{
	Road_Data[FRONT].AD_Value = adc_once(AD1, ADC_8bit);					//采集过程
	Road_Data[LEFT].AD_Value = adc_once(AD2, ADC_8bit);				//采集过程
	Road_Data[RIGHT].AD_Value = adc_once(AD3, ADC_8bit);		//采集过程
	Road_Data[MIDDLE].AD_Value = adc_once(AD4, ADC_8bit);				//采集过程

	//注意修改通道初始化
	//注意修改通道初始化

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 9; j++)						//对电感数据队列移位操作
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[9] = Road_Data[i].AD_Value;		//更新队首数据值
	}

	for (counter i = 0; i < AMP_MAX; i++)						//装入权重向前滤波法处理后的值
	{
		Road_Data[i].AD_Value = 0;
		int sum = 0;
		for (counter j = 0; j < 10; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * (j + 1);
			sum += (j+1);
		}
		Road_Data[i].AD_Value_fixed = Road_Data[i].AD_Value / sum;
	}

	if ((Road_Data[MIDDLE].AD_Value_fixed <= 5) && (Road_Data[LEFT].AD_Value_fixed <= 5) && (Road_Data[RIGHT].AD_Value_fixed <= 5) && (Service.RunMode != inductance_Mode))
	{
		Service.InductanceBase.InductanceLost++;
		if (Service.InductanceBase.InductanceLost >= 50);
                  System_Error(Taget_Lost);
	}
	else
	{
		Service.InductanceBase.InductanceLost = 0;
	}
}

/*============================================
函数名：Direction_Control()
作用:控制方向,速度等
==========================================*/
/*============================================
作用:计算差比和
什么是差比和算法:
(采集到的值)/(Σ本次采集采集到的所有值)
作用:对采集的数据滤波消除偶然误差
==========================================*/

void Direction_Calculate()
{

	if (Road_Data[RIGHT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed != 0)
	{
		Direction.sum[0] = 100 * (Road_Data[RIGHT].AD_Value_fixed - Road_Data[LEFT].AD_Value_fixed) / (Road_Data[LEFT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed);				//差比和计算
	}
	else
	{
		Direction.sum[0] = 0;
	}
	if (Road_Data[LEFT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed != 0)
	{
		Direction.sum[1] = 100 * (Road_Data[LEFT].AD_Value_fixed - Road_Data[MIDDLE].AD_Value_fixed) / (Road_Data[LEFT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed);			//差比和计算
	}
	else
	{
		Direction.sum[1] = 0;
	}
	if (Road_Data[RIGHT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed != 0)
	{
		Direction.sum[2] = 100 * (Road_Data[RIGHT].AD_Value_fixed - Road_Data[MIDDLE].AD_Value_fixed) / (Road_Data[MIDDLE].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed);		//差比和计算
	}
	else
	{
		Direction.sum[2] = 0;
	}

	Direction.sum[2] = Rk*Direction.sum[2] + Rb;
	Direction.sum[1] = Lk*Direction.sum[1] + Lb;

	Direction.err = (Direction.sum[0] * 5 + Direction.sum[1] + Direction.sum[2]) / 7;						//计算出最终误差

	Direction.PIDbase.Error_Speed[Now_Error] = Direction.err;

	if (Direction.sum[0] > 30)
	{
		Direction.PIDbase.Error_Speed[Now_Error] = (Direction.sum[0] * 5 + 2 * Direction.sum[2]) / 7;
	}
	if (Direction.sum[0] < -30)
	{
		Direction.PIDbase.Error_Speed[Now_Error] = (Direction.sum[0] * 5 + 2 * Direction.sum[1]) / 7;
	}

/*#define more 32
#define less 20

	Direction.PIDbase.D = Service.BlueToothBase.Information.D;
	Direction.PIDbase.P = Service.BlueToothBase.Information.P;

	if (Direction.PIDbase.Error_Speed[Now_Error]<less && Direction.PIDbase.Error_Speed[Now_Error]>-less)
	{
		Direction.PIDbase.P *= Direction.PIDbase.Error_Speed[Now_error]*;
		Direction.PIDbase.D *= 0.3;
	}
	if (Direction.PIDbase.Error_Speed[Now_Error] >  more)
	{
		Direction.PIDbase.P *= 1.4;
		Direction.PIDbase.D *= 1.3;
	}
	if (Direction.PIDbase.Error_Speed[Now_Error] < -more)
	{
		Direction.PIDbase.P *= 1.4;
		Direction.PIDbase.D *= 1.3;
	}
#undef more
#undef less*/
        Direction.PIDbase.D = Service.BlueToothBase.Information.D;
	Direction.PIDbase.P = Service.BlueToothBase.Information.P;
        if(Direction.PIDbase.Error_Speed[Now_Error]>0)
          Direction.PIDbase.P*=(0.04*Direction.PIDbase.Error_Speed[Now_Error]+0.1);
        else
          Direction.PIDbase.P*=(-0.04*Direction.PIDbase.Error_Speed[Now_Error]+0.1);
        
        if(Direction.PIDbase.Error_Speed[Now_Error]>0)
          Direction.PIDbase.D*=(0.03*Direction.PIDbase.Error_Speed[Now_Error]+0.18);
        else
          Direction.PIDbase.D*=(-0.03*Direction.PIDbase.Error_Speed[Now_Error]+0.18);

	Speed.Base.Aim_Speed = Service.BlueToothBase.Information.speed;
	TempSpeed = Service.BlueToothBase.Information.speed;

}

/*============================================
函数名：Direction_PID()
作用:方向PID控制
==========================================*/

void Direction_PID()
{
	int16 err_Delta = Direction.PIDbase.Error_Speed[Now_Error] - Direction.PIDbase.Error_Speed[last_Error];
	Direction.PIDbase.Error_Speed[last_Error] = Direction.PIDbase.Error_Speed[Now_Error];

	Direction.PIDbase.PID_Out_Speed = Direction.PIDbase.P*Direction.PIDbase.Error_Speed[Now_Error];
	Direction.PIDbase.PID_Out_Speed += Direction.PIDbase.D * err_Delta;

	Speed.Left.Turn_Speed = Direction.PIDbase.PID_Out_Speed;
	Speed.Right.Turn_Speed = -Direction.PIDbase.PID_Out_Speed;
}

bool hasToroid()
{
	if (Road_Data[LEFT].AD_Value_fixed < 75 && Road_Data[RIGHT].AD_Value_fixed < 100 && Road_Data[MIDDLE].AD_Value_fixed < 75)
	{
		if ((((Road_Data[LEFT].AD_Value_fixed+10) < Road_Data[MIDDLE].AD_Value_fixed)) || (((Road_Data[RIGHT].AD_Value_fixed+10) < Road_Data[MIDDLE].AD_Value_fixed)))
		{	
			if (1)
                        {
                          if(30<Road_Data[RIGHT].AD_Value_fixed&&30<Road_Data[LEFT].AD_Value_fixed)
                          {
				TempSpeed= 0;
				Direction.PIDbase.Error_Speed[Now_Error] = -50;//基本最大误差
				led(LED2, LED_ON);
			return true;
                          } 
                        }
			     
		}
		led(LED2, LED_OFF);
		return false;
	}
	else
	{
        led(LED2, LED_OFF);
		return false;
	}
}
