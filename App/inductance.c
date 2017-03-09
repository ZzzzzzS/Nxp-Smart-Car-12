#include "include.h"
#include "data.h"

/*============================================
函数名：ADC_Init()
作用:初始化ADC模数转换器
==========================================*/

void ADC_Init()
{
	adc_init(AMP1);													//初始化AMP1通道，PTB0
	adc_init(AMP2);													//初始化AMP2通道，PTB1
	//adc_init(AMP3);													//初始化AMP3通道，PTB2
	//adc_init(AMP4);													//初始化AMP4通道，PTB3
}

/*============================================
函数名：Get_MaxMin_AD_Value()
作用:采集ADC模数转换器传回的最大最小值，
并进行电感归一化处理
==========================================*/

/*============================================
什么是归一化:
(实际采集值 - 最小值)/(最大值 - 最小值)
作用:适应不同的场地
==========================================*/

//获取最大最小值时是否需要滤波？
void Get_MaxMin_AD_Value()
{
	int i;
	int16 AD_Value_Temp[2] = { 0,0 };
	for (i = 0; i < 2; i++)											//初始化最大最小值
	{
		Road_Data[i].Min_AD_Value = DEFAULT_MIN_VALUE;
		Road_Data[i].Max_AD_Value = DEFAULT_MAX_VALUE;
	}

	while (1)														//当开关一直按下时一直采集最大最小值并处理
	{
		AD_Value_Temp[LEFT] = adc_once(AMP1,ADC_8bit);				//采集过程
		AD_Value_Temp[RIGHT] = adc_once(AMP2, ADC_8bit);
	
		for (i = 0; i < 2; i++)										//比较并替换最大最小值过程
		{
			if (AD_Value_Temp[i] > Road_Data[i].Max_AD_Value)		//比较最大值
			{
				Road_Data[i].Max_AD_Value = AD_Value_Temp[i];
			}
			if (AD_Value_Temp[i] < Road_Data[i].Min_AD_Value)		//比较最小值
			{
				Road_Data[i].Min_AD_Value = AD_Value_Temp[i];
			}
		}
		for (i = 0; i < 2; i++)										//计算归一化的分母
		{
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}

		OLED_Normalization_Interface();								//OLED显示当前电感值
	}
}

/*============================================
函数名：ADC_Weight_Init()
作用:初始化权重向前滤波算法
==========================================*/
/*============================================
什么是权重向前滤波算法:
与之前采集的数据作加权平均数
作用:对采集的数据滤波消除偶然误差
==========================================*/

void ADC_Weight_Init()
{
	char i, j;

	for (i = 0; i < 2; i++)											//对前几次的电感值赋初值，避免影响前几次滤波后的数据
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value_Old[j] = (Road_Data[i].Max_AD_Value + Road_Data[i].Min_AD_Value) / 2;
		}
	}

	for (i = 0; i < 2; i++)											//滤波权重表赋初值
	{
		Road_Data[i].AD_Weight[0] = 1;
		Road_Data[i].AD_Weight[1] = 2;
		Road_Data[i].AD_Weight[2] = 3;
		Road_Data[i].AD_Weight[3] = 4;
	}
}

/*============================================
函数名：Get_AD_Value()
作用:采集ADC模数转换器传回的数据
==========================================*/
/*============================================
权重向前滤波法队列说明：
    [0][1][2][3]
旧数据        新数据
低权值        高权值
==========================================*/

void Get_AD_Value()
{
	char i, j;
	Road_Data[LEFT].AD_Value = adc_once(AMP1, ADC_8bit);	//采集过程
	Road_Data[RIGHT].AD_Value = adc_once(AMP2, ADC_8bit);

//我认为这段可以优化，存在数学关系
	for (i = 0; i < 2; i++)
	{
		if (Road_Data[i].AD_Value > Road_Data[i].Max_AD_Value)		//若检测到的电感值大于最大值，则修正最大值
		{
			Road_Data[i].Max_AD_Value = Road_Data[i].AD_Value;
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}
		if (Road_Data[i].AD_Value < Road_Data[i].Min_AD_Value)		//若检测到的电感值小于最小值，则修正最小值
		{
			Road_Data[i].Min_AD_Value = Road_Data[i].AD_Value;
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}
		for (j = 0; j < 3; j++)										//对电感数据队列移位操作
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//更新队首数据值
		Road_Data[i].AD_Value = 0;									//清空采集到的数据
	}

	for (i = 0; i < 2; i++)											//装入权重向前滤波法处理后的值
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
	}
	for (i = 0; i < 2; i++)											//装入电感归一化处理后的值
	{
		Road_Data[i].Normalized_Value = (Road_Data[i].AD_Value - Road_Data[i].Min_AD_Value) / Road_Data[i].normalization;
	}
}

/*============================================
函数名：Direction_Control()
作用:控制方向，速度等
==========================================*/

void Direction_Control()
{
	Direction.err = 0;												//采用权重算法计算误差
	Direction.err -= Road_Data[LEFT].Normalized_Value*LEFT_WEIGHT;
	Direction.err += Road_Data[RIGHT].Normalized_Value*RIGHT_WEIGHT;

	Left_Speed.Turn_Speed = Direction.err*DIRECTION_WEIGHT;
	Right_Speed.Turn_Speed = -Direction.err*DIRECTION_WEIGHT;

	//差弯道是否降低Go_Speed

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//计算最终目标速度
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}