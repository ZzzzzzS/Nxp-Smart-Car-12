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
	//adc_init(AMP3);												//初始化AMP3通道，PTB2
	//adc_init(AMP4);												//初始化AMP4通道，PTB3
	//adc_init(AMP5);												//初始化AMP5通道，PTB4
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
	for (char i = 0; i < AMP_MAX; i++)								//滤波权重表赋初值
	{
		//注意:修改权重值时请修改data.h->MAX_WEIGHT!
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
	Road_Data[LEFT].AD_Value = adc_once(AMP1, ADC_8bit);			//采集过程
	Road_Data[RIGHT].AD_Value = adc_once(AMP2, ADC_8bit);
	//Road_Data[3].AD_Value = adc_once(AMP3, ADC_8bit);
	//Road_Data[4].AD_Value = adc_once(AMP4, ADC_8bit);
	//Road_Data[5].AD_Value = adc_once(AMP5, ADC_8bit);


	for (i = 0; i < AMP_MAX; i++)
	{
		for (j = 0; j < 3; j++)										//对电感数据队列移位操作
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//更新队首数据值
		Road_Data[i].AD_Value = 0;									//清空采集到的数据
	}

/*============================================
作用:计算差比和
==========================================*/
/*============================================
什么是差比和算法:
(采集到的值)/(Σ本次采集采集到的所有值)
作用:对采集的数据滤波消除偶然误差
==========================================*/

	Direction.Normalization_Value = 0;								//清空上一次差比和的和总值
	double temp;													//临时储存电感值

	for (i = 0; i < AMP_MAX; i++)									//装入权重向前滤波法处理后的值
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
		Road_Data[i].AD_Value /= MAX_WEIGHT;
		Direction.Normalization_Value += Road_Data[i].AD_Value;		//计算本次差比和的和总值
	}
	for (i = 0; i < AMP_MAX; i++)									//计算差比和后的电感值
	{
		temp = Road_Data[i].AD_Value;
		Road_Data[i].Normalized_Value = (int)((temp / (double)Direction.Normalization_Value) * 200.0);
	}
}

/*============================================
函数名：Direction_Control()
作用:控制方向,速度等
==========================================*/

void Direction_Control()
{
	Direction.err = 0;												//采用权重算法计算误差
	Direction.err -= Road_Data[LEFT].Normalized_Value*LEFT_WEIGHT;
	Direction.err += Road_Data[RIGHT].Normalized_Value*RIGHT_WEIGHT;
	//AMP_MAX需要修改！！！
	Left_Speed.Turn_Speed = Direction.err;
	Right_Speed.Turn_Speed = -Direction.err;

	//差弯道是否降低Go_Speed

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//计算最终目标速度
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}

/*============================================
函数名：Direction_Control_Fuzzy()
作用:模糊控制计算方向,速度等
==========================================*/

void Direction_Control_Fuzzy()
{
	const unsigned char eRule[MAX_FUZZY_RULE][5/*AMP_MAX*/] =		//模糊论域
	{
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 }
	};
}

/*============================================
函数名：Similarity_Count_Fuzzy()
作用:模糊控制计算模糊隶属度
==========================================*/
/*============================================
方法：余弦相似性
计算隶属度：设有一个N维空间
取AMP_MAX个电感值构成一个N维列向量
计算这个N维向量与已知N维向量的COS值
来表示采集到的值与模糊论域中的值隶属程度
==========================================*/

void Similarity_Count_Fuzzy()
{
	const unsigned char eRule[MAX_FUZZY_RULE][5/*AMP_MAX*/] =		//模糊论域
	{
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 },
		{ 0,0,0,0,0 }
	};//研究结构体怎么初始化数组

	double eDenominator[2];											//余弦分母
	double eNumerator;												//余弦分子
	double temp;

	for (unsigned char i = 0; i < MAX_FUZZY_RULE; i++)				//求与MAX_FUZZY_RULE个值的隶属度
	{
		temp = 0;													//清零参数
		eNumerator = 0;												//清零参数
		eDenominator[0] = 0;										//清零参数
		eDenominator[1] = 0;										//清零参数


		for (unsigned char j = 0; j < AMP_MAX; j++)					//计算余弦分子
		{
			eNumerator += Road_Data[j].AD_Value * eRule[i][j];
		}

		for (unsigned char j = 0; j < AMP_MAX; j++)					//计算余弦分母1
		{
			temp += Road_Data[j].AD_Value * Road_Data[j].AD_Value;
		}
		eDenominator[1] = sqrt(temp);//需要研究更好的开方函数
		temp = 0;

		for (unsigned char j = 0; j < AMP_MAX; j++)					//计算余弦分母2
		{
			temp += eRule[i][j] * eRule[i][j];
		}
		eDenominator[2] = sqrt(temp);

		Fuzzy_Direction.eFuzzy[i] = eNumerator / (eDenominator[1] * eDenominator[2]);	//计算隶属度
	}
}
