#include "include.h"
#include "data.h"

#define LEFT_WEIGHT				1						//定义转向权重
#define RIGHT_WEIGHT	        1						//定义转向权重

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
	adc_init(AMP1);													//初始化AMP1通道，PTB0
	adc_init(AMP2);													//初始化AMP2通道，PTB1
	adc_init(AMP3);									 				//初始化AMP3通道，PTB2
	adc_init(AMP4);													//初始化AMP4通道，PTB3
	adc_init(AMP5);													//初始化AMP5通道，PTB4

	for (counter i = 0; i < AMP_MAX; i++)				//滤波权重表赋初值
	{
		//注意:修改权重值时请修改data.h->MAX_WEIGHT!
		Road_Data[i].AD_Weight[0] = 1;
		Road_Data[i].AD_Weight[1] = 2;
		Road_Data[i].AD_Weight[2] = 3;
		Road_Data[i].AD_Weight[3] = 4;
	}
}


/*============================================
函数名：Direction_Control()
作用:全局控制方向
==========================================*/

void Direction_Control()
{
	Get_AD_Value();
	//Similarity_Count_Fuzzy();
	//Direction_Control_Fuzzy();
	//if (!Fuzzy_Direction.isMatched)
	//{
		Direction_Calculate();
	//}
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
	Road_Data[0].AD_Value = adc_once(AMP1, ADC_8bit);				//采集过程
	Road_Data[1].AD_Value = adc_once(AMP2, ADC_8bit);				//采集过程
	Road_Data[2].AD_Value = adc_once(AMP3, ADC_8bit);				//采集过程
	Road_Data[3].AD_Value = adc_once(AMP4, ADC_8bit);				//采集过程
	Road_Data[4].AD_Value = adc_once(AMP5, ADC_8bit);				//采集过程
	//注意修改通道初始化

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 3; j++)						//对电感数据队列移位操作
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//更新队首数据值
	}

	for (counter i = 0; i < AMP_MAX; i++)						//装入权重向前滤波法处理后的值
	{
		Road_Data[i].AD_Value = 0;
		for (counter j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
		Road_Data[i].AD_Value_fixed=Road_Data[i].AD_Value/MAX_WEIGHT;
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
	Direction.sum[0] = 100*(Road_Data[RIGHT].AD_Value_fixed - Road_Data[LEFT].AD_Value_fixed) / (Road_Data[LEFT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed);
	Direction.sum[1] = 100*(Road_Data[MIDDLE].AD_Value_fixed - Road_Data[LEFT].AD_Value_fixed) / (Road_Data[LEFT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed);
	Direction.sum[2] = 100*(Road_Data[MIDDLE].AD_Value_fixed - Road_Data[RIGHT].AD_Value_fixed) / (Road_Data[MIDDLE].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed);

	//Direction.sum[1] = k1*Direction.sum[1] + b1;
	//Direction.sum[2] = k2*Direction.sum[1] + b2;

	char a = 1, b = 1;
	if (Direction.sum[1] < 0)
		a = -1;
	if (Direction.sum[2] < 0)
		b = -1;


	Direction.sum[2] = a1*Direction.sum[2] * Direction.sum[2] + b1*Direction.sum[2] + c1;
	Direction.sum[1] = a2*Direction.sum[1] * Direction.sum[1] + b1*Direction.sum[1] + c2;

	Direction.sum[1] *= a;
	Direction.sum[2] *= b;

	Direction.err = (Direction.sum[0] + Direction.sum[1] + Direction.sum[2]) / 3;
	Direction.err *= 0.006;

	char flag = 0;
	if (Direction.err < 0)		//方向判断，防止平方后正负项消失
	{
		flag = -1;
	}
	else
	{
		flag = 1;
	}
	

        if(Direction.err<5&&Direction.err>-5)
          Direction.err=0;
        else if(Direction.err>30)
          Direction.err*=1.2;
        else if(Direction.err<-30)
          Direction.err*=1.2;




	
	//Left_Speed.Turn_Speed = -sqrt(CharAbs(Direction.err*Direction.err*Direction.err))*flag;									//计算差速
	//Right_Speed.Turn_Speed = sqrt(CharAbs(Direction.err*Direction.err*Direction.err))*flag;


	Left_Speed.Turn_Speed = -Direction.err;
	Right_Speed.Turn_Speed = Direction.err;


	//差弯道是否降低Go_Speed
        
	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//计算最终目标速度
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;

}

/*============================================
函数名：eRule_Init_Fuzzy()
作用:模糊控制论域eRule初始化
==========================================*/

void eRule_Init_Fuzzy()
{
	Fuzzy_Direction.eRule[0].eAngle = 0;					//初始化角度相似规则
	Fuzzy_Direction.eRule[1].eAngle = 0;					//初始化角度相似规则
	Fuzzy_Direction.eRule[2].eAngle = 0;					//初始化角度相似规则
	Fuzzy_Direction.eRule[3].eAngle = 0;					//初始化角度相似规则
	Fuzzy_Direction.eRule[4].eAngle = 0;					//初始化角度相似规则
	Fuzzy_Direction.eRule[5].eAngle = 0;					//初始化角度相似规则
	//注意修改MAX_FUZZY_RULE!

	Fuzzy_Direction.eRule[0].eLength = 0;				//初始化长度相似规则
	Fuzzy_Direction.eRule[1].eLength = 0;				//初始化长度相似规则
	Fuzzy_Direction.eRule[2].eLength = 0;				//初始化长度相似规则
	Fuzzy_Direction.eRule[3].eLength = 0;				//初始化长度相似规则
	Fuzzy_Direction.eRule[4].eLength = 0;				//初始化长度相似规则
	Fuzzy_Direction.eRule[5].eLength = 0;				//初始化长度相似规则
	//注意修改MAX_FUZZY_RULE!

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
	float eDenominator = 0;														//余弦分母/临时变量
	float eNumerator = 0;															//余弦分子/长度计算临时变量

	/*****误差角度计算*****/
	for (counter j = 0; j < AMP_MAX; j++)									//计算余弦分子
	{
		eNumerator += Road_Data[j].AD_Value_fixed;
	}

	for (counter j = 0; j < AMP_MAX; j++)									//计算余弦分母
	{
			eDenominator += Road_Data[j].AD_Value_fixed * Road_Data[j].AD_Value_fixed;
	}
	eDenominator= sqrt(eDenominator);//需要研究更好的开方函数

	Fuzzy_Direction.Position.eAngle = eNumerator / eDenominator;	//计算与(1,1,1,1,...)的夹角余弦值

	/*****误差长度计算*****/
	for (counter j = 0; j < AMP_MAX; j++)									//计算向量长度
	{
		eNumerator += Road_Data[j].AD_Value_fixed * Road_Data[j].AD_Value_fixed;
	}
	Fuzzy_Direction.Position.eLength = sqrt(eNumerator);			//储存计算出的长度

	/*****隶属度计算*****/											
	for (counter j = 0; j < MAX_FUZZY_RULE; j++)
	{
		Fuzzy_Direction.eGrade[j].eAngle = Fuzzy_Direction.Position.eAngle - Fuzzy_Direction.eRule[j].eAngle;
		Fuzzy_Direction.eGrade[j].eLength = Fuzzy_Direction.Position.eLength - Fuzzy_Direction.eRule[j].eLength;
	}
}

/*============================================
函数名：Direction_Control_Fuzzy()
作用:模糊控制计算方向,速度等
==========================================*/

void Direction_Control_Fuzzy()
{
	int16 e=0;
	Fuzzy_Direction.isMatched = false;													//标记为未匹配到

	for (counter i = 0; i < MAX_FUZZY_RULE; i++)
	{
		if (Fuzzy_Direction.eGrade[i].eAngle < 0.1)										//匹配相似度90%
		{
			Fuzzy_Direction.isMatched = true;												//匹配成功标记
			if (0 == i)																					//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (1 == i)																			//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (2 == i)																			//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (3 == i)																			//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (4 == i)																			//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (5 == i)																			//根据匹配结果执行不同操作
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
		}
	}
	
	if (Fuzzy_Direction.isMatched)
	{
		Left_Speed.Turn_Speed = e;																					//计算差速
		Right_Speed.Turn_Speed = -e;

		Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//计算最终目标速度
		Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
	}
}