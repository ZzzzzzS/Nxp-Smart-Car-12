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
	adc_init(AD5);													//初始化AMP5通道，PTB4
        adc_init(AD6);
}


/*============================================
函数名：Direction_Control()
作用:全局控制方向
==========================================*/

void Direction_Control()
{
	Get_AD_Value();
	Similarity_Count_Fuzzy();
	Direction_Control_Fuzzy();
	if (!Fuzzy_Direction.isMatched)
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
    [0][1][2][3]
旧数据        新数据
低权值        高权值
==========================================*/

void Get_AD_Value()
{
	Road_Data[LEFT].AD_Value = adc_once(AD1, ADC_8bit);					//采集过程
	Road_Data[RIGHT].AD_Value = adc_once(AD2, ADC_8bit);				//采集过程
	Road_Data[MIDDLE].AD_Value = adc_once(AD3, ADC_8bit);				//采集过程
	Road_Data[FRONT_LEFT].AD_Value = adc_once(AD4, ADC_8bit);		//采集过程
	Road_Data[FRONT_RIGHT].AD_Value = adc_once(AD5, ADC_8bit);	//采集过程
	Road_Data[test].AD_Value = adc_once(AD6, ADC_8bit);	//采集过程
	//注意修改通道初始化
	//注意修改通道初始化

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 2; j++)						//对电感数据队列移位操作
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[2] = Road_Data[i].AD_Value;		//更新队首数据值
	}

	for (counter i = 0; i < AMP_MAX; i++)						//装入权重向前滤波法处理后的值
	{
		Road_Data[i].AD_Value = 0;
		for (counter j = 0; j < 3; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * (j + 1);
		}
		Road_Data[i].AD_Value_fixed = Road_Data[i].AD_Value / 6;
	}

	if ((Road_Data[MIDDLE].AD_Value_fixed <= 5) && (Road_Data[LEFT].AD_Value_fixed <= 5) && (Road_Data[RIGHT].AD_Value_fixed <= 5) && (Service.RunMode != inductance_Mode))
	{
		Service.InductanceBase.InductanceLost++;
		if (Service.InductanceBase.InductanceLost >= 50);
		//System_Error(Taget_Lost);
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
		Direction.sum[0] = 100 * ((1 * Road_Data[FRONT_RIGHT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed) - (1 * Road_Data[FRONT_LEFT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed)) / (1 * Road_Data[FRONT_LEFT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed + 1 * Road_Data[FRONT_RIGHT].AD_Value_fixed);				//差比和计算
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

	if (Direction.sum[0] > 0)
	{
		Direction.PIDbase.Error_Speed[Now_Error] = (Direction.sum[0] * 5 + 2 * Direction.sum[2]) / 7;
	}
	if (Direction.sum[0] < 0)
	{
		Direction.PIDbase.Error_Speed[Now_Error] = (Direction.sum[0] * 5 + 2 * Direction.sum[1]) / 7;
	}

#define more 40
#define less 15

	Direction.PIDbase.D = 4.5;
	Direction.PIDbase.P = 0.15;

	if (Direction.PIDbase.Error_Speed[Now_Error]<less && Direction.PIDbase.Error_Speed[Now_Error]>-less)
	{
		Direction.PIDbase.P *= 0.8;
		Direction.PIDbase.D *= 0.5;
	}
	if (Direction.PIDbase.Error_Speed[Now_Error] >  more)
	{
		Direction.PIDbase.P *= 1.35;
		Direction.PIDbase.D *= 1.5;
	}
	if (Direction.PIDbase.Error_Speed[Now_Error] < -more)
	{
		Direction.PIDbase.P *= 1.35;
		Direction.PIDbase.D *= 1.5;
	}
#undef more
#undef less

	Speed.Base.Aim_Speed = 15;

	/*if (Direction.err_Fixed > 20)
	{
	Speed.Base.Aim_Speed-=Direction.err_Fixed*0.04;
	}
	else if (Direction.err_Fixed < -20)
	{
	Speed.Base.Aim_Speed+=Direction.err_Fixed*0.04;
	}*/
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

	if (Direction.PIDbase.PID_Out_Speed >= 10)
		Direction.PIDbase.PID_Out_Speed = 10;
	else if (Direction.PIDbase.PID_Out_Speed <= -10)
		Direction.PIDbase.PID_Out_Speed = -10;

	Speed.Left.Turn_Speed = Direction.PIDbase.PID_Out_Speed;
	Speed.Right.Turn_Speed = -Direction.PIDbase.PID_Out_Speed;
}

/*============================================
函数名：eRule_Init_Fuzzy()
作用:模糊控制论域eRule初始化
==========================================*/

void eRule_Init_Fuzzy()
{
	Fuzzy_Direction.eRule[0].eAngle = 0.887;					//初始化角度相似规则
	Fuzzy_Direction.eRule[1].eAngle = 0;							//初始化角度相似规则
	Fuzzy_Direction.eRule[2].eAngle = 0;							//初始化角度相似规则
	Fuzzy_Direction.eRule[3].eAngle = 0;							//初始化角度相似规则
	Fuzzy_Direction.eRule[4].eAngle = 0;							//初始化角度相似规则
	Fuzzy_Direction.eRule[5].eAngle = 0;							//初始化角度相似规则
																	//注意修改MAX_FUZZY_RULE!

	Fuzzy_Direction.eRule[0].eLength = 97.76;				//初始化长度相似规则
	Fuzzy_Direction.eRule[1].eLength = 0;					//初始化长度相似规则
	Fuzzy_Direction.eRule[2].eLength = 0;					//初始化长度相似规则
	Fuzzy_Direction.eRule[3].eLength = 0;					//初始化长度相似规则
	Fuzzy_Direction.eRule[4].eLength = 0;					//初始化长度相似规则
	Fuzzy_Direction.eRule[5].eLength = 0;					//初始化长度相似规则
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
	eDenominator = sqrt(eDenominator);//需要研究更好的开方函数
	eDenominator *= 2.236;


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
	static  int16 e;
	led(LED2, LED_OFF);
	Fuzzy_Direction.isMatched--;
	if (Fuzzy_Direction.isMatched > 0)
	{
		Fuzzy_Direction.isMatched--;
	}
	else if (Fuzzy_Direction.isMatched < 0)
	{
		Fuzzy_Direction.isMatched = 0;
	}
	if (Fuzzy_Direction.isMatched == 0)
	{
		for (counter i = 0; i < MAX_FUZZY_RULE; i++)
		{
			if (Fuzzy_Direction.eGrade[i].eAngle < 0.05&&Fuzzy_Direction.eGrade[i].eAngle > -0.05)										//匹配相似度90%
			{

				if (0 == i)																					//根据匹配结果执行不同操作
				{
					if (Fuzzy_Direction.eGrade[i].eLength < 15 && Fuzzy_Direction.eGrade[i].eLength > -15)
					{
						e = 250 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
						led(LED2, LED_ON);
						Fuzzy_Direction.isMatched = 5;//匹配成功标记
					}
				}
				else if (1 == i)																			//根据匹配结果执行不同操作
				{
					if (Fuzzy_Direction.eGrade[i].eLength < 10 && Fuzzy_Direction.eGrade[i].eLength > -10)
					{
						e = 250 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
						Fuzzy_Direction.isMatched = 5;												//匹配成功标记
						led(LED2, LED_ON);
					}
				}
				else if (2 == i)																			//根据匹配结果执行不同操作
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (3 == i)																			//根据匹配结果执行不同操作
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (4 == i)																			//根据匹配结果执行不同操作
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (5 == i)																			//根据匹配结果执行不同操作
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
			}
		}
	}

	if (Fuzzy_Direction.isMatched)
	{
		Direction.PIDbase.Error_Speed[Now_Error] = e;
	}
}