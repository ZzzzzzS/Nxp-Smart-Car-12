#include "include.h"
#include "data.h"

/*============================================
��������ADC_Init()
����:��ʼ��ADCģ��ת����
==========================================*/

void ADC_Init()
{
	adc_init(AMP1);													//��ʼ��AMP1ͨ����PTB0
	adc_init(AMP2);													//��ʼ��AMP2ͨ����PTB1
	//adc_init(AMP3);													//��ʼ��AMP3ͨ����PTB2
	//adc_init(AMP4);													//��ʼ��AMP4ͨ����PTB3
}

/*============================================
��������Get_MaxMin_AD_Value()
����:�ɼ�ADCģ��ת�������ص������Сֵ��
�����е�й�һ������
==========================================*/

/*============================================
ʲô�ǹ�һ��:
(ʵ�ʲɼ�ֵ - ��Сֵ)/(���ֵ - ��Сֵ)
����:��Ӧ��ͬ�ĳ���
==========================================*/

//��ȡ�����Сֵʱ�Ƿ���Ҫ�˲���
void Get_MaxMin_AD_Value()
{
	int i;
	int16 AD_Value_Temp[2] = { 0,0 };
	for (i = 0; i < 2; i++)											//��ʼ�������Сֵ
	{
		Road_Data[i].Min_AD_Value = DEFAULT_MIN_VALUE;
		Road_Data[i].Max_AD_Value = DEFAULT_MAX_VALUE;
	}

	while (1)														//������һֱ����ʱһֱ�ɼ������Сֵ������
	{
		AD_Value_Temp[LEFT] = adc_once(AMP1,ADC_8bit);				//�ɼ�����
		AD_Value_Temp[RIGHT] = adc_once(AMP2, ADC_8bit);
	
		for (i = 0; i < 2; i++)										//�Ƚϲ��滻�����Сֵ����
		{
			if (AD_Value_Temp[i] > Road_Data[i].Max_AD_Value)		//�Ƚ����ֵ
			{
				Road_Data[i].Max_AD_Value = AD_Value_Temp[i];
			}
			if (AD_Value_Temp[i] < Road_Data[i].Min_AD_Value)		//�Ƚ���Сֵ
			{
				Road_Data[i].Min_AD_Value = AD_Value_Temp[i];
			}
		}
		for (i = 0; i < 2; i++)										//�����һ���ķ�ĸ
		{
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}

		OLED_Normalization_Interface();								//OLED��ʾ��ǰ���ֵ
	}
}

/*============================================
��������ADC_Weight_Init()
����:��ʼ��Ȩ����ǰ�˲��㷨
==========================================*/
/*============================================
ʲô��Ȩ����ǰ�˲��㷨:
��֮ǰ�ɼ�����������Ȩƽ����
����:�Բɼ��������˲�����żȻ���
==========================================*/

void ADC_Weight_Init()
{
	char i, j;

	for (i = 0; i < 2; i++)											//��ǰ���εĵ��ֵ����ֵ������Ӱ��ǰ�����˲��������
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value_Old[j] = (Road_Data[i].Max_AD_Value + Road_Data[i].Min_AD_Value) / 2;
		}
	}

	for (i = 0; i < 2; i++)											//�˲�Ȩ�ر���ֵ
	{
		Road_Data[i].AD_Weight[0] = 1;
		Road_Data[i].AD_Weight[1] = 2;
		Road_Data[i].AD_Weight[2] = 3;
		Road_Data[i].AD_Weight[3] = 4;
	}
}

/*============================================
��������Get_AD_Value()
����:�ɼ�ADCģ��ת�������ص�����
==========================================*/
/*============================================
Ȩ����ǰ�˲�������˵����
    [0][1][2][3]
������        ������
��Ȩֵ        ��Ȩֵ
==========================================*/

void Get_AD_Value()
{
	char i, j;
	Road_Data[LEFT].AD_Value = adc_once(AMP1, ADC_8bit);	//�ɼ�����
	Road_Data[RIGHT].AD_Value = adc_once(AMP2, ADC_8bit);

//����Ϊ��ο����Ż���������ѧ��ϵ
	for (i = 0; i < 2; i++)
	{
		if (Road_Data[i].AD_Value > Road_Data[i].Max_AD_Value)		//����⵽�ĵ��ֵ�������ֵ�����������ֵ
		{
			Road_Data[i].Max_AD_Value = Road_Data[i].AD_Value;
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}
		if (Road_Data[i].AD_Value < Road_Data[i].Min_AD_Value)		//����⵽�ĵ��ֵС����Сֵ����������Сֵ
		{
			Road_Data[i].Min_AD_Value = Road_Data[i].AD_Value;
			Road_Data[i].normalization = Road_Data[i].Max_AD_Value - Road_Data[i].Min_AD_Value;
		}
		for (j = 0; j < 3; j++)										//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//���¶�������ֵ
		Road_Data[i].AD_Value = 0;									//��ղɼ���������
	}

	for (i = 0; i < 2; i++)											//װ��Ȩ����ǰ�˲���������ֵ
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
	}
	for (i = 0; i < 2; i++)											//װ���й�һ��������ֵ
	{
		Road_Data[i].Normalized_Value = (Road_Data[i].AD_Value - Road_Data[i].Min_AD_Value) / Road_Data[i].normalization;
	}
}

/*============================================
��������Direction_Control()
����:���Ʒ����ٶȵ�
==========================================*/

void Direction_Control()
{
	Direction.err = 0;												//����Ȩ���㷨�������
	Direction.err -= Road_Data[LEFT].Normalized_Value*LEFT_WEIGHT;
	Direction.err += Road_Data[RIGHT].Normalized_Value*RIGHT_WEIGHT;

	Left_Speed.Turn_Speed = Direction.err*DIRECTION_WEIGHT;
	Right_Speed.Turn_Speed = -Direction.err*DIRECTION_WEIGHT;

	//������Ƿ񽵵�Go_Speed

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//��������Ŀ���ٶ�
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}