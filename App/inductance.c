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
	//adc_init(AMP5);													//��ʼ��AMP5ͨ����PTB4
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
	for (char i = 0; i < AMP_MAX; i++)											//�˲�Ȩ�ر���ֵ
	{
		//ע��:�޸�Ȩ��ֵʱ���޸�data.h->MAX_WEIGHT!
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
	//Road_Data[3].AD_Value = adc_once(AMP3, ADC_8bit);
	//Road_Data[4].AD_Value = adc_once(AMP4, ADC_8bit);
	//Road_Data[5].AD_Value = adc_once(AMP5, ADC_8bit);


	for (i = 0; i < AMP_MAX; i++)
	{
		for (j = 0; j < 3; j++)										//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//���¶�������ֵ
		Road_Data[i].AD_Value = 0;									//��ղɼ���������
	}

/*============================================
����:�����Ⱥ�
==========================================*/
/*============================================
ʲô�ǲ�Ⱥ��㷨:
(�ɼ�����ֵ)/(�����βɼ��ɼ���������ֵ)
����:�Բɼ��������˲�����żȻ���
==========================================*/

	Direction.Normalization_Value = 0;								//�����һ�β�Ⱥ͵ĺ���ֵ
	double temp;														//��ʱ������ֵ

	for (i = 0; i < AMP_MAX; i++)											//װ��Ȩ����ǰ�˲���������ֵ
	{
		for (j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
		Road_Data[i].AD_Value /= MAX_WEIGHT;
		Direction.Normalization_Value += Road_Data[i].AD_Value;		//���㱾�β�Ⱥ͵ĺ���ֵ
	}
	for (i = 0; i < AMP_MAX; i++)											//�����Ⱥͺ�ĵ��ֵ
	{
		temp = Road_Data[i].AD_Value;
		Road_Data[i].Normalized_Value = (int)((temp / (double)Direction.Normalization_Value) * 200.0);
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
	//AMP_MAX��Ҫ�޸ģ�����
	Left_Speed.Turn_Speed = Direction.err;
	Right_Speed.Turn_Speed = -Direction.err;

	//������Ƿ񽵵�Go_Speed

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//��������Ŀ���ٶ�
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}

/*============================================
��������Direction_Control_Fuzzy()
����:ģ�����������Ʒ���
==========================================*/

void Direction_Control_Fuzzy()
{
	//��ʱ����Ҫд
}
