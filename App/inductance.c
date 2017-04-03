#include "include.h"
#include "data.h"

/*============================================
���˳��
	�� �� ���� ���� ��
==========================================*/

/*============================================
��������ADC_Init()
����:��ʼ��ADCģ��ת����
==========================================*/

void ADC_Init()
{
	adc_init(AMP1);													//��ʼ��AMP1ͨ����PTB0
	adc_init(AMP2);													//��ʼ��AMP2ͨ����PTB1
	adc_init(AMP3);									 				//��ʼ��AMP3ͨ����PTB2
	adc_init(AMP4);													//��ʼ��AMP4ͨ����PTB3
	adc_init(AMP5);													//��ʼ��AMP5ͨ����PTB4
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
	for (char i = 0; i < AMP_MAX; i++)								//�˲�Ȩ�ر���ֵ
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
	Road_Data[0].AD_Value = adc_once(AMP1, ADC_8bit);			//�ɼ�����
	Road_Data[1].AD_Value = adc_once(AMP2, ADC_8bit);
	Road_Data[2].AD_Value = adc_once(AMP3, ADC_8bit);
	Road_Data[3].AD_Value = adc_once(AMP4, ADC_8bit);
	Road_Data[4].AD_Value = adc_once(AMP5, ADC_8bit);


	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 3; j++)						//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//���¶�������ֵ
	}
//����:�����Ⱥ�
//ʲô�ǲ�Ⱥ��㷨:
//(�ɼ�����ֵ)/(�����βɼ��ɼ���������ֵ)
//����:�Բɼ��������˲�����żȻ���
	Direction.Normalization_Value = 0;								//�����һ�β�Ⱥ͵ĺ���ֵ
	double temp;													//��ʱ������ֵ

	for (counter i = 0; i < AMP_MAX; i++)						//װ��Ȩ����ǰ�˲���������ֵ
	{
		Road_Data[i].AD_Value_fixed = 0;
		for (counter j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value_fixed += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
		Road_Data[i].AD_Value_fixed /= MAX_WEIGHT;
		Direction.Normalization_Value += Road_Data[i].AD_Value_fixed;		//���㱾�β�Ⱥ͵ĺ���ֵ
	}
	for (counter i = 0; i < AMP_MAX; i++)						//�����Ⱥͺ�ĵ��ֵ
	{
		temp = Road_Data[i].AD_Value_fixed;
		Road_Data[i].Normalized_Value = (int)((temp / (double)Direction.Normalization_Value) * 100.0);
	}
}

/*============================================
��������Direction_Control()
����:���Ʒ���,�ٶȵ�
==========================================*/

void Direction_Control()
{
	Direction.err = 0;												//����Ȩ���㷨�������
	Direction.err -= Road_Data[0].Normalized_Value*LEFT_WEIGHT;
	Direction.err += Road_Data[1].Normalized_Value*RIGHT_WEIGHT;
	//AMP_MAX��Ҫ�޸ģ�����
	Left_Speed.Turn_Speed = Direction.err;
	Right_Speed.Turn_Speed = -Direction.err;

	//������Ƿ񽵵�Go_Speed

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//��������Ŀ���ٶ�
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}

/*============================================
��������eRule_Init_Fuzzy()
����:ģ����������eRule��ʼ��
==========================================*/

void eRule_Init_Fuzzy()
{
	Fuzzy_Direction.eRule[0].eAngle = 0;
	Fuzzy_Direction.eRule[1].eAngle = 0;
	Fuzzy_Direction.eRule[2].eAngle = 0;
	Fuzzy_Direction.eRule[3].eAngle = 0;
	Fuzzy_Direction.eRule[4].eAngle = 0;
	Fuzzy_Direction.eRule[5].eAngle = 0;
	//ע���޸�MAX_FUZZY_RULE!

	Fuzzy_Direction.eRule[0].eLength = 0;
	Fuzzy_Direction.eRule[1].eLength = 0;
	Fuzzy_Direction.eRule[2].eLength = 0;
	Fuzzy_Direction.eRule[3].eLength = 0;
	Fuzzy_Direction.eRule[4].eLength = 0;
	Fuzzy_Direction.eRule[5].eLength = 0;
	//ע���޸�MAX_FUZZY_RULE!

}

/*============================================
��������Get_AD_Value_Fuzzy()
����:�ɼ�ADCģ��ת�������ص�����
==========================================*/
/*============================================
Ȩ����ǰ�˲�������˵����
[0][1][2][3]
������        ������
��Ȩֵ        ��Ȩֵ
==========================================*/
void Get_AD_Value_Fuzzy()
{
	Road_Data[0].AD_Value = adc_once(AMP1, ADC_8bit);				//�ɼ�����
	Road_Data[1].AD_Value = adc_once(AMP2, ADC_8bit);
	Road_Data[2].AD_Value = adc_once(AMP3, ADC_8bit);
	Road_Data[3].AD_Value = adc_once(AMP4, ADC_8bit);
	Road_Data[4].AD_Value = adc_once(AMP5, ADC_8bit);
	//ע���޸�ͨ����ʼ��

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 3; j++)						//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//���¶�������ֵ
		Road_Data[i].AD_Value = 0;									//��ղɼ���������
	}

	for (counter i = 0; i < AMP_MAX; i++)						//װ��Ȩ����ǰ�˲���������ֵ
	{
		for (counter j = 0; j < 4; j++)
		{
			Road_Data[i].AD_Value += Road_Data[i].AD_Value_Old[j] * Road_Data[i].AD_Weight[j];
		}
		Road_Data[i].AD_Value /= MAX_WEIGHT;
	}
}

/*============================================
��������Similarity_Count_Fuzzy()
����:ģ�����Ƽ���ģ��������
==========================================*/
/*============================================
����������������
���������ȣ�����һ��Nά�ռ�
ȡAMP_MAX�����ֵ����һ��Nά������
�������Nά��������֪Nά������COSֵ
����ʾ�ɼ�����ֵ��ģ�������е�ֵ�����̶�
==========================================*/

void Similarity_Count_Fuzzy()
{
	double eDenominator = 0;										//���ҷ�ĸ/��ʱ����
	double eNumerator = 0;											//���ҷ���/���ȼ�����ʱ����

	/*****���Ƕȼ���*****/
	for (counter j = 0; j < AMP_MAX; j++)						//�������ҷ���
	{
		eNumerator += Road_Data[j].AD_Value;
	}

	for (counter j = 0; j < AMP_MAX; j++)						//�������ҷ�ĸ
	{
			eDenominator += Road_Data[j].AD_Value * Road_Data[j].AD_Value;
	}
	eDenominator= sqrt(eDenominator);//��Ҫ�о����õĿ�������

	Fuzzy_Direction.Position.eAngle = eNumerator / eDenominator;	//������(1,1,1,1,...)�ļн�����ֵ

	/*****���ȼ���*****/
	for (counter j = 0; j < AMP_MAX; j++)						//������������
	{
		eNumerator += Road_Data[j].AD_Value * Road_Data[j].AD_Value;
	}
	Fuzzy_Direction.Position.eLength = sqrt(eNumerator);			//���������ĳ���
	/*****�����ȼ���*****/											
	for (counter j = 0; j < MAX_FUZZY_RULE; j++)				//��������������λ�ô��������ұ�
	{
		eDenominator = Fuzzy_Direction.Position.eAngle - Fuzzy_Direction.eRule[j].eAngle;
		if (eDenominator > Fuzzy_Direction.eGrade[0].eAngle)
		{
			Fuzzy_Direction.eGrade[MAX_FUZZY_COUNT_NUM-1].eAngle = eDenominator;
			Fuzzy_Direction.eGrade[MAX_FUZZY_COUNT_NUM - 1].eLength = Fuzzy_Direction.Position.eLength - Fuzzy_Direction.eRule[j].eLength;
			Fuzzy_Direction.eGrade[MAX_FUZZY_COUNT_NUM - 1].eValue = j;
		}
	}
	for (counter i = MAX_FUZZY_COUNT_NUM - 1 ; i > 0; --i)	//���������ȵ�i���λ��
	{
		for (counter j = 0; j < MAX_FUZZY_RULE; j++)			//ÿ�μ����������
		{
			eDenominator = Fuzzy_Direction.Position.eAngle - Fuzzy_Direction.eRule[j].eAngle;
			if ((eDenominator > Fuzzy_Direction.eGrade[i].eAngle) && (eDenominator < Fuzzy_Direction.eGrade[i + 1].eAngle))
			{
				Fuzzy_Direction.eGrade[i].eAngle = eDenominator;
				Fuzzy_Direction.eGrade[i].eLength = Fuzzy_Direction.Position.eLength - Fuzzy_Direction.eRule[j].eLength;
				Fuzzy_Direction.eGrade[i].eValue = j;
			}
		}
	}

}

/*============================================
��������Direction_Control_Fuzzy()
����:ģ�����Ƽ��㷽��,�ٶȵ�
==========================================*/

void Direction_Control_Fuzzy()
{
	char e=0;
	/*****���������������*****/
	for (counter i = 0; i < MAX_FUZZY_COUNT_NUM; i++)
	{
		if (0 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
		else if (1 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
		else if (2 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
		else if (3 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
		else if (4 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
		else if (5 == Fuzzy_Direction.eGrade[i].eValue)
		{
			e += 0 * Fuzzy_Direction.eGrade[i].eAngle * i;
		}
	}
	Left_Speed.Turn_Speed = e;
	Right_Speed.Turn_Speed = -e;

	Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//��������Ŀ���ٶ�
	Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
}
