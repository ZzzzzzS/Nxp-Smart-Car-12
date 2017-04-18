#include "include.h"
#include "data.h"

#define LEFT_WEIGHT				1						//����ת��Ȩ��
#define RIGHT_WEIGHT	        1						//����ת��Ȩ��

/*============================================
���˳��
	�� �� ���� ���� ��
==========================================*/

/*============================================
��������ADC_Init()
����:��ʼ��ADCģ��ת����,��ʼ��Ȩ���˲�
==========================================*/
/*============================================
ʲô��Ȩ����ǰ�˲��㷨:
��֮ǰ�ɼ�����������Ȩƽ����
����:�Բɼ��������˲�����żȻ���
==========================================*/

void ADC_Init()
{
	adc_init(AMP1);													//��ʼ��AMP1ͨ����PTB0
	adc_init(AMP2);													//��ʼ��AMP2ͨ����PTB1
	adc_init(AMP3);									 				//��ʼ��AMP3ͨ����PTB2
	adc_init(AMP4);													//��ʼ��AMP4ͨ����PTB3
	adc_init(AMP5);													//��ʼ��AMP5ͨ����PTB4

	for (counter i = 0; i < AMP_MAX; i++)				//�˲�Ȩ�ر���ֵ
	{
		//ע��:�޸�Ȩ��ֵʱ���޸�data.h->MAX_WEIGHT!
		Road_Data[i].AD_Weight[0] = 1;
		Road_Data[i].AD_Weight[1] = 2;
		Road_Data[i].AD_Weight[2] = 3;
		Road_Data[i].AD_Weight[3] = 4;
	}
}


/*============================================
��������Direction_Control()
����:ȫ�ֿ��Ʒ���
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
	Road_Data[0].AD_Value = adc_once(AMP1, ADC_8bit);				//�ɼ�����
	Road_Data[1].AD_Value = adc_once(AMP2, ADC_8bit);				//�ɼ�����
	Road_Data[2].AD_Value = adc_once(AMP3, ADC_8bit);				//�ɼ�����
	Road_Data[3].AD_Value = adc_once(AMP4, ADC_8bit);				//�ɼ�����
	Road_Data[4].AD_Value = adc_once(AMP5, ADC_8bit);				//�ɼ�����
	//ע���޸�ͨ����ʼ��

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 3; j++)						//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[3] = Road_Data[i].AD_Value;		//���¶�������ֵ
	}

	for (counter i = 0; i < AMP_MAX; i++)						//װ��Ȩ����ǰ�˲���������ֵ
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
��������Direction_Control()
����:���Ʒ���,�ٶȵ�
==========================================*/
/*============================================
����:�����Ⱥ�
ʲô�ǲ�Ⱥ��㷨:
(�ɼ�����ֵ)/(�����βɼ��ɼ���������ֵ)
����:�Բɼ��������˲�����żȻ���
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
	if (Direction.err < 0)		//�����жϣ���ֹƽ������������ʧ
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




	
	//Left_Speed.Turn_Speed = -sqrt(CharAbs(Direction.err*Direction.err*Direction.err))*flag;									//�������
	//Right_Speed.Turn_Speed = sqrt(CharAbs(Direction.err*Direction.err*Direction.err))*flag;


	Left_Speed.Turn_Speed = -Direction.err;
	Right_Speed.Turn_Speed = Direction.err;


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
	Fuzzy_Direction.eRule[0].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[1].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[2].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[3].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[4].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[5].eAngle = 0;					//��ʼ���Ƕ����ƹ���
	//ע���޸�MAX_FUZZY_RULE!

	Fuzzy_Direction.eRule[0].eLength = 0;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[1].eLength = 0;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[2].eLength = 0;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[3].eLength = 0;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[4].eLength = 0;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[5].eLength = 0;				//��ʼ���������ƹ���
	//ע���޸�MAX_FUZZY_RULE!

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
	float eDenominator = 0;														//���ҷ�ĸ/��ʱ����
	float eNumerator = 0;															//���ҷ���/���ȼ�����ʱ����

	/*****���Ƕȼ���*****/
	for (counter j = 0; j < AMP_MAX; j++)									//�������ҷ���
	{
		eNumerator += Road_Data[j].AD_Value_fixed;
	}

	for (counter j = 0; j < AMP_MAX; j++)									//�������ҷ�ĸ
	{
			eDenominator += Road_Data[j].AD_Value_fixed * Road_Data[j].AD_Value_fixed;
	}
	eDenominator= sqrt(eDenominator);//��Ҫ�о����õĿ�������

	Fuzzy_Direction.Position.eAngle = eNumerator / eDenominator;	//������(1,1,1,1,...)�ļн�����ֵ

	/*****���ȼ���*****/
	for (counter j = 0; j < AMP_MAX; j++)									//������������
	{
		eNumerator += Road_Data[j].AD_Value_fixed * Road_Data[j].AD_Value_fixed;
	}
	Fuzzy_Direction.Position.eLength = sqrt(eNumerator);			//���������ĳ���

	/*****�����ȼ���*****/											
	for (counter j = 0; j < MAX_FUZZY_RULE; j++)
	{
		Fuzzy_Direction.eGrade[j].eAngle = Fuzzy_Direction.Position.eAngle - Fuzzy_Direction.eRule[j].eAngle;
		Fuzzy_Direction.eGrade[j].eLength = Fuzzy_Direction.Position.eLength - Fuzzy_Direction.eRule[j].eLength;
	}
}

/*============================================
��������Direction_Control_Fuzzy()
����:ģ�����Ƽ��㷽��,�ٶȵ�
==========================================*/

void Direction_Control_Fuzzy()
{
	int16 e=0;
	Fuzzy_Direction.isMatched = false;													//���Ϊδƥ�䵽

	for (counter i = 0; i < MAX_FUZZY_RULE; i++)
	{
		if (Fuzzy_Direction.eGrade[i].eAngle < 0.1)										//ƥ�����ƶ�90%
		{
			Fuzzy_Direction.isMatched = true;												//ƥ��ɹ����
			if (0 == i)																					//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (1 == i)																			//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (2 == i)																			//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (3 == i)																			//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (4 == i)																			//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
			else if (5 == i)																			//����ƥ����ִ�в�ͬ����
			{
				e += 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
			}
		}
	}
	
	if (Fuzzy_Direction.isMatched)
	{
		Left_Speed.Turn_Speed = e;																					//�������
		Right_Speed.Turn_Speed = -e;

		Left_Speed.Aim_Speed = Left_Speed.Turn_Speed + Left_Speed.Go_Speed;			//��������Ŀ���ٶ�
		Right_Speed.Aim_Speed = Right_Speed.Turn_Speed + Right_Speed.Go_Speed;
	}
}