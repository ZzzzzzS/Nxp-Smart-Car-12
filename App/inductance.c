#include "include.h"
#include "data.h"

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
	adc_init(AD1);													//��ʼ��AMP1ͨ����PTB0
	adc_init(AD2);													//��ʼ��AMP2ͨ����PTB1
	adc_init(AD3);									 				//��ʼ��AMP3ͨ����PTB2
	adc_init(AD4);													//��ʼ��AMP4ͨ����PTB3
	adc_init(AD5);													//��ʼ��AMP5ͨ����PTB4
        adc_init(AD6);
}


/*============================================
��������Direction_Control()
����:ȫ�ֿ��Ʒ���
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
	Road_Data[LEFT].AD_Value = adc_once(AD1, ADC_8bit);					//�ɼ�����
	Road_Data[RIGHT].AD_Value = adc_once(AD2, ADC_8bit);				//�ɼ�����
	Road_Data[MIDDLE].AD_Value = adc_once(AD3, ADC_8bit);				//�ɼ�����
	Road_Data[FRONT_LEFT].AD_Value = adc_once(AD4, ADC_8bit);		//�ɼ�����
	Road_Data[FRONT_RIGHT].AD_Value = adc_once(AD5, ADC_8bit);	//�ɼ�����
	Road_Data[test].AD_Value = adc_once(AD6, ADC_8bit);	//�ɼ�����
	//ע���޸�ͨ����ʼ��
	//ע���޸�ͨ����ʼ��

	for (counter i = 0; i < AMP_MAX; i++)
	{
		for (counter j = 0; j < 2; j++)						//�Ե�����ݶ�����λ����
		{
			Road_Data[i].AD_Value_Old[j] = Road_Data[i].AD_Value_Old[j + 1];
		}
		Road_Data[i].AD_Value_Old[2] = Road_Data[i].AD_Value;		//���¶�������ֵ
	}

	for (counter i = 0; i < AMP_MAX; i++)						//װ��Ȩ����ǰ�˲���������ֵ
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

	if (Road_Data[RIGHT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed != 0)
	{
		Direction.sum[0] = 100 * ((1 * Road_Data[FRONT_RIGHT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed) - (1 * Road_Data[FRONT_LEFT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed)) / (1 * Road_Data[FRONT_LEFT].AD_Value_fixed + Road_Data[LEFT].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed + 1 * Road_Data[FRONT_RIGHT].AD_Value_fixed);				//��Ⱥͼ���
	}
	else
	{
		Direction.sum[0] = 0;
	}
	if (Road_Data[LEFT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed != 0)
	{
		Direction.sum[1] = 100 * (Road_Data[LEFT].AD_Value_fixed - Road_Data[MIDDLE].AD_Value_fixed) / (Road_Data[LEFT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed);			//��Ⱥͼ���
	}
	else
	{
		Direction.sum[1] = 0;
	}
	if (Road_Data[RIGHT].AD_Value_fixed + Road_Data[MIDDLE].AD_Value_fixed != 0)
	{
		Direction.sum[2] = 100 * (Road_Data[RIGHT].AD_Value_fixed - Road_Data[MIDDLE].AD_Value_fixed) / (Road_Data[MIDDLE].AD_Value_fixed + Road_Data[RIGHT].AD_Value_fixed);		//��Ⱥͼ���
	}
	else
	{
		Direction.sum[2] = 0;
	}

	Direction.sum[2] = Rk*Direction.sum[2] + Rb;
	Direction.sum[1] = Lk*Direction.sum[1] + Lb;

	Direction.err = (Direction.sum[0] * 5 + Direction.sum[1] + Direction.sum[2]) / 7;						//������������

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
��������Direction_PID()
����:����PID����
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
��������eRule_Init_Fuzzy()
����:ģ����������eRule��ʼ��
==========================================*/

void eRule_Init_Fuzzy()
{
	Fuzzy_Direction.eRule[0].eAngle = 0.887;					//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[1].eAngle = 0;							//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[2].eAngle = 0;							//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[3].eAngle = 0;							//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[4].eAngle = 0;							//��ʼ���Ƕ����ƹ���
	Fuzzy_Direction.eRule[5].eAngle = 0;							//��ʼ���Ƕ����ƹ���
																	//ע���޸�MAX_FUZZY_RULE!

	Fuzzy_Direction.eRule[0].eLength = 97.76;				//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[1].eLength = 0;					//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[2].eLength = 0;					//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[3].eLength = 0;					//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[4].eLength = 0;					//��ʼ���������ƹ���
	Fuzzy_Direction.eRule[5].eLength = 0;					//��ʼ���������ƹ���
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
	eDenominator = sqrt(eDenominator);//��Ҫ�о����õĿ�������
	eDenominator *= 2.236;


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
			if (Fuzzy_Direction.eGrade[i].eAngle < 0.05&&Fuzzy_Direction.eGrade[i].eAngle > -0.05)										//ƥ�����ƶ�90%
			{

				if (0 == i)																					//����ƥ����ִ�в�ͬ����
				{
					if (Fuzzy_Direction.eGrade[i].eLength < 15 && Fuzzy_Direction.eGrade[i].eLength > -15)
					{
						e = 250 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
						led(LED2, LED_ON);
						Fuzzy_Direction.isMatched = 5;//ƥ��ɹ����
					}
				}
				else if (1 == i)																			//����ƥ����ִ�в�ͬ����
				{
					if (Fuzzy_Direction.eGrade[i].eLength < 10 && Fuzzy_Direction.eGrade[i].eLength > -10)
					{
						e = 250 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
						Fuzzy_Direction.isMatched = 5;												//ƥ��ɹ����
						led(LED2, LED_ON);
					}
				}
				else if (2 == i)																			//����ƥ����ִ�в�ͬ����
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (3 == i)																			//����ƥ����ִ�в�ͬ����
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (4 == i)																			//����ƥ����ִ�в�ͬ����
				{
					e = 0 * (1 - Fuzzy_Direction.eGrade[i].eAngle);
				}
				else if (5 == i)																			//����ƥ����ִ�в�ͬ����
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