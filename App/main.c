//����������֧
#include "include.h"

void main()
{
	System_Init();
	Get_System_Ready();
	while (true)
	{
		if (Service.Debug == true)
		{
			//Send_Data();
		}
                
                	Get_AD_Value();													//��ȡADC��ģת������ֵ
	Direction_Control();											//���Ŀ��ת��Ƕ�
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	Motor_PID();													//�Ե����������ʽPID����
	Motor_Control();												//��������ٶ�
	//Stop_Car();														//ͣ�����

	//LPTMR_Flag_Clear();												//����жϱ�־λ��׼����һ���ж�
	}
}