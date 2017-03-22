#include "include.h"
#include "common.h"

/*============================================
��������System_Init()
���ã���ʼ������ϵͳ
==========================================*/

void System_Init()
{
	DisableInterrupts;												//�궨�壬��ֹ�ж�
	flash_init();													//flash��ʼ��
	ADC_Init();														//ADCģ��ת������ʼ��
	//eRule_Init_Fuzzy();											//����ģ�����������ʼ��
	Init_Key();														//��ʼ������	
	Motor_Init();													//�����ʼ��
	Motor_PID_Init();												//���PID���Ƴ�ʼ��
	Get_Motor_Speed_Init();											//FTM���������ʼ��
	OLED_Init();													//OLED��ʼ��
	//Stop_Car_Init();												//ͣ������ʼ��
	lptmr_timing_ms(20);											//���õ͹��Ķ�ʱ����������ʼ����ʱ������Ϊ��ʱģʽ����λ:ms
	set_vector_handler(LPTMR_VECTORn, LPTMR_IRQHandler);			//��ϵͳ������Ҫ�жϺ������뵽�ж���������
	EnableInterrupts;												//�궨�壬�����ж�
	disable_irq(LPTMR_IRQn);										//�رյ͹��Ķ�ʱ�������ж�
}

/*============================================
��������Get_System_Ready()
���ã���ǰ׼��
==========================================*/

void Get_System_Ready()
{
	OLED_Interface();												//��ʼ�������ý���
	ADC_Weight_Init();												//��ʼ��Ȩ����ǰ�˲���
	enable_irq(LPTMR_IRQn);											//�����͹��Ķ�ʱ�������жϣ�׼������
}

/*============================================
�������� LPTMR_IRQHandler()
���ã�ϵͳ���Ƶ���Ҫ�������ж�
==========================================*/

void LPTMR_IRQHandler()
{
	Get_AD_Value();													//��ȡADC��ģת������ֵ
	Direction_Control();											//���Ŀ��ת��Ƕ�
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	//FuzzyPID();														//��PID����ģ������
	Motor_PID();													//�Ե����������ʽPID����
	Motor_Control();												//��������ٶ�
	Send_Data();
	LPTMR_Flag_Clear();												//����жϱ�־λ��׼����һ���ж�
        
	
}

/*============================================
��������Debug_Init()
���ã�����ģʽʱ��ʼ���������
==========================================*/

void Debug_Init()
{
//������ʱ����Ҫд
}



