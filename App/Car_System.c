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
	Motor_Init();													//�����ʼ��
	Motor_PID_Init();												//���PID���Ƴ�ʼ��
	Get_Motor_Speed_Init();											//FTM���������ʼ��
	OLED_Init();													//OLED��ʼ��
	Init_Key();														//��ʼ������
	//Stop_Car_Init();												//ͣ������ʼ��
	lptmr_timing_ms(20);											//���õ͹��Ķ�ʱ����������ʼ����ʱ������Ϊ��ʱģʽ����λ:ms
	set_vector_handler(LPTMR_VECTORn, Main_Control_Interrupt);		//��ϵͳ������Ҫ�жϺ������뵽�ж���������
	EnableInterrupts;												//�궨�壬�����ж�
	disable_irq(LPTMR_IRQn);										//�رյ͹��Ķ�ʱ�������ж�
}

/*============================================
��������Get_System_Ready()
���ã���ǰ׼��
==========================================*/

void Get_System_Ready()
{
	/*OLED_Interface();												//��ʼ�������ý���
	if (Service.Debug == true)										//����ģʽ�������ճ����Ժ���ǰ��һ������
	{
		Debug_Init();												//����ģʽ��ʼ��

		uart_init(Bluetooth, 115200);//��ʱʹ���������				//�������ڳ�ʼ��
		Get_MaxMin_AD_Value();										//���е�й�һ��
		Save_Inductance();											//������ֵ��flash
		OLED_CLS();													//����ǰ�ر�OLED
	}
	else if (Service.Debug == false)								//��ʽ����ģʽ
	{
		load_Inductance();											//��ȡflash�еĵ��ֵ
	}
	ADC_Weight_Init();												//��ʼ��Ȩ����ǰ�˲���
	*/
	load_Inductance();
	ADC_Weight_Init();												//��ʼ��Ȩ����ǰ�˲���
	enable_irq(LPTMR_IRQn);											//�����͹��Ķ�ʱ�������жϣ�׼������
}

/*============================================
��������Main_Control_Interrupt()
���ã�ϵͳ���Ƶ���Ҫ�������ж�
==========================================*/

void Main_Control_Interrupt()
{
	Get_AD_Value();													//��ȡADC��ģת������ֵ
	Direction_Control();											//���Ŀ��ת��Ƕ�
	Get_Motor_Speed();												//��ȡFTM������������ɼ�����ֵ
	Motor_PID();													//�Ե����������ʽPID����
	Motor_Control();												//��������ٶ�
	Stop_Car();														//ͣ�����

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



