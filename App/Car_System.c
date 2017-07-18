#include "include.h"
#include "common.h"

/*============================================
��������System_Init()
���ã���ʼ������ϵͳ
==========================================*/

void Init_System()
{
	DisableInterrupts;																			//�궨�壬��ֹ�ж�
	uart_init(Bluetooth, Bluetooth_Band);												//���ڳ�ʼ��
	ADC_Init();																					//ADCģ��ת������ʼ��
	Init_Key();																						//��ʼ������	
	Motor_Init();																					//�����ʼ��
	Motor_PID_Init();																			//���PID���Ƴ�ʼ��
	Get_Motor_Speed_Init();																//TPM�����ʼ��
	OLED_Init();																					//OLED��ʼ��
	Stop_Car_Init();																				//ͣ������ʼ��
	lptmr_timing_ms(5);																		//���õ͹��Ķ�ʱ����������ʼ����ʱ������Ϊ��ʱģʽ����λ:ms
	set_vector_handler(LPTMR_VECTORn, LPTMR_IRQHandler);			//��ϵͳ������Ҫ�жϺ������뵽�ж���������
	EnableInterrupts;																			//�궨�壬�����ж�
	disable_irq(LPTMR_IRQn);																//�رյ͹��Ķ�ʱ�������ж�
}

/*============================================
��������Set_User_Information()
���ã������û�����
==========================================*/

void Set_User_Information()
{
	if (Service.RunMode == FastMode)
	{
		Service.BlueToothBase.Information.speed = 30;
		Service.BlueToothBase.Information.P = 0.6;
		Service.BlueToothBase.Information.D = 10;
	}
	else if (Service.RunMode == SlowMode)
	{
		Service.BlueToothBase.Information.speed = 20;
		Service.BlueToothBase.Information.P = 0.3;
		Service.BlueToothBase.Information.D = 20;
	}
	
}

/*============================================
��������Get_System_Ready()
���ã���ǰ׼��
==========================================*/

void Get_System_Ready()
{
	OLED_Interface();														//��ʼ�������ý���
	Set_User_Information();												//�����û�����
	enable_irq(LPTMR_IRQn);											//�����͹��Ķ�ʱ�������жϣ�׼������
}

/*============================================
�������� LPTMR_IRQHandler()
���ã�ϵͳ���Ƶ���Ҫ�������ж�
==========================================*/

void LPTMR_IRQHandler()
{
	count++;
	Direction_Control();
	Speed_Control();
	//Stop_Car();
    LPTMR_Flag_Clear();												//����жϱ�־λ��׼����һ���ж�
}

/*============================================
��������system_RunTime_Update()
���ã�ϵͳ����ʱִ�еĲ���
==========================================*/

void system_RunTime_Update()
{
	DELAY_MS(200);													//���ٺ���ִ��һ�Σ���ֹˢ������

	switch (Service.RunMode)										//�жϲ�ͬģʽִ�в�ͬ����
	{
	case FastMode:
		if (Service.OLEDbase.OLED_Renew >= 10)			//���³�ʼ����Ļ,һ���̶ȷ�ֹ����
		{
			//OLED_Init();
			Service.OLEDbase.OLED_Renew = 0;
		}
		else
		{
			Service.OLEDbase.OLED_Renew++;
		}
		DeBug_Interface();
		LED_Interface();
		Send_Data();
		Receive_Data();
		break;

	case inductance_Mode:
		DeBug_Interface();
		LED_Interface();
		Send_Data();
		Receive_Data();
		break;

	case SlowMode:
		if (Service.OLEDbase.OLED_Renew >= 10)			//���³�ʼ����Ļ,һ���̶ȷ�ֹ����
		{
			//OLED_Init();
			Service.OLEDbase.OLED_Renew = 0;
		}
		else
		{
			Service.OLEDbase.OLED_Renew++;
		}
		DeBug_Interface();
		LED_Interface();
		Send_Data();
		Receive_Data();
		break;

	default:
		Service.RunMode = inductance_Mode;
		System_Error(No_Mode);
		break;
	}
}