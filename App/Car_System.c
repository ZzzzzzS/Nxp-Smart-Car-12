#include "include.h"
#include "common.h"

/*============================================
��������System_Init()
���ã���ʼ������ϵͳ
==========================================*/

int stopflag;

void Init_System()
{
	DisableInterrupts;																			//�궨�壬��ֹ�ж�
	uart_init(Bluetooth, Bluetooth_Band);												//���ڳ�ʼ��
	ADC_Init();																					//ADCģ��ת������ʼ��
	Init_Key();																						//��ʼ������	
	Motor_Init();																					//�����ʼ��
	Motor_PID_Init();																			//���PID���Ƴ�ʼ��
	Get_Motor_Speed_Init();																//TPM�����ʼ��
	OLED_Init();																					//OLED��ʼ��																				//ͣ������ʼ��
	lptmr_timing_ms(2);																		//���õ͹��Ķ�ʱ����������ʼ����ʱ������Ϊ��ʱģʽ����λ:ms
	set_vector_handler(LPTMR_VECTORn, LPTMR_IRQHandler);			//��ϵͳ������Ҫ�жϺ������뵽�ж���������
	EnableInterrupts;																			//�궨�壬�����ж�
	disable_irq(LPTMR_IRQn);																//�رյ͹��Ķ�ʱ�������ж�
	stopflag = 0;
}

/*============================================
��������Set_User_Information()
���ã������û�����
==========================================*/

void Set_User_Information()
{
	if (Service.RunMode == SlowMode)
	{
		Service.BlueToothBase.Information.speed = 30;
		Service.BlueToothBase.Information.P = 0.4;
		Service.BlueToothBase.Information.D = 60;

		Service.BlueToothBase.Information.MaxSpeed = 80;
		Service.BlueToothBase.Information.MinSpeed = -60;
		Service.BlueToothBase.Information.ToroidTurnTimes = 30;
		Service.BlueToothBase.Information.ToroidSpeed = 80;
        Service.BlueToothBase.Information.StopTimes=100;
	}
	else if (Service.RunMode == FastMode)
	{
		Service.BlueToothBase.Information.speed =40 ;
		Service.BlueToothBase.Information.P = 0.4;
		Service.BlueToothBase.Information.D = 60;

		Service.BlueToothBase.Information.MaxSpeed = 80;
		Service.BlueToothBase.Information.MinSpeed = -80;
		Service.BlueToothBase.Information.ToroidTurnTimes = 30;
		Service.BlueToothBase.Information.ToroidSpeed = 80;
                Service.BlueToothBase.Information.StopTimes=120;
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
	DELAY_MS(2000);
    Stop_Car_Init();
	enable_irq(LPTMR_IRQn);											//�����͹��Ķ�ʱ�������жϣ�׼������
}

/*============================================
�������� LPTMR_IRQHandler()
���ã�ϵͳ���Ƶ���Ҫ�������ж�
==========================================*/

void LPTMR_IRQHandler()
{
  static unsigned char flag=0;
  flag++;
  if(flag>=25)
  {
    Get_Motor_Speed();
    flag=0;
  }
  if(Speed.Base.Now_Speed>100)
    led(LED0,LED_ON);
  else
    led(LED0,LED_OFF);
    Direction_Control();
    Speed_Control();
	stopflag++;
	if (stopflag > 2500)
	{
		Stop_Car();
		stopflag = 2600;
	}
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
		Send_Data();
		//Receive_Data();
		break;

	case inductance_Mode:
		DeBug_Interface();
		LED_Interface();
		Send_Data();
		Receive_Data();
		break;

	case SlowMode:
		Send_Data();
		//Receive_Data();
		break;

	default:
		Service.RunMode = inductance_Mode;
		System_Error(No_Mode);
		break;
	}
}