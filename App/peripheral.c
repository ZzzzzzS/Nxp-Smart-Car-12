#include "include.h"
#include "peripheral.h"

/*============================================
��������Stop_Car_Init()
���ã�ͣ������ʼ��
==========================================*/

void Stop_Car_Init()
{//����������ػ����½���
	port_init(CAR_STOP, ALT1 | IRQ_FALLING | PULLUP);			//��ʼ��ͣ���ɻɹܣ��½����ж�
	set_vector_handler(PORTE_VECTORn, Stop_Car);				//����ͣ���ж�����
	enable_irq(PORTE_IRQn);										//ʹ������
}

/*============================================
��������Stop_Car()
���ã�ͣ�����
==========================================*/

void Stop_Car()
{
	if (PORTA_ISFR & (1 << CAR_STOP_NUM))						//ȷ���жϹܽź�
	{
		PORTA_ISFR = (1 << CAR_STOP_NUM);						//����жϱ�־λ

		disable_irq(LPTMR_IRQn);								//��ֹ�͹��Ķ�ʱ�������ж�
		Right_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
		Left_Speed.Out_Speed = 0;								//�����ٶ�Ϊ0
		ftm_pwm_duty(FTM0, FTM_CH1, Right_Speed.Out_Speed);		//��������ת��
		ftm_pwm_duty(FTM0, FTM_CH2, Left_Speed.Out_Speed);		//��������ת��
	}
}

/*============================================
��������Send_Data()
���ã��������ڷ���
==========================================*/

void Send_Data()
{
	char Send_Temp[50];
	sprintf(Send_Temp, "%d %d %d %d", Road_Data[0].Normalized_Value, Road_Data[1].Normalized_Value, Road_Data[2].Normalized_Value, Road_Data[3].Normalized_Value);
	uart_putstr(Bluetooth, (uint8_t *)Send_Temp);				//��������
}

/*============================================
��������Save_Inductance()
���ã�flash��������ֵ
==========================================*/
/*============================================
flash����λ��: L=���ֵ S=��Сֵ
ƫ����:0  1  2  3  4  5  6  7
	   L0 L1 L2 L3 S0 S1 S2 S3
==========================================*/

void Save_Inductance()
{
	char i;
	flash_erase_sector(SECTOR_NUM);								//����flash������׼��д��

	for (i = 0; i < 4; i++)										//�����й�һ���ķ�ĸ
	{
		do
		{
			flash_write(SECTOR_NUM, i * 4, Road_Data[i].normalization);
		} while (flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i].normalization);	//�������֤�Ƿ񴢴���ȷ
	}

	for (i = 4; i < 8; i++)										//��������Сֵ
	{
		do
		{
			flash_write(SECTOR_NUM, i * 4, Road_Data[i-4].Min_AD_Value);
		} while (flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i - 4].Min_AD_Value);//�������֤�Ƿ񴢴���ȷ
	}
}

/*============================================
��������Save_Inductance()
���ã�flash��������ֵ
==========================================*/

void load_Inductance()
{
	char i;
	for (i = 0; i < 4; i++)
	{
		Road_Data[i].Max_AD_Value = flash_read(SECTOR_NUM, i * 4, int16);
	}
	for (i = 4; i < 8; i++)
	{
		Road_Data[i - 4].Min_AD_Value = flash_read(SECTOR_NUM, i * 4, int16);
	}
}

/*============================================
��������Key_Init()
���ã���ʼ������
==========================================*/

void Key_Init()
{
	key_init(KEY_U);
	key_init(KEY_D);
	key_init(KEY_R);
	Ket_init(KEY_L);
	key_init(KEY_A);
	key_init(KEY_B);
}

/*============================================
��������OLED_Interface()
���ã�OLED��ʾ����ǰ����
==========================================*/

void OLED_Interface()
{
	OLED_Print(15, 0, "��������ҵ��ѧ");
	OLED_Print(35, 2, "����У��");
	OLED_Print(15, 4, "718����ʵ����");
	OLED_Print(27, 6, "ţ�Ƴ�����");
	while (key_check(KEY_A) == KEY_UP);								//�ȴ���������																	//ȱ�پ�����Խ���
}

/*============================================
��������OLED_Normalization_Interface()
���ã�OLED��ʾ��й�һ���еĽ���
==========================================*/

void OLED_Normalization_Interface()
{
	char OLED_Temp[30];												//OLED��ʾʹ�õ���ʱ����
	OLED_CLS();														//���ǰ��������Ļ����ֹ�������
	OLED_Print(0, 0, "���ڻ�ȡ�����Сֵ");
	sprintf(OLED_Temp, "���%d %d %d %d", Road_Data[0].Max_AD_Value, Road_Data[1].Max_AD_Value, Road_Data[2].Max_AD_Value, Road_Data[3].Max_AD_Value);
	OLED_Print(0, 2, OLED_Temp);									//����ɼ��������ֵ
	sprintf(OLED_Temp, "��С%d %d %d %d", Road_Data[0].Min_AD_Value, Road_Data[1].Min_AD_Value, Road_Data[2].Min_AD_Value, Road_Data[3].Min_AD_Value);
	OLED_Print(0, 4, OLED_Temp);									//����ɼ�����С���ֵ
	sprintf(OLED_Temp, "��ǰ%d %d %d %d", Road_Data[0].AD_Value, Road_Data[1].AD_Value, Road_Data[2].AD_Value, Road_Data[3].AD_Value);
	OLED_Printf(0, 6, OLED_Temp);									//�����ǰ���ֵ
}