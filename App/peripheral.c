#include "include.h"
#include "peripheral.h"

/*============================================
��������Stop_Car_Init()
���ã�ͣ������ʼ��
==========================================*/

/*void Stop_Car_Init()
{//����������ػ����½���
	port_init(CAR_STOP, ALT1 | IRQ_FALLING | PULLUP);			//��ʼ��ͣ���ɻɹܣ��½����ж�
	set_vector_handler(PORTE_VECTORn, Stop_Car);				//����ͣ���ж�����
	enable_irq(PORTE_IRQn);										//ʹ������
}*/

/*============================================
��������Stop_Car()
���ã�ͣ�����
==========================================*/

/*void Stop_Car()
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
}*/

/*============================================
��������Send_Data()
���ã��������ڷ���
==========================================*/

void Send_Data()
{
	if (Service.Debug == true)
	{
		char var[2];
		for (char i = 0; i < 2; i++)
		{
			var[i] = Road_Data[i].Normalized_Value;					//����λ�����͵�й�һ�����ֵ
		}
		vcan_sendware(var, sizeof(var));							//���͵���λ����ע�ⷢ��Э�飬���Ͷ˿�
		printf("Out_Speed %d %d ", Left_Speed.Out_Speed, Right_Speed.Out_Speed);
		printf("NowSpeed %hd %hd", Left_Speed.Now_Speed, Right_Speed.Now_Speed);
	}
	
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
	//flash_erase_sector(SECTOR_NUM);								//����flash������׼��д��

	for (i = 0; i < 4; i++)										//�����й�һ���ķ�ĸ
	{
		//do
		//{
			//flash_write(SECTOR_NUM, i * 4, Road_Data[i].normalization);
		//} while //(flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i].normalization);	//�������֤�Ƿ񴢴���ȷ
	}

	for (i = 4; i < 8; i++)										//��������Сֵ
	{
		//do
		//{
			//flash_write(SECTOR_NUM, i * 4, Road_Data[i-4].Min_AD_Value);
		//} while //(flash_read(SECTOR_NUM, i * 4, int16) != Road_Data[i - 4].Min_AD_Value);//�������֤�Ƿ񴢴���ȷ
	}
}

/*============================================
��������Save_Inductance()
���ã�flash��������ֵ
==========================================*/

void load_Inductance()
{
	//�����ٶ���Ϣ��
}

/*============================================
��������Key_Init()
���ã���ʼ������
==========================================*/

void Init_Key()
{
	key_init(KEY_U);
	key_init(KEY_D);
	key_init(KEY_R);
	key_init(KEY_L);
	key_init(KEY_A);
	key_init(KEY_B);
}

/*============================================
��������OLED_Interface()
���ã�OLED��ʾ����ǰ����
==========================================*/

void OLED_Interface()
{
	OLED_Print(0, 0, "���V0.1");
	OLED_Print(0, 2, "#########");
	OLED_Print(15, 4, "718����ʵ����");
	OLED_Print(27, 6, "ţ�Ƴ�����");


	Service.Debug = true;



	/*while (key_check(KEY_A) == KEY_UP);								//�ȴ���������
	OLED_CLS();														//����OLED
	OLED_Print(0, 0, "ѡ��ģʽ");
	OLED_Print(0, 2, "A����ģʽ");
	OLED_Print(0, 4, "B����ģʽ");
	while (key_check(KEY_B) == KEY_UP&&key_check(KEY_D) == KEY_UP);//�ȴ���������
	if (key_check(KEY_B) == KEY_DOWN)								//����KEY_B����Ϊ����ģʽ
	{
		Service.Debug = true;
	}
	else if (key_check(KEY_D) == KEY_DOWN)
	{
		Service.Debug = false;
	}*/
}