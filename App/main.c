#include "include.h"

void main()     
{
	System_Init();						//ϵͳ��ʼ��
	Get_System_Ready();			//׼������
	while (Service.isDebug)
	{
		DELAY_MS(50);
			Debug();															//����ģʽ
	}
}
