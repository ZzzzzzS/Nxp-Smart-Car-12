//����������֧2���
//�µĲ�Ⱥ��㷨
#include "include.h"

void main()     
{
	System_Init();						//ϵͳ��ʼ��
	Get_System_Ready();			//׼������

	while (Service.isDebug==true)
	{
		Debug();						//����ģʽ
	}
}
