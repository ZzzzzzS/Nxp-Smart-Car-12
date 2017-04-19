/*#include "include.h"

void main()     
{
	System_Init();						//ϵͳ��ʼ��
	Get_System_Ready();			//׼������
	while (Service.isDebug)
	{
		DELAY_MS(20);
			Debug();								//����ģʽ
	}
}*/

/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 UD��ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v6.0
 * @date       2016-09-25
 */

#include "common.h"
#include "include.h"

void PORTE_IRQHandler();

/*!
 *  @brief      main����
 *  @since      v6.0
 *  @note       SPI ���� NRF24L01+
 */
void main(void)
{
    uint32 i=0;
    uint8 buff[DATA_PACKET];
    uint8 *str = "ok";

    printf("\n\n\n***********����ģ��NRF24L01+����************");

    while(!nrf_init())                  //��ʼ��NRF24L01+ ,�ȴ���ʼ���ɹ�Ϊֹ
    {
        printf("\n  NRF��MCU����ʧ�ܣ������¼����ߡ�\n");
    }
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    			//���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
    enable_irq(PORTE_IRQn);

    printf("\n      NRF��MCU���ӳɹ���\n");

    while(1)
    {
      buff[0]==3;
      buff[1]=252;
      for(unsigned char i=2;i<DATA_PACKET;i++)
        buff[i]=0;
        
        if(nrf_tx(buff,DATA_PACKET) == 1 )          //����һ�����ݰ���buff����Ϊ32�ֽڣ�
        {
            //�ȴ����͹����У��˴����Լ��봦������

            while(nrf_tx_state() == NRF_TXING);         //�ȴ��������

            if( NRF_TX_OK == nrf_tx_state () )
            {
                printf("\n���ͳɹ�:%d",i);
                i++;                                    //���ͳɹ����1������֤�Ƿ�©��
            }
            else
            {
                printf("\n����ʧ��:%d",i);
            }
        }
        else
        {
            printf("\n����ʧ��:%d",i);
        }
        DELAY_MS(10);
    }
}


