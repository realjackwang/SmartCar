/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_it.c
 * @brief      ɽ��K60 ƽ̨�жϷ�����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */


#include  "include.h"
#include  "MK60_it.h"
/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTB_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTB_ISFR;
    PORTB_ISFR  = ~0;                                   //���жϱ�־λ

    n = 16;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
       // Mid_Filter(imgbuff,  CAMERA_SIZE);
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}
/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 *  @author     xiong jin qi
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


