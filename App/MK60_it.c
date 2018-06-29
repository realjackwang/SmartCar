/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.c
 * @brief      山外K60 平台中断服务函数
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */


#include  "include.h"
#include  "MK60_it.h"
/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTB_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTB_ISFR;
    PORTB_ISFR  = ~0;                                   //清中断标志位

    n = 16;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
       // Mid_Filter(imgbuff,  CAMERA_SIZE);
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}
/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 *  @author     xiong jin qi
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


