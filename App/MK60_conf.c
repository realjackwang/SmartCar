/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_conf.c
 * @brief      ɽ��K60 ƽ̨���ù���ʵ���ļ�
 * @author     ɽ��Ƽ�
 * @version    v5.1
 * @date       2013-06-26
 */

#include    "common.h"
#include    "MK60_uart.h"
#include    "VCAN_KEY.h"
#include    "VCAN_LED.h"




/*!
 *  @brief      ����ʧ����ִ�еĺ���
 *  @param      file    �ļ�·����ַ
 *  @param      line    ����
 *  @since      v6.0
 *  Sample usage:       assert_failed(__FILE__, __LINE__);
 */
const char ASSERT_FAILED_STR[] = "����ʧ�ܣ���%s�ĵ�%d�С�������еĴ��룬���ݵĲ����쳣��\n";

void assert_failed(char *file, int line)
{
    led_init(LED2);
    while (1)
    {

        if(enter_init())
        {
            DEBUG_PRINTF(ASSERT_FAILED_STR, file, line);      //ͨ��������ʾ����ʧ��
        }

        //��ѭ���ȴ�����Ա���Ϊ�ζ���ʧ��
        led_turn(LED2);
        DELAY_MS(1000);

    }
}

/*!
 *  @brief      �ض���printf ������
 *  @param      ch      ��Ҫ��ӡ���ֽ�
 *  @param      stream  ������
 *  @since      v6.0
 *  @note       �˺�����FWD��printf������
 */
int enter_fputc(char ch)
{
    uart_putchar(VCAN_PORT, (char)ch);
    return(ch);
}




#include "VCAN_LCD.h"

/*!
 *  @brief      Ĭ���жϷ�����
 *  @since      v6.0
 *  @note       �˺���д���ж������������Ҫ�û�ִ��
 */
void default_isr(void)
{
    uint8_t tip_str[100];
    Site_t site     = {0, 0};
    Size_t size ;

    get_isr_info((char *)tip_str) ;

    led_init(LED1);

    LCD_INIT();                             //��ʼ��LCD

    size.W = LCD_W;
    size.H = LCD_H;

    LCD_rectangle(site, size, BCOLOUR);     //��ʼ������
     
    site.y=10;
    LCD_Str_ENCH(site,tip_str , FCOLOUR,BCOLOUR);

    while(1)
    {
        led_turn(LED1);

        DEBUG_PRINTF("%s", tip_str);
        DELAY_MS(1000);
    }
}

/* hard fault interrupt handler */
void hardfault_isr(void)
{
    //��������жϣ���ʾ�����ܷ��ˣ�ִ�����󣬻��ߵ���δ��ʼ����ģ�顣
    default_isr();
}
