#ifndef _LANDZO_ULTRASONIC_H
#define _LANDZO_ULTRASONIC_H

//extern uint32 Ultrasonic_us=0;           //������us����
//extern float Ultrasonic_Distance;  //���������Ծ���
//extern uint8 Ultrasonic_Flag=0;          //�Ƿ񳬳����Ծ���
//extern uint16 Ultrasonic_mm;             //����mm


void   Ultrasonic_Init();          //��������ʼ��
void   Ultrasonic_Delay(uint32 n);    //��������ʱus������
void   Ultrasonic_Trig();          //���������ͺ���
float  Ultrasonic_Compute(uint32 us);       //������������뺯��
#endif