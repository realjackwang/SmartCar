#include "common.h"
#include "include.h"
#include  "ultrasonic.h"

#define   TX     PTE28_OUT
#define   amend  1.25 

uint32 Ultrasonic_us=0;           //������us����
float Ultrasonic_Distance;  //���������Ծ���
uint8 Ultrasonic_Flag=0;          //�Ƿ񳬳����Ծ���
uint16 Ultrasonic_mm;             //����mm
float Distance=0;

void   Ultrasonic_Init()
{
   gpio_init(PTE28, GPO,0);                     //��ʼ��TX�˿�
   gpio_Interrupt_init(PTA5, GPI_DOWN,RING); //���������ж�
   //enable_irq (91);                                //��PTE�ж�
}

void   Ultrasonic_Delay(uint32 n)  //��������ʱus������
{
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<13;j++);   //   13   100M   1us
                         //   28   180M   1us
                         //   32   200M   1us
  }
}

void   Ultrasonic_Trig()          //���������ͺ���
{
  TX=1;                      //800ms����һ��ģ��
  Ultrasonic_Delay(1000000); //��ʱ800ms
  TX=0;
}
float  Ultrasonic_Compute(uint32 us)      //������������뺯��
{
 float Distance=0;
  
  Distance=(float)((us*340)/20000);//*amend;
  
  return Distance;
}
/****************����**********************/
void  obstacle_avoid()
{
  if(Distance<100);
    
}
