#include "common.h"
#include "include.h"
#include  "ultrasonic.h"

#define   TX     PTE28_OUT
#define   amend  1.25 

uint32 Ultrasonic_us=0;           //超声波us计数
float Ultrasonic_Distance;  //超声波测试距离
uint8 Ultrasonic_Flag=0;          //是否超出测试距离
uint16 Ultrasonic_mm;             //距离mm
float Distance=0;

void   Ultrasonic_Init()
{
   gpio_init(PTE28, GPO,0);                     //初始化TX端口
   gpio_Interrupt_init(PTA5, GPI_DOWN,RING); //开启接收中断
   //enable_irq (91);                                //开PTE中断
}

void   Ultrasonic_Delay(uint32 n)  //超声波延时us级函数
{
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<13;j++);   //   13   100M   1us
                         //   28   180M   1us
                         //   32   200M   1us
  }
}

void   Ultrasonic_Trig()          //超声波发送函数
{
  TX=1;                      //800ms启动一次模块
  Ultrasonic_Delay(1000000); //延时800ms
  TX=0;
}
float  Ultrasonic_Compute(uint32 us)      //超声波计算距离函数
{
 float Distance=0;
  
  Distance=(float)((us*340)/20000);//*amend;
  
  return Distance;
}
/****************避障**********************/
void  obstacle_avoid()
{
  if(Distance<100);
    
}
