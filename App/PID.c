#include<stdio.h>
#include<stdlib.h>
#include "include.h"
#include "common.h"
#include "PID.h"

struct _pid{
float SetSpeed; //�����趨ֵ
float ActualSpeed; //����ʵ��ֵ
float err; //����ƫ��ֵ
float err_next; //������һ��ƫ��ֵ
float err_last; //��������ǰ��ƫ��ֵ
float Kp,Ki,Kd; //������������֡�΢��ϵ��
float incrementSpeed; //������һ�ε��ٶȿ��Ʋ���
uint32 duty; //ռ�ձ�
}pid;
void PID_init_left(){
  pid.SetSpeed=0.0;
  pid.ActualSpeed=0.0;
  pid.err=0.0;
  pid.err_last=0.0;
  pid.err_next=0.0;
  pid.incrementSpeed=0;
  pid.Kp=5;
  pid.Ki=0;
  pid.Kd=10;
  pid.duty=100;
}
uint32 PID_realize_left(float speed,int16 actual_speed){
  
  pid.SetSpeed=speed;
  pid.ActualSpeed = (float)actual_speed; 
  pid.err=pid.SetSpeed-pid.ActualSpeed;
  float incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last); //�������

  pid.incrementSpeed += incrementSpeed;  //����ۼ�
  pid.err_last=pid.err_next;
  pid.err_next=pid.err;
  
  int k = 20;
  float err_increment = pid.incrementSpeed;
  float CON = err_increment*k;
  if(CON>1000){  //��ֹ̫������ռ�ձ�Ϊ��
  CON=999;
  }
  
  if (CON>=0){
    pid.duty = 1000-(uint32)CON;
  }
  else if (CON<0){
    CON = -CON;
    pid.duty = 1000+(uint32)CON;
  }
  if (pid.duty > 2000){ //����ռ�ձ������Сֵ
    pid.duty = 2000;
  }
  if (pid.duty < 10){
    pid.duty = 10;
  }

  
  return pid.duty;
  
}



struct _pid3{
float SetSpeed; //�����趨ֵ
float ActualSpeed; //����ʵ��ֵ
float err; //����ƫ��ֵ
float err_next; //������һ��ƫ��ֵ
float err_last; //��������ǰ��ƫ��ֵ
float Kp,Ki,Kd; //������������֡�΢��ϵ��
float incrementSpeed; //������һ�ε��ٶȿ��Ʋ���
uint32 duty; //ռ�ձ�
}pid3;
void PID_init_right(){
  pid3.SetSpeed=0.0;
  pid3.ActualSpeed=0.0;
  pid3.err=0.0;
  pid3.err_last=0.0;
  pid3.err_next=0.0;
  pid3.incrementSpeed=0;
  pid3.Kp=10;
  pid3.Ki=0;
  pid3.Kd=50;
  pid3.duty=100;
}
uint32 PID_realize_right(float speed,int16 actual_speed){
  
  pid3.SetSpeed=speed;
  pid3.ActualSpeed = (float)actual_speed; 
  pid3.err=pid3.SetSpeed-pid3.ActualSpeed;
  float incrementSpeed=pid3.Kp*(pid3.err-pid3.err_next)+pid3.Ki*pid3.err+pid3.Kd*(pid3.err-2*pid3.err_next+pid3.err_last); //�������

  pid3.incrementSpeed += incrementSpeed;  //����ۼ�
  pid3.err_last=pid3.err_next;
  pid3.err_next=pid3.err;
  
  int k = 20;
  float err_increment = pid3.incrementSpeed;
  float CON = err_increment*k;
  if(CON>1000){  //��ֹ̫������ռ�ձ�Ϊ��
  CON=999;
  }
  
  if (CON>=0){
    pid3.duty = 1000-(uint32)CON;
  }
  else if (CON<0){
    CON = -CON;
    pid3.duty = 1000+(uint32)CON;
  }
  if (pid3.duty > 2000){ //����ռ�ձ������Сֵ
    pid3.duty = 2000;
  }
  if (pid3.duty < 10){
    pid3.duty = 10;
  }

  
  return pid3.duty;
  
}



/*uint32 PID_realize(float speed,int16 actual_speed){  //������PID
  
  pid.SetSpeed=speed;
  pid.ActualSpeed = (float)actual_speed; 
  pid.err=pid.SetSpeed-pid.ActualSpeed;
  float incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last); //�������
  pid.err_last=pid.err_next;
  pid.err_next=pid.err;
  
  float outspeed  = incrementSpeed+(float)actual_speed;  //����ۼ�
  
  int k = 8;
  float CON = outspeed*k;
  if(CON>900){  //��ֹ̫������ռ�ձ�Ϊ��
  CON=895;
  }
  
  if (err_increment>=0){
    pid.duty = 900-(uint32)CON;
  }
  else if (err_increment<0){
    err_increment = -err_increment;
    pid.duty = 900+(uint32)CON;
  }
  if (pid.duty > 1000){ //����ռ�ձ������Сֵ
    pid.duty = 1000;
  }
  if (pid.duty < 500){
    pid.duty = 500;
  }

  
  return pid.duty;
  
}*/


struct _pid2{
float Target; //�����趨ֵ
float Encoder; //����ʵ��ֵ
float Bias; //����ƫ��ֵ
float Integral_bias; //������һ��ƫ��ֵ
float Last_Bias; //��������ǰ��ƫ��ֵ
float Position_KP,Position_Ki,Position_KD; //������������֡�΢��ϵ��
float incrementangle; //����ۼ�ֵ
}pid2;
void PID_init2(){
  pid2.Target=0.0;
  pid2.Encoder=0.0;
  pid2.Bias=0.0;
  pid2.Integral_bias=0.0;
  pid2.Last_Bias=0.0;
  
  pid2.Position_KP=0.3;
  pid2.Position_Ki=0;
  pid2.Position_KD=5;
  pid2.incrementangle=0;
}
uint32 Position_PID(float Encoder,int32 Target){
  pid2.Target = (float)Target;
  pid2.Encoder = Encoder;
  pid2.Bias = pid2.Target-pid2.Encoder;
  pid2.Integral_bias+=pid2.Bias;
  float Pwm = pid2.Position_KP*pid2.Bias/100+pid2.Position_KD*(pid2.Bias-pid2.Last_Bias)/100;
  pid2.Last_Bias = pid2.Bias;
  pid2.incrementangle+=Pwm;
  
  float ka = 3;
  float CONA = pid2.incrementangle*ka;
  uint32 dutya;
  //����ȡ��Ϊ������Ϊ��
  
  if(CONA>75){  //��ֹ̫������ռ�ձ�Ϊ��
  CONA=75;
  }

  
  if (CONA>=0){
    dutya = 75-(uint32)CONA;
  }
  else if (CONA<0){
    CONA = -CONA;
    dutya = 75+(uint32)CONA;
  }
  if (dutya > 100){ //����ռ�ձ������Сֵ  
    dutya = 100;
  }
  if (dutya < 50){
    dutya = 50;
  }
  
  return dutya;
}






float getitem(){
  return  pid.incrementSpeed*5;
}
