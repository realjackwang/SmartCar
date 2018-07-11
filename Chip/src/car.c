#include "common.h"
#include "include.h"
#include "math.h"
#include "car.h"


/*ת�������ƺ���
 angle���Ƕ� ������ -35�ȵ�35��*/
int MotorActuator(int angle)
{
  uint32 val;
  switch(angle)
  {
      case -36:val=906;break;
      case -35:val=905;break;
      case -34:val=904;break;
      case -33:val=903;break;
      case -32:val=902;break;
      case -31:val=901;break;
      case -30:val=900;break;
      case -29:val=899;break;
      case -28:val=898;break;
      case -27:val=897;break;
      case -26:val=896;break;
      case -25:val=895;break;
      case -24:val=894;break;
      case -23:val=893;break;
      case -22:val=892;break;
      case -21:val=891;break;
      case -20:val=890;break;
      case -19:val=889;break;
      case -18:val=888;break;
      case -17:val=887;break;
      case -16:val=886;break;
      case -15:val=885;break;
      case -14:val=884;break;
      case -13:val=883;break;
      case -12:val=882;break;
      case -11:val=881;break;
      case -10:val=880;break;
      case -9:val=879;break;
      case -8:val=878;break;
      case -7:val=877;break;
      case -6:val=876;break;
      case -5:val=875;break;
      case -4:val=874;break;
      case -3:val=873;break;
      case -2:val=872;break;
      case -1:val=871;break;
      case 0:val=870;break;
      case 1:val=869;break;
      case 2:val=868;break;
      case 3:val=867;break;
      case 4:val=866;break;
      case 5:val=865;break;
      case 6:val=864;break;
      case 7:val=863;break;
      case 8:val=862;break;
      case 9:val=861;break;
      case 10:val=860;break;
      case 11:val=859;break;
      case 12:val=858;break;
      case 13:val=857;break;
      case 14:val=856;break;
      case 15:val=855;break;
      case 16:val=854;break;
      case 17:val=853;break;
      case 18:val=852;break;
      case 19:val=851;break;
      case 20:val=850;break;
      case 21:val=849;break;
      case 22:val=848;break;
      case 23:val=847;break;
      case 24:val=846;break;
      case 25:val=845;break;
      case 26:val=844;break;
      case 27:val=843;break;
      case 28:val=842;break;
      case 29:val=841;break;
      case 30:val=840;break;
      case 31:val=839;break;
      case 32:val=838;break;
      case 33:val=837;break;
      case 34:val=836;break;
      case 35:val=835;break;
     
      default:break;
  }
 
  return val;
 
 }

/*ҡͷ�������1*/
int Y_TA(int angle_x)
{
 
  if(angle_x==1)
  { 
   ftm_pwm_init(FTM1, FTM_CH1, 100,1000);
   DELAY_MS(100);
  }
  else if(angle_x==0){ftm_pwm_init(FTM1, FTM_CH1,100,1000); DELAY_MS(50);}
  return 0;

}
/*ҡͷ�����λ ��ǰ��д�� ������û�ã�*/
void Y_T(int n)
{
 int val=1550-(n*11);
 ftm_pwm_init(FTM1, FTM_CH1, 100,val);

}
/*����ͼ��ɼ�����(�����Ϻõ�����´���Ч���ܺã�������ڸ��ţ����ױ��㷨�ж�Ϊͼ��)
 */

void ContinuousImage(uint32 threshold)//����ÿ��ͼ����1�ĸ�������ֵ һ������ �ж�Ϊ��ͼ�� ����ѭ��
{
     
    uint32_t i;
    for(i=0;i<3;i++)
       {
          uint32 j=0,h=0;
          camera_get_img(); //����ͷ��ȡͼ��
          img_extract(img, (uint8 *)imgbuff,CAMERA_SIZE);  //��ѹͼ��
          
          for(h=0;h<4800;h++){if(img[h]==1){j++;}}
          
          if(j>threshold)
          {break;}
       }

}
/*�����ɼ�ͼ���������Ը��ݸ��Ų�����˸���˳�һ���ָ��ţ�����������Ź��󸲸���ͼ����Ҳû�а취����ǰ��д�� ������û�ã�*/

void AntijammingContinuousImage(uint32 thresholds)
{
    uint32_t i;
    uint8 string[19200];
    for(i=0;i<5;i++)
       {
          uint32 x=0,j=0,h=0;
          camera_get_img(); //����ͷ��ȡͼ��
          img_extract(img, imgbuff,CAMERA_SIZE);  //��ѹͼ��
          for(h=0;h<19200;h++){string[h]=img[h];}
          DELAY_MS(9);
          camera_get_img(); //����ͷ��ȡͼ��
          img_extract(img, imgbuff,CAMERA_SIZE);  //��ѹͼ��
          for(h=0;h<19200;h++){if(img[h]==1){j++;}if(string[h]==1){x++;}}
          if(abs(x-j)>thresholds){ for(h=0;h<19200;h++){img[h]=img[h]^string[h];}break;}
         }
      
}

/*�����������
ForwardSpeed����ǰת��
ReverseSpeed�����ת��
*/
void Motor(uint32 ForwardSpeed1,uint32 ForwardSpeed2,uint32 ReverseSpeed1,uint32 ReverseSpeed2)
{
  
  ftm_pwm_init(FTM0, FTM_CH0, 1000, ReverseSpeed2);

  ftm_pwm_init(FTM0, FTM_CH2, 1000, ForwardSpeed2);

  ftm_pwm_init(FTM0, FTM_CH1, 1000, ForwardSpeed1);

  ftm_pwm_init(FTM0, FTM_CH3, 1000, ReverseSpeed1);
}

/*���ݴ�����
*/
int ImageProcessing(uint8 *string,uint32 *m)//����Ҫ�Ĵ�ž�������� �ж�ͼ��λ�� Ȼ�󷵻ؽǶ�
{
    uint32 i,x,y,a,b,B=0,H=255;
    double IA;
    
    /*�˲�*/
     for(i=0;i<4800;i++)
    {
       
        if(string[i]==H&&string[i+1]==H&&string[i+2]==H&&i%(i/80)<58)
        {
            string[i]=H;
            string[i+1]=H;
            string[i+2]=H;
            i=i+2;}
         else {string[i]=B;}
     }
     
     
   
    for(i=80;i<4720;i++)//ȷ��ͼ��ĵ�һ��
        {

        if(string[i]==H&&string[i-80]==B&&string[i+1]==H&&string[i-79]==B&&string[i+2]==H&&string[i-78]==B&&string[i+80*1]==H&&string[i+1+80*2]==H)
        { a=i/79+1;break;}
        }

    for(i=80;i<4720;i++)//ȷ��ͼ������һ�� 
        {        
        if(string[i]==H&&string[i+80]==B&&string[i-80]==H&&string[i+1]==H&&string[i+81]==B&&string[i-79]==H&&string[i+2]==H&&string[i+82]==B&&string[i-78]==H&&string[i+1-80*2]==H)
         { b=i/79+1;}
        if(i>=4640 && string[i]==H&&string[i+80]==H&&string[i-80]==H&&string[i+1]==H&&string[i+81]==H&&string[i-79]==H&&string[i+2]==H&&string[i+82]==H&&string[i-78]==H&&string[i+1-80*2]==H){b = 60;}
        }

     
     
 
        x=(b-a)/2+a;//ͼ����м���
        a=0;b=0;
        if(x!=0)//ȷ���� 
          
        {  
         
          for(i=(x-1)*80;i<x*80;i++) //��һ��   
            { 
             if(string[i]==H&&string[i+1]==H&&string[i+2]==H&&string[i-1]==B){a=(i-((x-1)*80))+1;break;}
             
            }
             for(i=(x-1)*80;i<x*80;i++) //���һ��   
             {
               if(string[i]==H&&string[i+1]==B&&string[i-1]==H&&string[i-2]==H){b=(i-((x-1)*80))+1;}   
             }
        }

       y=(b-a)/2+a; //�м���
       a=0;b=0;
       if(x>0&&x<80){(*m)=x;}//����ͼ����м��� ���ڱ���
       if(x!=0&&y!=0)//ת���ɽǶ� �ͷ���ֵ
       {
         IA=((double)y-40)/(60-(double)x);
         x=0;y=0;
         IA= atan(IA)*180.0/3.14;// �Ƕ�ת����ʽ �Լ����
         if(IA<(-60)){IA=-60;}
         if(IA>(60)){IA=60;}
         return (int)IA;
       }  
      else{return 200;}    
}


/*������ʾ����*/
void SerialDisplay(uint8 *string)
{
  uint32 i;
   for(i=0;i<4800;i++)
       
       {
         printf("%d\r",string[i]);
         if(i%80==79){printf("\r\n");}
       }
       printf("                                      \r\n");
       printf("end\r\n");
}

/*���Ϻ�����ǰ��׼���Ӻ�����д�� Ч������ ����û�ã�*/
void Avoid(int16 code)
{
  uint32 detance_x=0,i;
  volatile uint16 var=0;
  for(i=0;i<3;i++)
      {
         
         var=adc_once(ADC1_SE5a,ADC_8bit);
         detance_x+=var;
      }
      detance_x=detance_x/3;
        if((detance_x>60)&&code>700){printf("              y2\r\n");Motor(20,20,0,0);MotorActuator(35);DELAY_MS(200);MotorActuator(-35);DELAY_MS(100);MotorActuator(0);}
   else if((detance_x>65)&&code>600){printf("              y1\r\n");Motor(20,20,0,0);MotorActuator(35);DELAY_MS(200);MotorActuator(-35);DELAY_MS(100);MotorActuator(0);}
   else if((detance_x>70)&&code>500){printf("              x2\r\n");Motor(20,20,0,0);MotorActuator(35);DELAY_MS(300);MotorActuator(-35);DELAY_MS(200);MotorActuator(0);}
   else if((detance_x>75)){printf("              x2\r\n");Motor(20,20,0,0);MotorActuator(35);DELAY_MS(300);MotorActuator(-35);DELAY_MS(200);MotorActuator(0);}

   
   

}
