/*!
 *
 * @file       main.c
 * @brief      ����������ƴ���������
 * @author     �����������γ������Ž�
 * @version    v1.0
 * @date       2018/4/18-2018/6/25
 */


//TODO��


#include "common.h"
#include "include.h"
#include "math.h"
#include "car.h"
#include "PID.h"

/*a�������ᵽ�Ĳɼ�ѭ�� ѭ��һ��a��1 b�ǽ����ⲻ��ͼ�����ֵĴ��� c�Ǽ�⵽�ĸ�b���� 
speed_flag��ͼ�����з��ص�����ֵ Ҳ����ͨ��ͼ��������жϾ��� jiance���𲽼�� ֻ��һ����Ч jiance_x�ǵ�һ�μ�⵽ͼ�� ����һ����Ч*/

//ȫ�ֱ�������
/*******************************************************************************************************************************************************************/
uint32 a=0; // �޹⿪ʼѰ�Ƶ���ֵ
uint32 state_start=1;//�𲽼�� 
uint32 state = 1;//1��Ѱ��״̬ 2��ȥ�ҵƵ�״̬ 3��ͣ�����״̬ 4��������״̬     1��2��3��4��1����������
uint32 speed_flag=0;//ͼ�����ڵ������������жϾ��룩

uint32 b=0,c=0;

uint32 d = 0;  //���ڷ�ֹС���������ٷ��¸ǰ�


volatile int16 code=0;                          //������ֵ
uint32 testing_left=90,testing_right=90;        //ת���ٶ� ����ͨ���������� 


float forespeed = 10.0;

volatile int32 angle;                           //ͼ�����귵�صĽǶ�
uint8 imgbuff[CAMERA_SIZE];                     //����洢����ͼ�������
uint8 img[CAMERA_W*CAMERA_H];                   //��ѹ���60*80������
int16 actual_speed;                             //�����ʱ�ı������ٶ�
int16 actual_speed_left;                        //�����������ٶ�
int16 actual_speed_right;                       //�ҵ���������ٶ�
float SpeedA;                                   //�����ٶ�
float SpeedB;                                   //�ҵ���ٶ�
float Servo;								
int backcount = 0; //����ʱ���ʱ��

/*******************************************************************************************************************************************************************/

//��������
/*******************************************************************************************************************************************************************/
#define TRIG    PTD2
#define ECHG    PTD1
#define FLAGWAIT    0xFFFFF


#define MOTOR_FTM   FTM0

#define MOTOR1_IO   PTD15
#define MOTOR2_IO   PTA19
#define MOTOR3_IO   PTA5
#define MOTOR4_IO   PTA24

#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH5
#define MOTOR4_PWM  FTM_CH6

#define MOTOR1_PWM_IO  FTM0_CH3
#define MOTOR2_PWM_IO  FTM0_CH4
#define MOTOR3_PWM_IO  FTM0_CH5
#define MOTOR4_PWM_IO  FTM0_CH6

#define DUOJI_FTM  FTM1
#define DUOJI1_PWM  FTM_CH0
#define DUOJI2_PWM  FTM_CH1


#define PI 3.14159265f 

//����ģʽ�£�Ƶ��Ӧ���� 30~100��
//����ģʽ�£�Ƶ��Ӧ���� 20k ����
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (10*1000) //˫���Ƶ��Ϊ10K
#endif
/*******************************************************************************************************************************************************************/

//��������
/*******************************************************************************************************************************************************************/
void PORTA_IRQHandler();
//void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void Find();  
void Stop();
void Carcontrol(int angle);                 //С��������ƺ���
void Find(int type,int angle);              //Ѱ�ƺ���
void Speedcontrol(float v,float angle);     //˫������ٺ���
void MotorControl(int a,int b,int c,int d);  //���PWM���ƺ���
/*******************************************************************************************************************************************************************/


/*!
 *  @brief      main����
 *  @since      v6.0
 *  @note       ɽ��LCD LCD��ʾ
 */
  void  main(void)
{

    ASSERT(enter_init());   //����Ӣ���ֿ�ͺ����ֿⶼ��FWD������Ҫ�ж��Ƿ������FWD�̼�

    //����ͷ
    camera_init(imgbuff);    //����ͷ��ʼ��
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler


    
    //�ж����ȼ�����
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    NVIC_SetPriority(PORTA_IRQn,0);
    //NVIC_SetPriority(PIT0_IRQn,1);
    NVIC_SetPriority(PIT1_IRQn,1);
    
    //���
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
 
    
    //���
    ftm_pwm_init(FTM1, DUOJI1_PWM, 50,125);  //��ʼ���ڵƶ��
    ftm_pwm_init(FTM1, DUOJI2_PWM, 50,75);  //��ʼ��ת����
    
    /*//�������
    ftm_quad_init(FTM1);                                    //FTM1 ���������ʼ�� //���õĹܽſɲ� port_cfg.h �� FTM1_QDPHA_PIN �� FTM1_QDPHB_PIN
    pit_init_ms(PIT0, 5);                                   //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�*/
    
    //�ұ�����
    ftm_quad_init(FTM2);                                    //FTM1 ���������ʼ�� //���õĹܽſɲ� port_cfg.h �� FTM1_QDPHA_PIN �� FTM1_QDPHB_PIN
    pit_init_ms(PIT1, 5);                                 //��ʼ��PIT1����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT1���жϷ�����Ϊ PIT1_IRQHandler
    enable_irq (PIT1_IRQn);   
   
    //���PID��ʼ��
    PID_init_left();
    PID_init_right();
    
    //���PID��ʼ��
    PID_init2();
    
    //��ʾ��
    
    LCD_init();   //(�Ͷ����ͻ����ʱ�ر�)   
    Site_t site     = {0, 0};                           //��ʾͼ�����Ͻ�λ��
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С
    Size_t size; 
    size.H = CAMERA_H;      //��ʵ��ʾ
    size.W = CAMERA_W;
    
    //��ʼ�ٶ�
    actual_speed_left = 0.0;
    actual_speed_right = 0.0;


    int setpeeds  = 5.0;
      while(1)
      {  


		        
         //��������Ҫ�Ƕȼ��㡢�����������ٶȿ��ƺ���
         ContinuousImage(15);//ͼ��ɼ� �˺����������˸����

         site.x = 0;
         site.y = 0;
         LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //��ʾͼ��

         angle=ImageProcessing(img,&speed_flag);//ͼ���� ���ؽǶ�
         Carcontrol(angle); 



		 site.y = 70;
         char buff[20];                                
         sprintf(buff,"%.1f",(float)angle);             
         LCD_str(site, buff, BLUE, RED);             //�Ƕ���ʾ
         site.y = 110;
         LCD_num_C(site, speed_flag, BLUE, RED);           //������ֵ��ʾ
         site.x = 50;
         site.y = 110;
         LCD_num_C(site, state, BLUE, RED);           //������ֵ��ʾ

     
        }
      }
      




    /**  //LCD��ʾ����
    site.x = 0;
    site.y = 10;
    LCD_str(site, "Jack", BLUE, RED);               //Ӣ����ʾ

    site.y = 30;
    LCD_str(site, "Jill", BLUE, RED);          //Ӣ����ʾ


    site.y = 50;
    LCD_Str_CH(site, "����������", BLUE, RED);        //������ʾ

    site.y = 70;
    LCD_Str_ENCH(site, "������˧��", BLUE, RED);  //����Ӣ�Ļ����ʾ

    while(1)
    {
        site.y = 90;
        f=f+0.1;
        sprintf(buff,"%.1f",f); // ����ת��Ϊ�ַ���
        LCD_str(site, buff, BLUE, RED);             //��������ʾ

        site.y = 110;
        LCD_num_C(site, i++, BLUE, RED);           //������ֵ��ʾ
    }
	**/

/*!PORTA�жϷ�����
 *  @brief      PORTA�жϷ�����
 *  @since      v6.0
 */
void PORTA_IRQHandler(){
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
    
#if 0                           //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      PIT0�жϷ�����
 *  @since      v6.0
 */
/*
void PIT0_IRQHandler(void)
{
    actual_speed_left = -ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)(ע�⣬�ٶȺ�ʵ���ٶȷ����ˣ�����ȡ�˷�)(��С��������ٶȷ����Ƿ���)
    ftm_quad_clean(FTM1);
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}
*/
/*!
 *  @brief      PIT1�жϷ�����
 *  @since      v6.0
 */
void PIT1_IRQHandler(void)
{
    actual_speed_right = -ftm_quad_get(FTM2);          //��ȡFTM �������� ��������(������ʾ������)(ע�⣬�ٶȺ�ʵ���ٶȷ����ˣ�����ȡ�˷�)����С�����ұ��ٶȷ����ֱ������)
    actual_speed_left = actual_speed_right;
    ftm_quad_clean(FTM2);
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

//Ѱ�ƺ���
void Find(int type,int angle) //Ѱ����һ��Ŀ��ĺ���
 {  
   state = 1; //����Ѱ��
   
   if (type==0)  //ֱ��Ѱ�ҵƣ�����Ѱ�ҵƣ������赲��Ѱ�ƣ�
   {
       ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 50);//�����
       
       if(angle==200)
       {
         
         MotorControl(600,1000,600,1000);
         
         
         /*uint32 duty_speed_left = PID_realize_left(5,actual_speed_left);  //��ȡPID����ֵ
	 if (duty_speed_left<1000 && duty_speed_left>=10)
	 {                                                                                  
            MotorControl(duty_speed_left,1000,duty_speed_left,1000);
	 }
	 if (duty_speed_left<=2000 && duty_speed_left>=1000)
	 {
           MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_left);
	 }*/
       }
       else
       {
         state = 2;
         a= 0; 
         PID_init_left();
         PID_init2();
       }
   }
   
   else if (type==1) //����Ѱ�ҵƣ���ƺ�Ѱ�ƣ�
   {
     
  
     if (backcount < 4 ){
	MotorControl(1000,500,1000,500);
        DELAY_MS(500);
      }
     
   else
   {
      ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 50);//�����
        
      if(angle==200)
      {
        /*
            uint32 duty_speed_left = PID_realize_left(5,actual_speed_left);  //��ȡPID����ֵ
    
            if (duty_speed_left<1000 && duty_speed_left>=10)
            {                                                                                  
              MotorControl(duty_speed_left,1000,duty_speed_left,1000);
            }
            if (duty_speed_left<=2000 && duty_speed_left>=1000)
            {
              MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_left);
            }
        */
    
      }
      else
      {
        MotorControl(600,1000,600,1000);
        state = 2;
        a= 0; 
        PID_init_left();
        PID_init2();d = 1;
      }
   }
   
   backcount ++;
   if (backcount>10){backcount = 10;}
   }
 }

void MotorControl(int a,int b,int c,int d)
{
   ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,b);
   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,a);
   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,c);
   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,d);
}



int Lighter()  //����������ֵ
{
  
}

void Goback()  //��������  //ǰ�����ϰ��������������붼����ʧ�Ļ����͵��ˣ���Ȼ�ȴ���
{

}

void Carcontrol(int angle)  //�ٶȿ���
{
    float Max_angle_left = 45; 
    float Max_angle_right = 105; 

    if(angle == 200)
    {
      a++;
    }
    
    if(state == 4)//����ƺ��˽���Ѱ��
    {
      ftm_pwm_duty(DUOJI_FTM, DUOJI1_PWM,125);//����ڵ�
      Find(1,angle);
    }

    if(a >= 10)//4��ѭ����û��ͼ�� ������ͼ������
    {       
      Find(0,angle); 
      if(a > 10)a=10;
    }
    else 
    {                    
	state_start = 0;
       
        if (angle !=200){backcount=0;}
        if (angle == 200)angle =0;
        
        if (state != 1 && state != 4)
        {
          
          uint32 duty_angle = Position_PID(angle,0);//���PID����PWM
          if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}  //����ռ�ձȷ�Χ
          if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}
  
          if(speed_flag > 50 && d != 1) //��ʼ׼��ɲ��  TODO���������ֵ
          {
            ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 75);//ת�������Ŀ��
            forespeed = 0.0;	
            state = 3;
          }
          else if(speed_flag >= 3 && speed_flag <= 50)
          {       
            d = 0;
            ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, duty_angle);//ת�������Ŀ��
            forespeed = 10.0;
          }
          
          if (state ==3 && actual_speed_left<1)
          {
            MotorControl(1000,1000,1000,1000);
            Stop();
          }
          
          
          uint32 duty_speed_left = PID_realize_left(forespeed,actual_speed_left);  //��ȡPID����ֵ
          if (duty_speed_left<1000 && duty_speed_left>=10)
          {                                                                                  
            MotorControl(duty_speed_left,1000,duty_speed_left,1000);
          }
          if (duty_speed_left<=2000 && duty_speed_left>=1000)
          {
            MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_left);
          }

        }

    
    }
}

void Speedcontrol(float v,float duty)
{
  float T = 15.5;//�����
  float L = 14.5;//���峤
  float K = 0.66667;//�������
  float SERVO_INIT = 75; //��ʼֵ
  float turn_angle = ((SERVO_INIT - duty)/K/180)*PI;
  SpeedA = v*(1+T*tan(turn_angle)/2/L);
  SpeedB = v*(1-T*tan(turn_angle)/2/L);
 
}

void Stop()  //ͣ����� 
{
   ftm_pwm_duty(FTM1, DUOJI1_PWM,90);//����ڵ�
   state = 4;
   DELAY_MS(2000);
}
