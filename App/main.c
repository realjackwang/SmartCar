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

uint32 b=0,c=0,speed_flag=0,jiance_x=0;
volatile int16 code=0;                          //������ֵ
uint32 testing_left=90,testing_right=90;        //ת���ٶ� ����ͨ���������� 

volatile int32 angle;                           //ͼ�����귵�صĽǶ�
uint8 imgbuff[CAMERA_SIZE];                     //����洢����ͼ�������
uint8 img[CAMERA_W*CAMERA_H];                   //��ѹ���60*80������
int16 actual_speed;                             //�����ʱ�ı������ٶ�
int16 actual_speed_left;                        //�����������ٶ�
int16 actual_speed_right;                       //�ҵ���������ٶ�
float SpeedA;                                   //�����ٶ�
float SpeedB;                                   //�ҵ���ٶ�
float Servo;    
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


#define DUOJI1_PWM  FTM_CH1
#define DUOJI2_PWM  FTM_CH2


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
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void Find();  
void Stop();
void Carcontrol(int angle);                 //С��������ƺ���
void Find(int type,int angle);              //Ѱ�ƺ���
void Speedcontrol(float v,float angle);     //˫������ٺ���
void MotorControl(int a,int b,int c,int d);  //���PWM���ƺ���
/*******************************************************************************************************************************************************************/


//���������Ʋ��� ������ͼ����ĽǶ���Ϣ
void CarDriving(uint32 angle)
 {
   if(angle==200){a++;}//200Ϊû��ͼ���ʱ��Ƕȷ���ֵ ��Ҫ����Ϊʲô��200 �ܵ��и�ֵ��
   if(a>=4)//4��ѭ����û��ͼ�� ������ͼ������
  {
    if(state_start==1){Y_TA(1); MotorActuator(-35);jiance_x=1;}//������ҡͷ���һ���Ƕȷ�����
   else
   { 
     a=0;
     
     b++;
     /*���䴦���� �����Ǻܺ�*/
     if(b<3)
     {
       if(code<500)b=3;
       if(code>1000){Motor(0,0,80,80);}
       else if(code>900){Motor(0,0,70,70);}
       else if(code>800){Motor(0,0,60,60);b++;}
       else if(code>700){Motor(0,0,50,50);b++;}
       MotorActuator(-36);
     }
      
       else {Motor(testing_left,testing_right,0,0); MotorActuator(-36);}
    
    
     }
    
    if(b>6&&code<100){Motor(0,0,70,70);MotorActuator(35);DELAY_MS(500); a=4; b=3;}//�����Զ����� ����������Ӵ򻬿�ת �Զ�������Ч
    if(a>10)a=10;//��ֹa���
    if(b>20)b=20;//ͬ��
    c=0;
   }
  
   if(angle!=200)//��⵽ͼ��Ĵ�����
{
  
  
  
       state_start=0;
      
       if(jiance_x==1){jiance_x=0;Y_TA(0);b=3;}//����ҡͷ����нǶȵ�ʱ���⵽ͼ��
      else
  {    
       a=0;
       b=0;
       c++;
       MotorActuator(angle);//�Ƕ�
       if(c<20){ Motor(90,90,0,0);}//���
       else if(c<40){ Motor(80,80,0,0);}
     
       if(speed_flag>=24){MotorActuator(35);if(code<700)DELAY_MS(70);a=3;c=0;}//���� ������������ʱ ��Ϊ�ٶȵ��˿���ײ��
       if(a>20&&code<100){Motor(0,0,70,70);MotorActuator(35);DELAY_MS(500); a=0;c=0;}//������ �ο���һ��
        if(c>300)c=300;//�����
        }
   }
   
 }


/*!
 *  @brief      main����
 *  @since      v6.0
 *  @note       ɽ��LCD LCD��ʾ
 */
 void  main(void)
{

    ASSERT(enter_init());   //����Ӣ���ֿ�ͺ����ֿⶼ��FWD������Ҫ�ж��Ƿ������FWD�̼�

    /*//����ͷ
    camera_init(imgbuff);    //����ͷ��ʼ��
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
*/

    
   /* //�ж����ȼ�����
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    NVIC_SetPriority(PORTA_IRQn,0);
    NVIC_SetPriority(PIT0_IRQn,1);
    NVIC_SetPriority(PIT1_IRQn,1);*/
    
    //���
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,1000);      //��ʼ�� ��� PWM
    
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
 
    /*//������
    gpio_init(TRIG,GPO,0);
    gpio_init(ECHG,GPI,0);*/
    
    /*//���
    ftm_pwm_init(FTM0, DUOJI1_PWM, 50,75);  //��ʼ�����*/
    
    //�������
    ftm_quad_init(FTM1);                                    //FTM1 ���������ʼ�� //���õĹܽſɲ� port_cfg.h �� FTM1_QDPHA_PIN �� FTM1_QDPHB_PIN
    pit_init_ms(PIT0, 5);                                   //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
    
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
   // Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С
  //  Size_t size; 
   // size.H = CAMERA_H;      //��ʵ��ʾ
  //  size.W = CAMERA_W;
    
    //��ʼ�ٶ�
    actual_speed = 0.0;
    int jjj = 0;
    float angelsss=0;
      while(1)
      {  
         //DELAY_MS(1500);
         //ftm_pwm_duty(FTM0, DUOJI1_PWM,45);//���ת��

          
         /*//����ͷ+�������
          
          float Max_angle_left = 45; 
          float Max_angle_right = 105; 
          
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,900);
	  ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000); 
          
          
          ContinuousImage(20);//ͼ��ɼ�
          angle = ImageProcessing(img,&speed_flag);//ͼ���� �����˽Ƕ�
          
          
         // site.y = 0;
         // LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //��ʾͼ��
         // site.y = 70;
         // char buff[20];                                
         // sprintf(buff,"%.1f",(float)angle);             
         // LCD_str(site, buff, BLUE, RED);             //�Ƕ���ʾ
          
          if (angle !=200){
          uint32 duty_angle = Position_PID(angle,0);
          
          Speedcontrol(20,duty_angle);
          
          if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}
          if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}
          
          ftm_pwm_duty(FTM0, DUOJI1_PWM,duty_angle);//���ת��

         //site.y = 90;
         // LCD_num_C(site, duty_angle, BLUE, RED);  //��ʾ�ٶ�
          }
         // site.y = 110;
         // float angleshow = (float)angle;
         // char show[5];
         // sprintf(show,"%.1f",angleshow);             // ����ת��Ϊ�ַ���*/
      
        /*//��������Ҫ�Ƕȼ��㡢�����������ٶȿ��ƺ���
        if(Sonar()<��ֵ)  //���������̫С��Ҫײǽ�ˣ����˹��
        {
          Goback();
        }
        else  //Ѱ�ҹ�Դ
        {
          ContinuousImage(20);//ͼ��ɼ� �˺����������˸����
          //site.y = 0;
          //LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //��ʾͼ��
          angle=ImageProcessing(img,&speed_flag);//ͼ���� �����˽Ƕ�
          Carcontrol(angle); //���ﲻ���Ǽ򵥵ļ���Ŷ
          DELAY_MS(200);
          if(angle==200)  //�ǶȲ����ڣ���Ҫ�ٴ�Ѱ��
          {
            Find(0)
          }
          else    //�ҵ�Ŀ�꣬��������
          {
            if(Sonar()<��ֵ||Light()>��ֵ)  //�ҵ���Դ�����
            {
              Stop();  
            }
            else
            {
              ftm_pwm_duty(FTM1, FTM_CH0, MotorActuator(angle);
            }
          }
        
       }
     */
        
        /*//���������
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,900);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000);
        */
  
        /*//PID��������ٲ���
        
        float setspeed = 30;
        int Max_speed = 700;
        
        uint32 duty_speed = PID_realize(setspeed,actual_speed);  //��ȡPID����ֵ
  
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed, BLUE, RED);           //ռ�ձ�
        site.x = 80;
        LCD_Str_ENCH(site, "ռ�ձ�", BLUE, RED); 
        
        if (duty_speed<1000 || duty_speed>Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty_speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000); 
        site.y = 70;
        site.x = 0;
        LCD_Str_ENCH(site, "PIDϵͳ����", BLUE, RED);        //����
        }
        
	else {
        site.y = 90;
        site.x = 0;
        LCD_Str_ENCH(site, "Ҫ�����", BLUE, RED);        //�ܷ�
        }
         
        site.y = 50;
        site.x = 0;
        char buff[20];                                
        sprintf(buff,"%.1f",getitem());             // ����ת��Ϊ�ַ���
        LCD_str(site, buff, BLUE, RED);             //��������ʾ
        site.x = 80;
        LCD_Str_ENCH(site, "����ۼ�", BLUE, RED); */
        
        /*//PID��������ٲ���
        spe = 20 + ssss * 0.001;
        ssss+=5;
        if (ssss>30000){ssss=30000;}
        site.x = 0;
        site.y = 100;
        char buff2[20];                                
        sprintf(buff2,"%.1f",spe);             // ����ת��Ϊ�ַ���
        LCD_str(site, buff2, BLUE, RED);             //��������ʾ
        
	uint32 duty_speed = PID_realize(spe,actual_speed);  //��ȡPID����ֵ
        int Max_speed = 700;
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed, BLUE, RED);           //ռ�ձ�
        site.x = 80;
        LCD_Str_ENCH(site, "ռ�ձ�", BLUE, RED); 
        
        if (duty_speed<1000 || duty_speed>Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty_speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000); 
        site.y = 70;
        site.x = 0;
        LCD_Str_ENCH(site, "PIDϵͳ����", BLUE, RED);        //����
        }
        
	else {
        site.y = 90;
        site.x = 0;
        LCD_Str_ENCH(site, "Ҫ�����", BLUE, RED);        //�ܷ�
        }
         
        site.y = 50;
        site.x = 0;
        char buff[20];                                
        sprintf(buff,"%.1f",getitem());             // ����ת��Ϊ�ַ���
        LCD_str(site, buff, BLUE, RED);             //��������ʾ
        site.x = 80;
        LCD_Str_ENCH(site, "����ۼ�", BLUE, RED);*/
        
        /*//����������
        if (actual_speed>0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, actual_speed, BLUE, RED);   //��ʵ�ٶ�
        site.x = 80;
        LCD_Str_ENCH(site, "r/5ms", BLUE, RED); 
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "ǰ��", BLUE, RED);  
        }
        if (actual_speed<0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, -actual_speed, BLUE, RED);   //��ʵ�ٶ�
        site.x = 80;
        LCD_Str_ENCH(site, "r/5ms", BLUE, RED); 
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "����", BLUE, RED);    
        }*/
        
        
      //  MotorControl(500,1000,500,1000);
        
        //˫����������
        if (actual_speed_left>0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, actual_speed_left, BLUE, RED);   //��ʵ�ٶ�
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "ǰ��", BLUE, RED);  
        }
        if (actual_speed_left<0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, -actual_speed_left, BLUE, RED);   //��ʵ�ٶ�

        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "����", BLUE, RED);    
        }
            
        if (actual_speed_right>0){
        site.y = 10;
        site.x = 80;
        LCD_num_C(site, actual_speed_right, BLUE, RED);   //��ʵ�ٶ�
        site.y = 0;
        site.x = 80;
        LCD_Str_CH(site, "ǰ��", BLUE, RED);  
        }
        if (actual_speed_right<0){
        site.y = 10;
        site.x = 80;
        LCD_num_C(site, -actual_speed_right, BLUE, RED);   //��ʵ�ٶ�
        site.y = 0;
        site.x = 80;
        LCD_Str_CH(site, "����", BLUE, RED);    
        }
          
        //PID˫����ֱ���Ʋ���
        
        
        
        float setspeed = 2;
        if (jjj>300){setspeed = 0;}
        site.x = 0;
        site.y = 50;
        LCD_num_C(site, jjj, BLUE, RED);           //ռ�ձ�
        int Max_speed = 10;
        
        float angletest = 75;
        Speedcontrol(setspeed,angletest);
        
        uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //��ȡPID����ֵ
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //��ȡPID����ֵ
        
  
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed_left, BLUE, RED);           //ռ�ձ�
        site.x = 80;
        LCD_Str_ENCH(site, "ռ�ձ�", BLUE, RED); 
        
        
        if (duty_speed_left<1000 && duty_speed_left>=Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,duty_speed_left);
	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, duty_speed_left, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "����", BLUE, RED);        //����
        }
        
        if (duty_speed_left<=2000 && duty_speed_left>=1000){
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000);
	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000-duty_speed_left); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, 2000-duty_speed_left, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "����", BLUE, RED);        //����
        }
        
        
        if (duty_speed_right<1000 && duty_speed_right>=Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,duty_speed_right);
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1000); 
        site.y = 90;
        site.x = 0;
        LCD_num_C(site, duty_speed_right, BLUE, RED); 
        site.x = 80;
        LCD_Str_ENCH(site, "�ҵ��", BLUE, RED);        //����
        }
        
        if (duty_speed_right<=2000 && duty_speed_right>=1000){
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,1000);
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000-duty_speed_right); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, 2000-duty_speed_right, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "�ҵ��", BLUE, RED);        //����
        }
        

     
       
       
        site.y = 110;
        site.x = 0; 
        char buff3[20]; 
        sprintf(buff3,"%.1f",getitem());             // ����ת��Ϊ�ַ���
        LCD_str(site, buff3, BLUE, RED);             //��������ʾ
        site.x = 80;
        LCD_Str_ENCH(site, "����ۼ�", BLUE, RED); 
        
    
        jjj++;
       
        /*//���Ե�������Эͬ����
          int angles[] = {35,34,31,30,29,27,25,20,15,10,5,0,-5,-6,-7,-8,-10,-6,-3,0,3,0};//����
          for (int i =0;i<sizeof(angles)-1;i++){//����
           // site.y = 90;
           // LCD_num_C(site, i, BLUE, RED);  //��ʾ�Ƕ�
           // site.y = 110;
            //LCD_num_C(site, angles[i], BLUE, RED);  //��ʾ�Ƕ�
	    Carcontrol(angles[i]); //���ﲻ���Ǽ򵥵ļ���Ŷ
            DELAY_MS(500);//����
          }
*/
        
        /*//PID�������
        
        float Max_angle_left = 45 ;
        float Max_angle_right = 105;
        
        //ftm_pwm_duty(FTM0, DUOJI1_PWM,900);//�������
       
        int32 setangle = 35;
        uint32 pwm = Position_PID(angelsss,setangle);
        angelsss++;*/
        
        
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
void PIT0_IRQHandler(void)
{
    actual_speed_left = -ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)(ע�⣬�ٶȺ�ʵ���ٶȷ����ˣ�����ȡ�˷�)(��С��������ٶȷ����Ƿ���)
    ftm_quad_clean(FTM1);
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

/*!
 *  @brief      PIT1�жϷ�����
 *  @since      v6.0
 */
void PIT1_IRQHandler(void)
{
    actual_speed_right = ftm_quad_get(FTM2);          //��ȡFTM �������� ��������(������ʾ������)(ע�⣬�ٶȺ�ʵ���ٶȷ����ˣ�����ȡ�˷�)����С�����ұ��ٶȷ����ֱ������)
    ftm_quad_clean(FTM2);
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

//Ѱ�ƺ���
void Find(int type,int angle) //Ѱ����һ��Ŀ��ĺ���
 {  
   state = 1; //����Ѱ��
   if (type==0)  //ֱ��Ѱ�ҵƣ�����Ѱ�ҵƣ������赲��Ѱ�ƣ�
   {
       ftm_pwm_duty(FTM0, DUOJI1_PWM, 45);//�����
       if(angle==200)
	   {
		Speedcontrol(4,45);
		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //��ȡPID����ֵ
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //��ȡPID����ֵ
        
		if (duty_speed_left<1000 && duty_speed_left>=10)
		{
			MotorControl(duty_speed_left,1000,duty_speed_right,1000);
		}
		if (duty_speed_left<=2000 && duty_speed_left>=1000)
		{
			MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_right);
		}

	   }
   }
   
   else if (type==1) //����Ѱ�ҵƣ���ƺ�Ѱ�ƣ�
   {
   for(int i=0;i<3;i++)//����
   {  
		MotorControl(1000,700,1000,700);
		DELAY_MS(500);//����
   }
    ftm_pwm_duty(FTM0, DUOJI1_PWM, 45);//�����
    

   if(angle==200)
   {
		Speedcontrol(4,45);
		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //��ȡPID����ֵ
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //��ȡPID����ֵ
        
		if (duty_speed_left<1000 && duty_speed_left>=10)
		{
			MotorControl(duty_speed_left,1000,duty_speed_right,1000);
		}
		if (duty_speed_left<=2000 && duty_speed_left>=1000)
		{
			MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_right);
		}
   }
   }
   
 }

void MotorControl(int a,int b,int c,int d)
{
   ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,b);
   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,a);
   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,c);
   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,d);
}

int Sonar()  //���������
{
        uint32 timevar;
        uint32 flag;
        flag = 0;
        gpio_set(TRIG,1);               //������������
        pit_delay_us(PIT1,15);
        gpio_set(TRIG,0);
        
        while(gpio_get(ECHG) == 0);             //�ȴ���ƽ��ߣ��͵�ƽһֱ�ȴ�
        pit_time_start  (PIT0);                 //��ʼ��ʱ
        while(gpio_get(ECHG) == 1)              //�ȴ���ƽ��ͣ��ߵ�ƽһֱ�ȴ�
        {
            flag++;
            if(flag >FLAGWAIT)
            {
                break;
            }
        };             
        
        timevar = pit_time_get_us    (PIT0);    //ֹͣ��ʱ����ȡ��ʱʱ��
        if(flag <FLAGWAIT )
        {
            timevar = timevar * 340 /2/1000;
                
            if(timevar > 5)
            {
                return(timevar);
            }
        }
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

    if( angle == 200 ){a++;}

    if(a>=4)//4��ѭ����û��ͼ�� ������ͼ������
	{
		if (state_start == 1) //С������ʱ����ʼѰ��
		{ 
			Find(0,angle); 
		}

	    if(state == 3)//����ƺ��˽���Ѱ��
		{
			state = 4;
			Find(1,angle);
		}
		else  //һ��״̬���Ʊ��𣬻��Ǳ��赲��Ѱ��
		{   
			Find(0,angle); 
		}

		if(a>10)a=10;
    }
  
    if( angle !=200 )
	{
		state_start = 0;

		uint32 duty_angle = Position_PID(-angle,0);//���PID����PWM
        if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}  //����ռ�ձȷ�Χ
        if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}

		if(speed_flag > 24) //��ʼ׼��ɲ��  TODO���������ֵ
		{
			Speedcontrol(0,duty_angle);//��PWM������ת��Ƕȣ��������������Ĳ���		
			state = 3;
		}
		else 
		{
			Speedcontrol(12,duty_angle);//��PWM������ת��Ƕȣ��������������Ĳ���
			state = 2;
		}

		ftm_pwm_duty(FTM0, FTM_CH0, duty_angle);//ת�������Ŀ��

		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //��ȡPID����ֵ
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //��ȡPID����ֵ
        
	    if (duty_speed_left<1000 && duty_speed_left>=10)
		{
			MotorControl(duty_speed_left,1000,duty_speed_right,1000);
		}
		if (duty_speed_left<=2000 && duty_speed_left>=1000)
		{
			MotorControl(1000,2000-duty_speed_left,1000,2000-duty_speed_right);
		}

		if (state = 3 && actual_speed_left<2 && actual_speed_right<2)
		{
			Stop();
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
  
}
