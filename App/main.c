/*!
 *
 * @file       main.c
 * @brief      第六届光电设计大赛主程序
 * @author     王保键、贾梦朝、王雅健
 * @version    v1.0
 * @date       2018/4/18-2018/6/25
 */


//TODO：


#include "common.h"
#include "include.h"
#include "math.h"
#include "car.h"
#include "PID.h"

/*a是上面提到的采集循环 循环一次a加1 b是进入检测不到图像处理部分的次数 c是检测到的跟b类似 
speed_flag是图像处理中返回的行数值 也就是通过图像的行数判断距离 jiance是起步检测 只第一次有效 jiance_x是第一次检测到图像 仅第一次有效*/

//全局变量声明
/*******************************************************************************************************************************************************************/
uint32 a=0; // 无光开始寻灯的阈值
uint32 state_start=1;//起步检测 
uint32 state = 1;//1：寻灯状态 2：去找灯的状态 3：停车灭灯状态 4：灭灯完成状态     1→2→3→4→1·····
uint32 speed_flag=0;//图像所在的行数（用于判断距离）

uint32 b=0,c=0;

uint32 d = 0;  //用于防止小车倒车后再放下盖板


volatile int16 code=0;                          //编码器值
uint32 testing_left=90,testing_right=90;        //转弯速度 可以通过蓝牙调节 


float forespeed = 10.0;

volatile int32 angle;                           //图像处理完返回的角度
uint8 imgbuff[CAMERA_SIZE];                     //定义存储接收图像的数组
uint8 img[CAMERA_W*CAMERA_H];                   //解压后的60*80的数组
int16 actual_speed;                             //单电机时的编码器速度
int16 actual_speed_left;                        //左电机编码器速度
int16 actual_speed_right;                       //右电机编码器速度
float SpeedA;                                   //左电机速度
float SpeedB;                                   //右电机速度
float Servo;								
int backcount = 0; //倒退时间计时器

/*******************************************************************************************************************************************************************/

//变量定义
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

//滑行模式下，频率应该是 30~100。
//常规模式下，频率应该是 20k 左右
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (10*1000) //双电机频率为10K
#endif
/*******************************************************************************************************************************************************************/

//函数声明
/*******************************************************************************************************************************************************************/
void PORTA_IRQHandler();
//void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void Find();  
void Stop();
void Carcontrol(int angle);                 //小车整体控制函数
void Find(int type,int angle);              //寻灯函数
void Speedcontrol(float v,float angle);     //双电机差速函数
void MotorControl(int a,int b,int c,int d);  //电机PWM控制函数
/*******************************************************************************************************************************************************************/


/*!
 *  @brief      main函数
 *  @since      v6.0
 *  @note       山外LCD LCD显示
 */
  void  main(void)
{

    ASSERT(enter_init());   //由于英文字库和汉字字库都在FWD里，因此需要判断是否加载了FWD固件

    //摄像头
    camera_init(imgbuff);    //摄像头初始化
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler


    
    //中断优先级设置
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    NVIC_SetPriority(PORTA_IRQn,0);
    //NVIC_SetPriority(PIT0_IRQn,1);
    NVIC_SetPriority(PIT1_IRQn,1);
    
    //电机
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
 
    
    //舵机
    ftm_pwm_init(FTM1, DUOJI1_PWM, 50,125);  //初始化遮灯舵机
    ftm_pwm_init(FTM1, DUOJI2_PWM, 50,75);  //初始化转向舵机
    
    /*//左编码器
    ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化 //所用的管脚可查 port_cfg.h 的 FTM1_QDPHA_PIN 和 FTM1_QDPHB_PIN
    pit_init_ms(PIT0, 5);                                   //初始化PIT0，定时时间为： 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断*/
    
    //右编码器
    ftm_quad_init(FTM2);                                    //FTM1 正交解码初始化 //所用的管脚可查 port_cfg.h 的 FTM1_QDPHA_PIN 和 FTM1_QDPHB_PIN
    pit_init_ms(PIT1, 5);                                 //初始化PIT1，定时时间为： 5ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT1的中断服务函数为 PIT1_IRQHandler
    enable_irq (PIT1_IRQn);   
   
    //电机PID初始化
    PID_init_left();
    PID_init_right();
    
    //舵机PID初始化
    PID_init2();
    
    //显示屏
    
    LCD_init();   //(和舵机冲突，暂时关闭)   
    Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size; 
    size.H = CAMERA_H;      //真实显示
    size.W = CAMERA_W;
    
    //初始速度
    actual_speed_left = 0.0;
    actual_speed_right = 0.0;


    int setpeeds  = 5.0;
      while(1)
      {  


		        
         //主程序，需要角度计算、超声波测距和速度控制函数
         ContinuousImage(15);//图像采集 此函数解决了闪烁问题

         site.x = 0;
         site.y = 0;
         LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //显示图像

         angle=ImageProcessing(img,&speed_flag);//图像处理 返回角度
         Carcontrol(angle); 



		 site.y = 70;
         char buff[20];                                
         sprintf(buff,"%.1f",(float)angle);             
         LCD_str(site, buff, BLUE, RED);             //角度显示
         site.y = 110;
         LCD_num_C(site, speed_flag, BLUE, RED);           //整型数值显示
         site.x = 50;
         site.y = 110;
         LCD_num_C(site, state, BLUE, RED);           //整型数值显示

     
        }
      }
      




    /**  //LCD显示例程
    site.x = 0;
    site.y = 10;
    LCD_str(site, "Jack", BLUE, RED);               //英文显示

    site.y = 30;
    LCD_str(site, "Jill", BLUE, RED);          //英文显示


    site.y = 50;
    LCD_Str_CH(site, "朝朝是辣鸡", BLUE, RED);        //汉字显示

    site.y = 70;
    LCD_Str_ENCH(site, "键键是帅比", BLUE, RED);  //汉字英文混合显示

    while(1)
    {
        site.y = 90;
        f=f+0.1;
        sprintf(buff,"%.1f",f); // 变量转化为字符串
        LCD_str(site, buff, BLUE, RED);             //浮点数显示

        site.y = 110;
        LCD_num_C(site, i++, BLUE, RED);           //整型数值显示
    }
	**/

/*!PORTA中断服务函数
 *  @brief      PORTA中断服务函数
 *  @since      v6.0
 */
void PORTA_IRQHandler(){
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
    
#if 0                           //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      PIT0中断服务函数
 *  @since      v6.0
 */
/*
void PIT0_IRQHandler(void)
{
    actual_speed_left = -ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)(注意，速度和实际速度反向了，所以取了反)(换小车后，左边速度方向还是反的)
    ftm_quad_clean(FTM1);
    PIT_Flag_Clear(PIT0);       //清中断标志位
}
*/
/*!
 *  @brief      PIT1中断服务函数
 *  @since      v6.0
 */
void PIT1_IRQHandler(void)
{
    actual_speed_right = -ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)(注意，速度和实际速度反向了，所以取了反)（换小车后，右边速度方向又变回来了)
    actual_speed_left = actual_speed_right;
    ftm_quad_clean(FTM2);
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

//寻灯函数
void Find(int type,int angle) //寻找下一个目标的函数
 {  
   state = 1; //正在寻灯
   
   if (type==0)  //直接寻找灯（开机寻找灯）（被阻挡后寻灯）
   {
       ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 50);//左打死
       
       if(angle==200)
       {
         
         MotorControl(600,1000,600,1000);
         
         
         /*uint32 duty_speed_left = PID_realize_left(5,actual_speed_left);  //获取PID反馈值
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
   
   else if (type==1) //倒退寻找灯（灭灯后寻灯）
   {
     
  
     if (backcount < 4 ){
	MotorControl(1000,500,1000,500);
        DELAY_MS(500);
      }
     
   else
   {
      ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 50);//左打死
        
      if(angle==200)
      {
        /*
            uint32 duty_speed_left = PID_realize_left(5,actual_speed_left);  //获取PID反馈值
    
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



int Lighter()  //光敏传感器值
{
  
}

void Goback()  //倒车避让  //前面有障碍物，如果超过多少秒都不消失的话，就倒退，不然等待。
{

}

void Carcontrol(int angle)  //速度控制
{
    float Max_angle_left = 45; 
    float Max_angle_right = 105; 

    if(angle == 200)
    {
      a++;
    }
    
    if(state == 4)//灭完灯后倒退进行寻灯
    {
      ftm_pwm_duty(DUOJI_FTM, DUOJI1_PWM,125);//舵机遮灯
      Find(1,angle);
    }

    if(a >= 10)//4次循环都没有图像 进入无图像处理部分
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
          
          uint32 duty_angle = Position_PID(angle,0);//舵机PID返回PWM
          if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}  //设置占空比范围
          if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}
  
          if(speed_flag > 50 && d != 1) //开始准备刹车  TODO：测这个阈值
          {
            ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, 75);//转动舵机到目标
            forespeed = 0.0;	
            state = 3;
          }
          else if(speed_flag >= 3 && speed_flag <= 50)
          {       
            d = 0;
            ftm_pwm_duty(DUOJI_FTM, DUOJI2_PWM, duty_angle);//转动舵机到目标
            forespeed = 10.0;
          }
          
          if (state ==3 && actual_speed_left<1)
          {
            MotorControl(1000,1000,1000,1000);
            Stop();
          }
          
          
          uint32 duty_speed_left = PID_realize_left(forespeed,actual_speed_left);  //获取PID反馈值
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
  float T = 15.5;//车轴宽
  float L = 14.5;//车体长
  float K = 0.66667;//舵机参数
  float SERVO_INIT = 75; //初始值
  float turn_angle = ((SERVO_INIT - duty)/K/180)*PI;
  SpeedA = v*(1+T*tan(turn_angle)/2/L);
  SpeedB = v*(1-T*tan(turn_angle)/2/L);
 
}

void Stop()  //停车灭灯 
{
   ftm_pwm_duty(FTM1, DUOJI1_PWM,90);//舵机遮灯
   state = 4;
   DELAY_MS(2000);
}
