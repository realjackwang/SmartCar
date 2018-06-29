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

uint32 b=0,c=0,speed_flag=0,jiance_x=0;
volatile int16 code=0;                          //编码器值
uint32 testing_left=90,testing_right=90;        //转弯速度 可以通过蓝牙调节 

volatile int32 angle;                           //图像处理完返回的角度
uint8 imgbuff[CAMERA_SIZE];                     //定义存储接收图像的数组
uint8 img[CAMERA_W*CAMERA_H];                   //解压后的60*80的数组
int16 actual_speed;                             //单电机时的编码器速度
int16 actual_speed_left;                        //左电机编码器速度
int16 actual_speed_right;                       //右电机编码器速度
float SpeedA;                                   //左电机速度
float SpeedB;                                   //右电机速度
float Servo;    
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


#define DUOJI1_PWM  FTM_CH1
#define DUOJI2_PWM  FTM_CH2


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
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void Find();  
void Stop();
void Carcontrol(int angle);                 //小车整体控制函数
void Find(int type,int angle);              //寻灯函数
void Speedcontrol(float v,float angle);     //双电机差速函数
void MotorControl(int a,int b,int c,int d);  //电机PWM控制函数
/*******************************************************************************************************************************************************************/


//舵机电机控制部分 传入了图像处理的角度信息
void CarDriving(uint32 angle)
 {
   if(angle==200){a++;}//200为没有图像的时候角度返回值 不要问我为什么是200 总得有个值吧
   if(a>=4)//4次循环都没有图像 进入无图像处理部分
  {
    if(state_start==1){Y_TA(1); MotorActuator(-35);jiance_x=1;}//发车给摇头舵机一个角度方便检测
   else
   { 
     a=0;
     
     b++;
     /*过弯处理部分 并不是很好*/
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
    
    if(b>6&&code<100){Motor(0,0,70,70);MotorActuator(35);DELAY_MS(500); a=4; b=3;}//卡死自动后退 但是如果轮子打滑空转 自动后退无效
    if(a>10)a=10;//防止a溢出
    if(b>20)b=20;//同上
    c=0;
   }
  
   if(angle!=200)//检测到图像的处理部分
{
  
  
  
       state_start=0;
      
       if(jiance_x==1){jiance_x=0;Y_TA(0);b=3;}//起步在摇头舵机有角度的时候检测到图像
      else
  {    
       a=0;
       b=0;
       c++;
       MotorActuator(angle);//角度
       if(c<20){ Motor(90,90,0,0);}//电机
       else if(c<40){ Motor(80,80,0,0);}
     
       if(speed_flag>=24){MotorActuator(35);if(code<700)DELAY_MS(70);a=3;c=0;}//壁障 加上了条件延时 因为速度低了可能撞灯
       if(a>20&&code<100){Motor(0,0,70,70);MotorActuator(35);DELAY_MS(500); a=0;c=0;}//防卡死 参考上一个
        if(c>300)c=300;//防溢出
        }
   }
   
 }


/*!
 *  @brief      main函数
 *  @since      v6.0
 *  @note       山外LCD LCD显示
 */
 void  main(void)
{

    ASSERT(enter_init());   //由于英文字库和汉字字库都在FWD里，因此需要判断是否加载了FWD固件

    /*//摄像头
    camera_init(imgbuff);    //摄像头初始化
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
*/

    
   /* //中断优先级设置
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    NVIC_SetPriority(PORTA_IRQn,0);
    NVIC_SetPriority(PIT0_IRQn,1);
    NVIC_SetPriority(PIT1_IRQn,1);*/
    
    //电机
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,1000);      //初始化 电机 PWM
    
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
 
    /*//超声波
    gpio_init(TRIG,GPO,0);
    gpio_init(ECHG,GPI,0);*/
    
    /*//舵机
    ftm_pwm_init(FTM0, DUOJI1_PWM, 50,75);  //初始化舵机*/
    
    //左编码器
    ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化 //所用的管脚可查 port_cfg.h 的 FTM1_QDPHA_PIN 和 FTM1_QDPHB_PIN
    pit_init_ms(PIT0, 5);                                   //初始化PIT0，定时时间为： 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    
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
   // Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
  //  Size_t size; 
   // size.H = CAMERA_H;      //真实显示
  //  size.W = CAMERA_W;
    
    //初始速度
    actual_speed = 0.0;
    int jjj = 0;
    float angelsss=0;
      while(1)
      {  
         //DELAY_MS(1500);
         //ftm_pwm_duty(FTM0, DUOJI1_PWM,45);//舵机转向

          
         /*//摄像头+舵机测试
          
          float Max_angle_left = 45; 
          float Max_angle_right = 105; 
          
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,900);
	  ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000); 
          
          
          ContinuousImage(20);//图像采集
          angle = ImageProcessing(img,&speed_flag);//图像处理 返回了角度
          
          
         // site.y = 0;
         // LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //显示图像
         // site.y = 70;
         // char buff[20];                                
         // sprintf(buff,"%.1f",(float)angle);             
         // LCD_str(site, buff, BLUE, RED);             //角度显示
          
          if (angle !=200){
          uint32 duty_angle = Position_PID(angle,0);
          
          Speedcontrol(20,duty_angle);
          
          if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}
          if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}
          
          ftm_pwm_duty(FTM0, DUOJI1_PWM,duty_angle);//舵机转向

         //site.y = 90;
         // LCD_num_C(site, duty_angle, BLUE, RED);  //显示速度
          }
         // site.y = 110;
         // float angleshow = (float)angle;
         // char show[5];
         // sprintf(show,"%.1f",angleshow);             // 变量转化为字符串*/
      
        /*//主程序，需要角度计算、超声波测距和速度控制函数
        if(Sonar()<阈值)  //超声波测距太小，要撞墙了，倒退规避
        {
          Goback();
        }
        else  //寻找光源
        {
          ContinuousImage(20);//图像采集 此函数解决了闪烁问题
          //site.y = 0;
          //LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  //显示图像
          angle=ImageProcessing(img,&speed_flag);//图像处理 返回了角度
          Carcontrol(angle); //这里不能是简单的加速哦
          DELAY_MS(200);
          if(angle==200)  //角度不存在，需要再次寻找
          {
            Find(0)
          }
          else    //找到目标，继续跟进
          {
            if(Sonar()<阈值||Light()>阈值)  //找到光源并灭灯
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
        
        /*//单电机测试
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,900);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000);
        */
  
        /*//PID单电机定速测试
        
        float setspeed = 30;
        int Max_speed = 700;
        
        uint32 duty_speed = PID_realize(setspeed,actual_speed);  //获取PID反馈值
  
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed, BLUE, RED);           //占空比
        site.x = 80;
        LCD_Str_ENCH(site, "占空比", BLUE, RED); 
        
        if (duty_speed<1000 || duty_speed>Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty_speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000); 
        site.y = 70;
        site.x = 0;
        LCD_Str_ENCH(site, "PID系统正常", BLUE, RED);        //正常
        }
        
	else {
        site.y = 90;
        site.x = 0;
        LCD_Str_ENCH(site, "要起飞了", BLUE, RED);        //跑飞
        }
         
        site.y = 50;
        site.x = 0;
        char buff[20];                                
        sprintf(buff,"%.1f",getitem());             // 变量转化为字符串
        LCD_str(site, buff, BLUE, RED);             //浮点数显示
        site.x = 80;
        LCD_Str_ENCH(site, "误差累加", BLUE, RED); */
        
        /*//PID单电机变速测试
        spe = 20 + ssss * 0.001;
        ssss+=5;
        if (ssss>30000){ssss=30000;}
        site.x = 0;
        site.y = 100;
        char buff2[20];                                
        sprintf(buff2,"%.1f",spe);             // 变量转化为字符串
        LCD_str(site, buff2, BLUE, RED);             //浮点数显示
        
	uint32 duty_speed = PID_realize(spe,actual_speed);  //获取PID反馈值
        int Max_speed = 700;
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed, BLUE, RED);           //占空比
        site.x = 80;
        LCD_Str_ENCH(site, "占空比", BLUE, RED); 
        
        if (duty_speed<1000 || duty_speed>Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty_speed);
	ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000); 
        site.y = 70;
        site.x = 0;
        LCD_Str_ENCH(site, "PID系统正常", BLUE, RED);        //正常
        }
        
	else {
        site.y = 90;
        site.x = 0;
        LCD_Str_ENCH(site, "要起飞了", BLUE, RED);        //跑飞
        }
         
        site.y = 50;
        site.x = 0;
        char buff[20];                                
        sprintf(buff,"%.1f",getitem());             // 变量转化为字符串
        LCD_str(site, buff, BLUE, RED);             //浮点数显示
        site.x = 80;
        LCD_Str_ENCH(site, "误差累加", BLUE, RED);*/
        
        /*//编码器测试
        if (actual_speed>0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, actual_speed, BLUE, RED);   //真实速度
        site.x = 80;
        LCD_Str_ENCH(site, "r/5ms", BLUE, RED); 
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "前进", BLUE, RED);  
        }
        if (actual_speed<0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, -actual_speed, BLUE, RED);   //真实速度
        site.x = 80;
        LCD_Str_ENCH(site, "r/5ms", BLUE, RED); 
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "倒退", BLUE, RED);    
        }*/
        
        
      //  MotorControl(500,1000,500,1000);
        
        //双编码器测试
        if (actual_speed_left>0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, actual_speed_left, BLUE, RED);   //真实速度
        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "前进", BLUE, RED);  
        }
        if (actual_speed_left<0){
        site.y = 10;
        site.x = 0;
        LCD_num_C(site, -actual_speed_left, BLUE, RED);   //真实速度

        site.y = 0;
        site.x = 0;
        LCD_Str_CH(site, "倒退", BLUE, RED);    
        }
            
        if (actual_speed_right>0){
        site.y = 10;
        site.x = 80;
        LCD_num_C(site, actual_speed_right, BLUE, RED);   //真实速度
        site.y = 0;
        site.x = 80;
        LCD_Str_CH(site, "前进", BLUE, RED);  
        }
        if (actual_speed_right<0){
        site.y = 10;
        site.x = 80;
        LCD_num_C(site, -actual_speed_right, BLUE, RED);   //真实速度
        site.y = 0;
        site.x = 80;
        LCD_Str_CH(site, "倒退", BLUE, RED);    
        }
          
        //PID双电机分别控制测试
        
        
        
        float setspeed = 2;
        if (jjj>300){setspeed = 0;}
        site.x = 0;
        site.y = 50;
        LCD_num_C(site, jjj, BLUE, RED);           //占空比
        int Max_speed = 10;
        
        float angletest = 75;
        Speedcontrol(setspeed,angletest);
        
        uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //获取PID反馈值
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //获取PID反馈值
        
  
        site.x = 0;
        site.y = 30;
        LCD_num_C(site, duty_speed_left, BLUE, RED);           //占空比
        site.x = 80;
        LCD_Str_ENCH(site, "占空比", BLUE, RED); 
        
        
        if (duty_speed_left<1000 && duty_speed_left>=Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,duty_speed_left);
	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, duty_speed_left, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "左电机", BLUE, RED);        //正常
        }
        
        if (duty_speed_left<=2000 && duty_speed_left>=1000){
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,1000);
	ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000-duty_speed_left); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, 2000-duty_speed_left, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "左电机", BLUE, RED);        //正常
        }
        
        
        if (duty_speed_right<1000 && duty_speed_right>=Max_speed){
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,duty_speed_right);
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1000); 
        site.y = 90;
        site.x = 0;
        LCD_num_C(site, duty_speed_right, BLUE, RED); 
        site.x = 80;
        LCD_Str_ENCH(site, "右电机", BLUE, RED);        //正常
        }
        
        if (duty_speed_right<=2000 && duty_speed_right>=1000){
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,1000);
	ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000-duty_speed_right); 
        site.y = 70;  
        site.x = 0;
        LCD_num_C(site, 2000-duty_speed_right, BLUE, RED);        
        site.x = 80;
        LCD_Str_ENCH(site, "右电机", BLUE, RED);        //正常
        }
        

     
       
       
        site.y = 110;
        site.x = 0; 
        char buff3[20]; 
        sprintf(buff3,"%.1f",getitem());             // 变量转化为字符串
        LCD_str(site, buff3, BLUE, RED);             //浮点数显示
        site.x = 80;
        LCD_Str_ENCH(site, "误差累加", BLUE, RED); 
        
    
        jjj++;
       
        /*//测试电机和马达协同合作
          int angles[] = {35,34,31,30,29,27,25,20,15,10,5,0,-5,-6,-7,-8,-10,-6,-3,0,3,0};//测试
          for (int i =0;i<sizeof(angles)-1;i++){//测试
           // site.y = 90;
           // LCD_num_C(site, i, BLUE, RED);  //显示角度
           // site.y = 110;
            //LCD_num_C(site, angles[i], BLUE, RED);  //显示角度
	    Carcontrol(angles[i]); //这里不能是简单的加速哦
            DELAY_MS(500);//测试
          }
*/
        
        /*//PID舵机测试
        
        float Max_angle_left = 45 ;
        float Max_angle_right = 105;
        
        //ftm_pwm_duty(FTM0, DUOJI1_PWM,900);//舵机居中
       
        int32 setangle = 35;
        uint32 pwm = Position_PID(angelsss,setangle);
        angelsss++;*/
        
        
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
void PIT0_IRQHandler(void)
{
    actual_speed_left = -ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)(注意，速度和实际速度反向了，所以取了反)(换小车后，左边速度方向还是反的)
    ftm_quad_clean(FTM1);
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

/*!
 *  @brief      PIT1中断服务函数
 *  @since      v6.0
 */
void PIT1_IRQHandler(void)
{
    actual_speed_right = ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)(注意，速度和实际速度反向了，所以取了反)（换小车后，右边速度方向又变回来了)
    ftm_quad_clean(FTM2);
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

//寻灯函数
void Find(int type,int angle) //寻找下一个目标的函数
 {  
   state = 1; //正在寻灯
   if (type==0)  //直接寻找灯（开机寻找灯）（被阻挡后寻灯）
   {
       ftm_pwm_duty(FTM0, DUOJI1_PWM, 45);//左打死
       if(angle==200)
	   {
		Speedcontrol(4,45);
		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //获取PID反馈值
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //获取PID反馈值
        
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
   
   else if (type==1) //倒退寻找灯（灭灯后寻灯）
   {
   for(int i=0;i<3;i++)//倒退
   {  
		MotorControl(1000,700,1000,700);
		DELAY_MS(500);//测试
   }
    ftm_pwm_duty(FTM0, DUOJI1_PWM, 45);//左打死
    

   if(angle==200)
   {
		Speedcontrol(4,45);
		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //获取PID反馈值
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //获取PID反馈值
        
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

int Sonar()  //超声波测距
{
        uint32 timevar;
        uint32 flag;
        flag = 0;
        gpio_set(TRIG,1);               //产生触发脉冲
        pit_delay_us(PIT1,15);
        gpio_set(TRIG,0);
        
        while(gpio_get(ECHG) == 0);             //等待电平变高，低电平一直等待
        pit_time_start  (PIT0);                 //开始计时
        while(gpio_get(ECHG) == 1)              //等待电平变低，高电平一直等待
        {
            flag++;
            if(flag >FLAGWAIT)
            {
                break;
            }
        };             
        
        timevar = pit_time_get_us    (PIT0);    //停止计时，获取计时时间
        if(flag <FLAGWAIT )
        {
            timevar = timevar * 340 /2/1000;
                
            if(timevar > 5)
            {
                return(timevar);
            }
        }
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

    if( angle == 200 ){a++;}

    if(a>=4)//4次循环都没有图像 进入无图像处理部分
	{
		if (state_start == 1) //小车启动时，开始寻灯
		{ 
			Find(0,angle); 
		}

	    if(state == 3)//灭完灯后倒退进行寻灯
		{
			state = 4;
			Find(1,angle);
		}
		else  //一般状态，灯被灭，或是被阻挡后寻灯
		{   
			Find(0,angle); 
		}

		if(a>10)a=10;
    }
  
    if( angle !=200 )
	{
		state_start = 0;

		uint32 duty_angle = Position_PID(-angle,0);//舵机PID返回PWM
        if (duty_angle < Max_angle_left){duty_angle = Max_angle_left;}  //设置占空比范围
        if (duty_angle > Max_angle_right){duty_angle = Max_angle_right;}

		if(speed_flag > 24) //开始准备刹车  TODO：测这个阈值
		{
			Speedcontrol(0,duty_angle);//由PWM算出舵机转向角度，再算出两个电机的差速		
			state = 3;
		}
		else 
		{
			Speedcontrol(12,duty_angle);//由PWM算出舵机转向角度，再算出两个电机的差速
			state = 2;
		}

		ftm_pwm_duty(FTM0, FTM_CH0, duty_angle);//转动舵机到目标

		uint32 duty_speed_left = PID_realize_left(SpeedA,actual_speed_left);  //获取PID反馈值
        uint32 duty_speed_right = PID_realize_right(SpeedB,actual_speed_right);  //获取PID反馈值
        
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
  
}
