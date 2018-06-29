/******************** 郑州轻工业学院创新实验室************************
 *
 * 文件名称: Speed_control
 * 功能    : 速度控制
 *
 * 平台    : MK60FX512
 * 描述    : None
 *
 * 作 者   : 2015/2/4, by My Team    P_speed =1.4  I_speed=3.3
 **********************We are the best team !  **********************/
#include "include.h"
#include "Speed_control.h"
#include "common.h"




int16 Speed_test;  //由编码器采集回来的速度值
int16 Speed_Value; //与PWM占空比相匹配的值

int16 Speed_set,Speed_error=0,Speed_last_error=0,Speed_second_error=0;
int16 Speed_Control=0;    //定义速度的最终输出值
int16 Speed_control=0;
int32 Speed_Out=0;

float S_P=2.5,S_I=0.04,S_D=0;//定义控制速度的PID参数

/************************************************************
 * 函数名称: Speed_control()
 * 输入参数: None
 * 输出参数: None
 *
 * 功 能: 速度控制
 *
 * 作 者: 2015/2/4, by Han Zhenshuai
 ************************************************************/

void speed_control(void)
{
  float I;
  /************************电机PID***************************************/
        static int16 Last_speed_control;
        Speed_error=Speed_set-Speed_Value;
        if((Speed_error)>500)
          I=0;
        else
          I= S_I;

        Speed_Control=(int16)(S_P*(Speed_error-Speed_last_error)\
                      +I*Speed_error\
                      +S_D*(Speed_error-2 * Speed_last_error+Speed_second_error));
        Speed_second_error=Speed_last_error;
        Speed_last_error=Speed_error;


        Speed_control = Last_speed_control + Speed_Control;
        Last_speed_control = Speed_control;
       // Last_speed_control = Speed_control;
        //if(Speed_Control>3000)
          //Speed_Control=3000;
         //if(Speed_Control<-3000)
          //Speed_Control=-3000;



}





void speed_Out(int16 code,int16 Speed)
{
     Speed_test= code;  //获取FTM 正交解码 的脉冲数(负数表示反方向)
     Speed_Value=Speed_test;

    Speed_set =Speed;    //速度设定
    speed_control();
   //速度控制
    
    Speed_Out=Speed_control/10;
    Motor(Speed_Out,Speed_Out,0,0);
 /*   if(Speed_Out_left>10000)
      Speed_Out_left=5000;
    if(Speed_Out_left<1000)
       Speed_Out_left=1000;

    if(Speed_Out_right>10000)
      Speed_Out_right=5000;
    if(Speed_Out_right<1000)
       Speed_Out_right=1000;            //限幅处理
  */

}



