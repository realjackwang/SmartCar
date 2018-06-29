/*********************************** 郑州轻工业学院创新实验室*************************************************************
 *
 * 文件名称: Speed_control
 * 功能    : 速度控制
 *
 * 平台    : MK60FX512
 * 描述    : None
 *
 * 作 者   : 2015/2/4, by My Team
 ***********************************We are the best team !  **************************************************/
#ifndef  __Speed_control_H__
#define  __Speed_control_H__

#include "include.h"
#include "stdlib.h"



   
extern int32 Speed_Out;
extern float S_P,S_I,S_D;     //定义控制速度的PID参数
extern int16 Speed_Value;
extern int16 Speed_set;






void speed_control(void);

void speed_Out(int16 code,int16 Speed);


#define INT_COUNT 500

#endif