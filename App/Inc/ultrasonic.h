#ifndef _LANDZO_ULTRASONIC_H
#define _LANDZO_ULTRASONIC_H

//extern uint32 Ultrasonic_us=0;           //³¬Éù²¨us¼ÆÊı
//extern float Ultrasonic_Distance;  //³¬Éù²¨²âÊÔ¾àÀë
//extern uint8 Ultrasonic_Flag=0;          //ÊÇ·ñ³¬³ö²âÊÔ¾àÀë
//extern uint16 Ultrasonic_mm;             //¾àÀëmm


void   Ultrasonic_Init();          //³¬Éù²¨³õÊ¼»¯
void   Ultrasonic_Delay(uint32 n);    //³¬Éù²¨ÑÓÊ±us¼¶º¯Êı
void   Ultrasonic_Trig();          //³¬Éù²¨·¢ËÍº¯Êı
float  Ultrasonic_Compute(uint32 us);       //³¬Éù²¨¼ÆËã¾àÀëº¯Êı
#endif