#include  "common.h"
#include  "include.h"
#ifndef _CAMERA_DEAL_H_
#define _CAMERA_DEAL_H_

extern uint8 center_point;         //中心点

extern void  img_extracts(uint8 *dst, uint8 *src, uint32 srclen); // 压缩图像  
extern void  Mid_Filter(uint8 *array,uint8 length);          //中位值滤波函数
extern void  capture_target(uint8 *arry,uint8 length);       //中心目标
extern void  sendimg(uint8 *imgaddr, uint32 imgsize);        //发送函数

#endif