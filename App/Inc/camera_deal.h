#include  "common.h"
#include  "include.h"
#ifndef _CAMERA_DEAL_H_
#define _CAMERA_DEAL_H_

extern uint8 center_point;         //���ĵ�

extern void  img_extracts(uint8 *dst, uint8 *src, uint32 srclen); // ѹ��ͼ��  
extern void  Mid_Filter(uint8 *array,uint8 length);          //��λֵ�˲�����
extern void  capture_target(uint8 *arry,uint8 length);       //����Ŀ��
extern void  sendimg(uint8 *imgaddr, uint32 imgsize);        //���ͺ���

#endif