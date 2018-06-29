#ifndef __CAR_H__
#define __CAR_H__

extern int MotorActuator(int angle);
extern int Y_TA(int angle);
extern void Y_T(int n);
extern void Motor(uint32 ForwardSpeed1,uint32 ForwardSpeed2,uint32 ReverseSpeed1,uint32 ReverseSpeed2);
extern void AntijammingContinuousImage(uint32 threshold);
int ImageProcessing(uint8 *string,uint32 *m);
extern void ContinuousImage(uint32 threshold);
extern void SerialDisplay(uint8 *string);
extern void Avoid(int16 code);
#endif   