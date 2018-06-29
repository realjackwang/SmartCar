#include  "common.h"
#include  "include.h"

#ifndef __PID_H__
#define __PID_H__

//extern struct _pid
extern void PID_init_left();
extern void PID_init_right();
extern uint32 PID_realize_left(float speed,int16 actual_speed);
extern uint32 PID_realize_right(float speed,int16 actual_speed);
extern uint32 Position_PID(float Encoder,int32 Target);
extern void PID_init2();
extern float getitem();

#endif  //__PID_H__