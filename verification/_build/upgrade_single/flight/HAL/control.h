#ifndef __CONTROL_H
#define __CONTROL_H
#include "pid.h"
#include "sys.h"
extern float sins_high,sins_vel;
extern uint16_t thr_hold,low_thr_cnt;
void FlightPidControl(float dt);
void MotorControl(void);
#endif
