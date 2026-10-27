#ifndef J_PLATFORM_H
#define J_PLATFORM_H
#include "j_core.h"
#include "j_sensors.h"
extern JCore j_flight;
extern JSensorFilter j_sensor;
extern volatile uint32_t j_last_motor_ms,j_max_isr_us,j_deadline_misses;
extern volatile uint8_t j_deadline_fault;
void JPlatform_Init(void);
void JPlatform_ControlTick(float dt,uint8_t module_tick);
void JPlatform_ApplyLED(uint8_t mask);
void JPlatform_ZeroMotors(void);
void JPlatform_OnControl(const uint8_t *packet,uint32_t now);
void JPlatform_RadioPoll(void);
#endif
