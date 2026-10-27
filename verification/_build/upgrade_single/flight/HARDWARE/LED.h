#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"
/* Actual LEDs use TIM3 CH2/CH1/CH4/CH3 on PA7/PA6/PB1/PB0.
 * Position/color/polarity are centralized in j_config.h. */
typedef struct {
    uint16_t FlashTime;
    enum { AlwaysOn,AlwaysOff,AllFlashLight,AlternateFlash,WARNING,DANGEROURS,GET_OFFSET } status;
} sLED;
extern sLED LED;
void LEDInit(void);
void LEDtest(void);
void PilotLED(void);
#endif
