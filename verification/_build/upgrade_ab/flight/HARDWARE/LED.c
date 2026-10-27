#include "ALL_DEFINE.h"
#include "j_platform.h"
sLED LED;
void LEDInit(void) { JPlatform_ApplyLED(0); }
void LEDtest(void) { if(!j_flight.armed) JPlatform_ApplyLED(JCore_LEDMask(&j_flight,SysTick_count)); }
void PilotLED(void) { JPlatform_ApplyLED(JCore_LEDMask(&j_flight,SysTick_count)); }
