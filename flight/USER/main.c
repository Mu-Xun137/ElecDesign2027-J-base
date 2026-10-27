#include "ALL_DEFINE.h"
#include "j_platform.h"
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    cycleCounterInit();
    NVIC_SetPriority(SysTick_IRQn,0); /* milliseconds may preempt the sensor/motor ISR */
    ALL_Init();
    for(;;) RC_Analy(); /* no blocking USB polling or foreground I2C transactions */
}
