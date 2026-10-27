#include "ALL_DEFINE.h"
#include "j_platform.h"
void SysTick_Handler(void)
{
    ++SysTick_count;
    if(j_flight.armed && (uint32_t)(SysTick_count-j_last_motor_ms)>J_CONTROL_DEADLINE_MS) {
        j_deadline_fault=1; JPlatform_ZeroMotors();
    }
}
void cycleCounterInit(void) { SysTick_Config(SystemCoreClock/1000U); }
uint32_t GetSysTime_us(void)
{
    uint32_t ms,ticks;
    do { ms=SysTick_count; ticks=SysTick->VAL; } while(ms!=SysTick_count);
    return ms*1000U+(SysTick->LOAD-ticks)/(SystemCoreClock/1000000U);
}
void delay_ms(uint16_t ms)
{ uint32_t start=SysTick_count; while((uint32_t)(SysTick_count-start)<ms) {} }
void delay_us(unsigned int us)
{ uint32_t start=GetSysTime_us(); while((uint32_t)(GetSysTime_us()-start)<us) {} }
