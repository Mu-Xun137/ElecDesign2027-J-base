#include "ALL_DEFINE.h"
#include "j_platform.h"
void TIM1_UP_IRQHandler(void)
{
    static uint8_t divide;
    static uint32_t previous_us;
    uint32_t start,elapsed;
    float dt;
    if(TIM_GetITStatus(TIM1,TIM_IT_Update)==RESET) return;
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
    start=GetSysTime_us();
    dt=previous_us?(float)(uint32_t)(start-previous_us)*0.000001f:0.003f;
    previous_us=start;
    divide^=1U;
    JPlatform_ControlTick(dt,divide==0U);
    elapsed=(uint32_t)(GetSysTime_us()-start);
    if(elapsed>j_max_isr_us) j_max_isr_us=elapsed;
    if(elapsed>=3000U) ++j_deadline_misses;
    /* SysTick owns milliseconds; there is NO second clock increment here. */
}
