#include "j_bh1750.h"
#include "j_bh_i2c.h"
static uint8_t phase,waiting_retry,have_cycle;
static uint32_t stamp,cycle_ms;
void JBH_Init(void)
{
    JBHI2C_Init();
    phase=waiting_retry=have_cycle=0; stamp=cycle_ms=0;
}
void JBH_Tick(JSensors *s,uint32_t now)
{
    uint8_t b[2],bad=0;
    uint16_t raw;
    if(waiting_retry) {
        if((uint32_t)(now-stamp)<J_BH_RETRY_MS) return;
        waiting_retry=0;
    }
    if(phase==0U) { bad=JBHI2C_SendCommand(0x01U)!=0; phase=1; }
    else if(phase==1U) { bad=JBHI2C_SendCommand(0x07U)!=0; phase=2; }
    else if(phase==2U) {
        if(have_cycle && (uint32_t)(now-cycle_ms)<J_BH_PERIOD_MS) return;
        bad=JBHI2C_SendCommand(0x20U)!=0; /* ONE-TIME high resolution, MTreg default 69 */
        cycle_ms=stamp=now; have_cycle=1;phase=3;
    } else if((uint32_t)(now-stamp)>=J_BH_CONVERSION_MS) {
        bad=JBHI2C_Receive(b,2U)!=0;
        if(!bad) {
            raw=(uint16_t)(((uint16_t)b[0]<<8)|b[1]);
            if(raw==65535U) bad=1; /* saturation is insufficient evidence */
            else { s->lux=(float)raw/1.2f; s->light_ok=1; s->light_ms=now; ++s->light_id; }
        }
        phase=2;
    }
    if(bad) { s->light_ok=0; phase=0; waiting_retry=1; stamp=now; cycle_ms=0;have_cycle=0; }
}
