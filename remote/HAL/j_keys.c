#include "j_keys.h"
#include "j_config.h"
uint8_t JKeys_Step(JKeys *s,uint8_t raw,uint32_t now)
{
    uint8_t k,events=0;
    raw&=15U;
    if(!s->initialized) {
        s->initialized=1; s->raw=s->stable=raw; s->released=0;
        for(k=0;k<4U;++k) s->changed_ms[k]=now;
        return 0;
    }
    for(k=0;k<4U;++k) {
        uint8_t bit=(uint8_t)(1U<<k);
        if((raw&bit)!=(s->raw&bit)) { s->raw^=bit; s->changed_ms[k]=now; }
        if((uint32_t)(now-s->changed_ms[k])<J_KEY_DEBOUNCE_MS) continue;
        s->validated|=bit; /* C2 may use a real stable held level, even at boot. */
        if(!(raw&bit)) s->released|=bit;
        if((raw&bit)!=(s->stable&bit)) {
            s->stable^=bit;
            if((raw&bit) && (s->released&bit)) {
                s->count[k]=(uint8_t)((s->count[k]+1U)&15U);
                events|=bit;
            }
        }
    }
    return events;
}
