#include "j_protocol.h"
#include <string.h>
static void put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8)|p[1]); }
static void put32(uint8_t *p,uint32_t v) { put16(p,(uint16_t)(v>>16)); put16(p+2,(uint16_t)v); }
static uint32_t get32(const uint8_t *p) { return ((uint32_t)get16(p)<<16)|get16(p+2); }
static void checksum(uint8_t *p)
{ uint8_t k,sum=0; for(k=0;k<31U;++k) sum=(uint8_t)(sum+p[k]); p[31]=sum; }
uint8_t JPacket_Check(const uint8_t *p,uint8_t type)
{
    uint8_t k,sum=0;
    if(p[0]!=0xAAU || p[1]!=0xAFU || p[2]!=type || p[3]!=27U) return 0;
    for(k=0;k<31U;++k) sum=(uint8_t)(sum+p[k]);
    return sum==p[31];
}
void JControl_Encode(uint8_t *p,const JControlPacket *c)
{
    uint8_t k;
    memset(p,0,J_PKT_SIZE);
    p[0]=0xAA; p[1]=0xAF; p[2]=0x03; p[3]=27;
    for(k=0;k<8U;++k) put16(p+4U+2U*k,c->channel[k]); /* exactly eight channels */
    p[20]=J_VERSION; p[21]=(uint8_t)((c->pressed&15U)|(c->reply?128U:0U));
    put32(p+22,c->session); put16(p+26,c->seq);
    p[28]=(uint8_t)((c->key_count[0]&15U)|((c->key_count[1]&15U)<<4));
    p[29]=(uint8_t)((c->key_count[2]&15U)|((c->key_count[3]&15U)<<4));
    p[30]=c->ready_keys&15U;
    checksum(p);
}
uint8_t JControl_Decode(const uint8_t *p,JControlPacket *c)
{
    uint8_t k;
    if(!JPacket_Check(p,3U) || p[20]!=J_VERSION || (p[21]&0x70U) || (p[30]&0xF0U)) return 0;
    for(k=0;k<8U;++k) {
        c->channel[k]=get16(p+4U+2U*k);
        if(c->channel[k]<1000U || c->channel[k]>2000U) return 0;
    }
    c->session=get32(p+22); c->seq=get16(p+26);
    c->pressed=p[21]&15U; c->reply=(p[21]>>7)&1U;
    c->ready_keys=p[30]&15U;
    c->key_count[0]=p[28]&15U; c->key_count[1]=p[28]>>4;
    c->key_count[2]=p[29]&15U; c->key_count[3]=p[29]>>4;
    return c->session!=0U;
}
uint8_t JReceiver_Accept(JReceiver *r,const uint8_t *p,uint32_t now,
                        JControlPacket *c,uint8_t *events,uint8_t *gap)
{
    uint8_t k,sync;
    uint16_t delta;
    *events=*gap=0;
    if(!JControl_Decode(p,c)) return 0;
    if(r->ready && c->session==r->previous_session && c->session!=r->session) return 0;
    *gap=r->ready && (uint32_t)(now-r->last_ms)>=J_RC_LOST_MS;
    sync=!r->ready || *gap || c->session!=r->session;
    if(!sync) {
        delta=(uint16_t)(c->seq-r->seq);
        if(delta==0U || delta>=32768U) return 0; /* duplicates and out-of-order never keep link alive */
        for(k=0;k<4U;++k) {
            uint8_t n=(uint8_t)((c->key_count[k]-r->count[k])&15U);
            if(n>0U && n<8U) *events|=(uint8_t)(1U<<k);
        }
    } else if(r->ready && c->session!=r->session) r->previous_session=r->session;
    r->ready=1; r->session=c->session; r->seq=c->seq; r->last_ms=now;
    for(k=0;k<4U;++k) r->count[k]=c->key_count[k];
    if(c->pressed&c->ready_keys&2U) *events|=2U; /* real held C2, never an AUX-default edge */
    return 1;
}
void JStatus_Encode(uint8_t *p,const JStatusPacket *s)
{
    memset(p,0,J_PKT_SIZE);
    p[0]=0xAA; p[1]=0xAF; p[2]=0x83; p[3]=27;
    put32(p+4,s->control_session); put16(p+8,s->echo_seq); put32(p+10,s->flight_session);
    p[14]=s->mode; p[15]=s->phase; p[16]=s->flags; p[17]=s->align;
    put16(p+18,s->light_event); put16(p+20,s->takeover_event);
    put16(p+22,s->alert_elapsed_ms); put16(p+24,s->height_mm);
    p[26]=s->fault; p[27]=s->sensors; put16(p+28,s->telemetry_seq); p[30]=J_VERSION;
    checksum(p);
}
uint8_t JStatus_Decode(const uint8_t *p,JStatusPacket *s)
{
    if(!JPacket_Check(p,0x83U) || p[30]!=J_VERSION) return 0;
    s->control_session=get32(p+4); s->echo_seq=get16(p+8); s->flight_session=get32(p+10);
    s->mode=p[14]; s->phase=p[15]; s->flags=p[16]; s->align=p[17];
    s->light_event=get16(p+18); s->takeover_event=get16(p+20);
    s->alert_elapsed_ms=get16(p+22); s->height_mm=get16(p+24);
    s->fault=p[26]; s->sensors=p[27]; s->telemetry_seq=get16(p+28);
    return s->control_session!=0U && s->flight_session!=0U && s->mode<=7U &&
           s->phase<=8U && s->align<=4U && s->fault<=13U;
}
