#include "j_protocol.h"
#include "j_keys.h"
#include "j_feedback.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,cases;
static const char *current;
#define CHECK(x) do { ++checks; if(!(x)) { printf("FAIL %s:%d: %s\n",current,__LINE__,#x); exit(2); } } while(0)
#define RUN(f) do { current=#f; f(); ++cases; printf("PASS %s\n",current); } while(0)
static void sum(uint8_t *p) { unsigned k; p[31]=0; for(k=0;k<31;++k) p[31]=(uint8_t)(p[31]+p[k]); }
static JControlPacket control(void)
{
    JControlPacket c; unsigned k; memset(&c,0,sizeof(c));
    for(k=0;k<8;++k) c.channel[k]=(uint16_t)(1000+100*k);
    c.session=0x12345678U; c.seq=100; c.reply=1; return c;
}
static uint8_t accept(JReceiver *r,JControlPacket *c,uint32_t now,uint8_t *e,uint8_t *gap)
{ uint8_t p[32]; JControlPacket decoded; JControl_Encode(p,c); return JReceiver_Accept(r,p,now,&decoded,e,gap); }
static JStatusPacket status(void)
{
    JStatusPacket s; memset(&s,0,sizeof(s));
    s.control_session=0x12345678U; s.flight_session=0x87654321U;
    s.echo_seq=100; s.telemetry_seq=10; s.mode=3; s.phase=2; s.flags=J_STATUS_ARMED;
    s.height_mm=800; s.sensors=15; return s;
}
static uint8_t feedback(JFeedback *f,JStatusPacket *s,uint32_t now,uint16_t latest)
{ uint8_t p[32]; JStatus_Encode(p,s); return JFeedback_Accept(f,p,now,0x12345678U,latest,now); }
static void eight_channel_bounds_and_mapping(void)
{
    struct { uint32_t a; uint8_t p[32]; uint32_t b; } guarded;
    JControlPacket c=control(),d; memset(&guarded,0xCD,sizeof(guarded));
    JControl_Encode(guarded.p,&c); CHECK(guarded.a==0xCDCDCDCDU && guarded.b==0xCDCDCDCDU);
    CHECK(JControl_Decode(guarded.p,&d)); CHECK(memcmp(c.channel,d.channel,sizeof(c.channel))==0);
    CHECK(guarded.p[16]==(uint8_t)(1600>>8) && guarded.p[17]==(uint8_t)1600);
    CHECK(guarded.p[18]==(uint8_t)(1700>>8) && guarded.p[19]==(uint8_t)1700 && guarded.p[30]==0);
    CHECK(d.session==c.session && d.seq==100 && d.reply);
}
static void invalid_packets_and_all_single_bit_errors(void)
{
    uint8_t p[32],bad[32]; unsigned a,b; JControlPacket c=control(),d;
    JControl_Encode(p,&c);
    for(a=0;a<32;++a) for(b=0;b<8;++b) { memcpy(bad,p,32); bad[a]^=(uint8_t)(1U<<b); CHECK(!JControl_Decode(bad,&d)); }
    memcpy(bad,p,32);bad[2]=0x83;sum(bad);CHECK(!JControl_Decode(bad,&d));
    memcpy(bad,p,32);bad[20]=0;sum(bad);CHECK(!JControl_Decode(bad,&d));
    c.channel[0]=2001;JControl_Encode(bad,&c);CHECK(!JControl_Decode(bad,&d));
    c.channel[0]=999;JControl_Encode(bad,&c);CHECK(!JControl_Decode(bad,&d));
}
static void first_duplicate_reorder_and_bad_no_liveness(void)
{
    JReceiver r={0}; JControlPacket c=control(),d; uint8_t e,g,p[32];
    c.key_count[2]=7;CHECK(accept(&r,&c,1000,&e,&g)&&!e&&!g);
    CHECK(!accept(&r,&c,1200,&e,&g)&&r.last_ms==1000);
    --c.seq; CHECK(!accept(&r,&c,1300,&e,&g)&&r.last_ms==1000);
    JControl_Encode(p,&c);p[31]^=1;
    CHECK(!JReceiver_Accept(&r,p,1450,&d,&e,&g)&&r.last_ms==1000);
    c.seq=101;c.key_count[2]=8; CHECK(accept(&r,&c,1490,&e,&g)&&e==4&&!g);
}
static void simultaneous_keys_loss_and_modular_wrap(void)
{
    JReceiver r={0};JControlPacket c=control();uint8_t e,g;unsigned k;
    c.seq=65535;for(k=0;k<4;++k)c.key_count[k]=15;
    CHECK(accept(&r,&c,1000,&e,&g)&&!e);
    c.seq=0;for(k=0;k<4;++k)c.key_count[k]=0;
    CHECK(accept(&r,&c,1020,&e,&g)&&e==15);
    c.seq=4;c.key_count[1]=3;c.key_count[2]=2;CHECK(accept(&r,&c,1100,&e,&g)&&e==6);
    c.seq=5;c.key_count[0]=8;CHECK(accept(&r,&c,1120,&e,&g)&&e==0); /* ambiguous 8/16 resync, no fake key */
}
static void reboot_and_reconnect_resynchronization(void)
{
    JReceiver r={0};JControlPacket c=control(),old;uint8_t e,g;
    c.key_count[2]=1;CHECK(accept(&r,&c,1000,&e,&g));old=c;
    c.session=99;c.seq=1;c.key_count[2]=0;CHECK(accept(&r,&c,1020,&e,&g)&&!e);
    old.seq=101;old.key_count[1]=1;CHECK(!accept(&r,&old,1030,&e,&g));
    c.seq=2;c.key_count[2]=1;CHECK(accept(&r,&c,1040,&e,&g)&&e==4);
    c.seq=50;c.key_count[1]=2;CHECK(accept(&r,&c,1541,&e,&g)&&!e&&g);
    c.seq=51;c.key_count[1]=3;CHECK(accept(&r,&c,1561,&e,&g)&&e==2&&!g);
}
static void physical_debounce_boot_hold_and_all_keys(void)
{
    JKeys k={0};uint8_t e;
    CHECK(!JKeys_Step(&k,4,0));CHECK(!JKeys_Step(&k,4,100));CHECK(k.count[2]==0&&k.validated==15);
    JKeys_Step(&k,0,110);JKeys_Step(&k,0,130);
    CHECK(!JKeys_Step(&k,15,140));e=JKeys_Step(&k,15,160);CHECK(e==15);
    CHECK(!JKeys_Step(&k,15,1160));CHECK(k.count[0]==1&&k.count[1]==1&&k.count[2]==1&&k.count[3]==1);
    JKeys_Step(&k,0,1170);JKeys_Step(&k,0,1190); k.count[1]=15;
    JKeys_Step(&k,2,1200);CHECK(JKeys_Step(&k,2,1220)==2 && k.count[1]==0);
}
static void contact_bounce_no_false_press(void)
{
    JKeys k={0};JKeys_Step(&k,0,0);JKeys_Step(&k,0,30);
    CHECK(!JKeys_Step(&k,8,40));CHECK(!JKeys_Step(&k,0,45));CHECK(!JKeys_Step(&k,8,50));
    CHECK(!JKeys_Step(&k,8,69));CHECK(JKeys_Step(&k,8,70)==8);
}
static void telemetry_roundtrip_and_fresh_echo(void)
{
    JFeedback f={0};JStatusPacket s=status(),d;uint8_t p[32];
    s.light_event=123;s.takeover_event=456;s.align=2;s.fault=5;
    JStatus_Encode(p,&s);CHECK(JStatus_Decode(p,&d));CHECK(d.light_event==123&&d.takeover_event==456&&d.height_mm==800);
    CHECK(feedback(&f,&s,1000,100));CHECK(!f.success_on);
    CHECK(!feedback(&f,&s,1010,100));
    ++s.telemetry_seq;s.echo_seq=101;CHECK(!feedback(&f,&s,1020,100));
    s.echo_seq=90;CHECK(!feedback(&f,&s,1020,100));
    s.echo_seq=100;s.control_session=1;CHECK(!feedback(&f,&s,1020,100));
}
static void takeover_once_and_old_reply_never_success(void)
{
    JFeedback f={0};JStatusPacket s=status();CHECK(feedback(&f,&s,1000,100));
    ++s.telemetry_seq;s.echo_seq=101;s.mode=1;s.takeover_event=1;
    CHECK(feedback(&f,&s,1020,101)&&f.success_on);CHECK(JFeedback_Buzzer(&f,1020));
    CHECK(!feedback(&f,&s,1100,101));CHECK(f.success_ms==1020);
    CHECK(!JFeedback_Buzzer(&f,1120));CHECK(JFeedback_Buzzer(&f,1210));
    ++s.telemetry_seq;s.takeover_event=2;s.echo_seq=102;
    CHECK(!feedback(&f,&s,1300,200));CHECK(f.takeover_seen==1);
}
static void reboot_snapshot_and_out_of_order_feedback(void)
{
    JFeedback f={0};JStatusPacket s=status(),old;
    s.takeover_event=9;s.mode=1;CHECK(feedback(&f,&s,1000,100));CHECK(!f.success_on);old=s;
    ++s.telemetry_seq;CHECK(feedback(&f,&s,1020,100));
    CHECK(!feedback(&f,&old,1030,100));
    s.flight_session=55;s.takeover_event=0;s.telemetry_seq=0;CHECK(feedback(&f,&s,1040,100)&&!f.success_on);
    ++old.telemetry_seq;CHECK(!feedback(&f,&old,1050,100));
    s.takeover_event=7;s.telemetry_seq=1;CHECK(feedback(&f,&s,1700,100)&&!f.success_on);
}
static void continuous_three_second_alarm_and_click(void)
{
    JFeedback f={0};JStatusPacket s=status();unsigned t;
    s.flags|=J_STATUS_ALERT;s.light_event=1;CHECK(feedback(&f,&s,0,100));
    for(t=0;t<3000;++t) {
        if(t%20==0) { ++s.telemetry_seq;s.alert_elapsed_ms=(uint16_t)t;feedback(&f,&s,t,100); }
        if(t%137==0)JFeedback_Click(&f,t);
        CHECK(JFeedback_Buzzer(&f,t));
    }
    JFeedback_Buzzer(&f,3000);CHECK(!f.alert_on && f.alert_start_ms==0);
}
static void direction_patterns_and_height_same_tone(void)
{
    JFeedback f={0};JStatusPacket s=status();uint8_t low[1000];unsigned t;
    s.flags|=J_STATUS_ALIGN;s.align=1;feedback(&f,&s,0,100);
    CHECK(JFeedback_Buzzer(&f,0));CHECK(!JFeedback_Buzzer(&f,120));CHECK(JFeedback_Buzzer(&f,300));
    memset(&f,0,sizeof(f));s.align=2;feedback(&f,&s,0,100);
    CHECK(JFeedback_Buzzer(&f,0));CHECK(JFeedback_Buzzer(&f,120));CHECK(!JFeedback_Buzzer(&f,300));
    memset(&f,0,sizeof(f));s.flags=J_STATUS_ARMED;s.sensors=J_SENSOR_LOW;feedback(&f,&s,0,100);
    for(t=0;t<1000;++t){ f.last_ms=t;low[t]=JFeedback_Buzzer(&f,t); }
    memset(&f,0,sizeof(f));s.sensors=J_SENSOR_HIGH;feedback(&f,&s,0,100);
    for(t=0;t<1000;++t){ f.last_ms=t;CHECK(low[t]==JFeedback_Buzzer(&f,t));CHECK(low[t]==(t<100)); }
}
static void whole_fault_and_direction_groups(void)
{
    JFeedback f={0};JStatusPacket s=status();unsigned t;
    s.flags=J_STATUS_ARMED|J_STATUS_FAULT|J_STATUS_ALIGN;s.align=1;feedback(&f,&s,0,100);
    for(t=0;t<3000;++t) {
        uint8_t expected;
        f.last_ms=t;
        if(t<1000||t>=2000) { unsigned x=t%1000;expected=x<480 && x%120<60; }
        else { unsigned x=t-1000;expected=x<90||(x>=190&&x<440); }
        CHECK(JFeedback_Buzzer(&f,t)==expected);
    }
}
static void stale_feedback_and_emergency_priority(void)
{
    JFeedback f={0};JStatusPacket s=status();s.flags|=J_STATUS_ALERT;s.light_event=1;feedback(&f,&s,0,100);
    CHECK(JFeedback_Buzzer(&f,0));
    s.flags|=J_STATUS_ESTOP|J_STATUS_FAULT;++s.telemetry_seq;s.alert_elapsed_ms=50;feedback(&f,&s,50,100);
    CHECK(JFeedback_Buzzer(&f,50));CHECK(!JFeedback_Buzzer(&f,110));
    memset(&f,0,sizeof(f));s=status();s.flags|=J_STATUS_ALIGN;s.align=1;feedback(&f,&s,0,100);
    JFeedback_Buzzer(&f,0);CHECK(JFeedback_Buzzer(&f,501));CHECK(!JFeedback_Buzzer(&f,561));
}
static void feedback_timer_wrap(void)
{
    JFeedback f={0};JStatusPacket s=status();unsigned t;uint32_t start=0xFFFFFF00U;
    s.flags|=J_STATUS_ALERT;s.light_event=1;feedback(&f,&s,start,100);
    for(t=0;t<3000;++t) { f.last_ms=start+t;CHECK(JFeedback_Buzzer(&f,start+t)); }
    f.last_ms=start+3000;CHECK(!JFeedback_Buzzer(&f,start+3000));
}
static void validated_physical_c2_first_frame_is_not_an_aux_edge(void)
{
    JReceiver r={0};JControlPacket c=control();uint8_t e,g;
    c.pressed=2;c.ready_keys=0;CHECK(accept(&r,&c,1000,&e,&g)&&e==0); /* physical level not debounced yet */
    c.session=77;c.seq=1;c.ready_keys=2;CHECK(accept(&r,&c,1020,&e,&g)&&e==2);
    ++c.seq;CHECK(accept(&r,&c,1040,&e,&g)&&e==2); /* keep emergency held; no rearming */
    ++c.seq;c.pressed=0;CHECK(accept(&r,&c,1060,&e,&g)&&e==0);
}
static void new_event_full_three_seconds_and_estop_discards_it(void)
{
    JFeedback f={0};JStatusPacket s=status();uint32_t t;feedback(&f,&s,0,100);
    ++s.telemetry_seq;s.flags|=J_STATUS_ALERT;s.light_event=1;s.alert_elapsed_ms=20;feedback(&f,&s,20,100);
    CHECK(f.alert_start_ms==20);
    for(t=20;t<3020;++t){f.last_ms=t;CHECK(JFeedback_Buzzer(&f,t));}
    f.last_ms=3020;CHECK(!JFeedback_Buzzer(&f,3020));
    memset(&f,0,sizeof(f));s=status();s.flags|=J_STATUS_ALERT;s.light_event=1;feedback(&f,&s,0,100);
    ++s.telemetry_seq;s.flags=J_STATUS_ESTOP|J_STATUS_FAULT;feedback(&f,&s,50,100);CHECK(!f.alert_on&&!f.success_on);
}
static void absolute_echo_age_rejects_old_ack_when_tx_paused(void)
{
    JFeedback f={0};JStatusPacket s=status();uint8_t p[32];feedback(&f,&s,1000,100);
    ++s.telemetry_seq;s.takeover_event=1;s.mode=1;JStatus_Encode(p,&s);
    CHECK(!JFeedback_Accept(&f,p,1200,0x12345678U,100,1000)&&!f.success_on);
}
int main(void)
{
    RUN(eight_channel_bounds_and_mapping);RUN(invalid_packets_and_all_single_bit_errors);
    RUN(first_duplicate_reorder_and_bad_no_liveness);RUN(simultaneous_keys_loss_and_modular_wrap);
    RUN(reboot_and_reconnect_resynchronization);RUN(physical_debounce_boot_hold_and_all_keys);
    RUN(contact_bounce_no_false_press);RUN(telemetry_roundtrip_and_fresh_echo);
    RUN(takeover_once_and_old_reply_never_success);RUN(reboot_snapshot_and_out_of_order_feedback);
    RUN(continuous_three_second_alarm_and_click);RUN(direction_patterns_and_height_same_tone);
    RUN(whole_fault_and_direction_groups);RUN(stale_feedback_and_emergency_priority);RUN(feedback_timer_wrap);
    RUN(validated_physical_c2_first_frame_is_not_an_aux_edge);RUN(new_event_full_three_seconds_and_estop_discards_it);
    RUN(absolute_echo_age_rejects_old_ack_when_tx_paused);
    printf("RESULT cases=%u checks=%u\n",cases,checks);return 0;
}
