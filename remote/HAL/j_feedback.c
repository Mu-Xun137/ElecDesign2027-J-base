#include "j_feedback.h"
static uint8_t newer(uint16_t a,uint16_t b)
{ uint16_t d=(uint16_t)(a-b); return d>0U && d<32768U; }
uint8_t JFeedback_Accept(JFeedback *f,const uint8_t *p,uint32_t now,
                         uint32_t session,uint16_t latest,uint32_t latest_ms)
{
    JStatusPacket s;
    uint8_t sync;
    if(!JStatus_Decode(p,&s) || s.control_session!=session ||
       (uint32_t)(now-latest_ms)>J_FEEDBACK_ECHO_MAX_MS ||
       (uint16_t)(latest-s.echo_seq)>=J_FEEDBACK_ECHO_WINDOW) return 0;
    if(f->have && s.flight_session==f->retired_flight_session && s.flight_session!=f->status.flight_session) return 0;
    sync=!f->have || s.flight_session!=f->status.flight_session ||
         (uint32_t)(now-f->last_ms)>J_FEEDBACK_FRESH_MS;
    if(!sync && !newer(s.telemetry_seq,f->status.telemetry_seq)) return 0;
    if(sync) {
        if(f->have && s.flight_session!=f->status.flight_session) f->retired_flight_session=f->status.flight_session;
        f->takeover_seen=s.takeover_event; /* never replay success on reboot/reconnection */
        f->success_on=0;
        f->light_seen=s.light_event;
        f->alert_on=(s.flags&J_STATUS_ALERT) && s.alert_elapsed_ms<J_ALERT_MS;
        if(f->alert_on) f->alert_start_ms=now-s.alert_elapsed_ms;
    } else {
        if(newer(s.light_event,f->light_seen)) {
            f->light_seen=s.light_event;
            if((s.flags&J_STATUS_ALERT) && s.alert_elapsed_ms<J_ALERT_MS) {
                f->alert_on=1; f->alert_start_ms=now; /* a normally received NEW event sounds a full 3s */
            }
        }
        if(newer(s.takeover_event,f->takeover_seen)) {
            f->takeover_seen=s.takeover_event;
            if(s.mode==1U && !(s.flags&(J_STATUS_ALIGN|J_STATUS_ESTOP))) {
                f->success_on=1; f->success_ms=now;
            }
        }
    }
    if((s.flags&J_STATUS_ESTOP) || !(s.flags&J_STATUS_ARMED)) f->alert_on=f->success_on=f->click_on=0;
    f->status=s; f->last_ms=now; f->have=1;
    return 1;
}
void JFeedback_Click(JFeedback *f,uint32_t now) { f->click_on=1; f->click_ms=now; }
static uint8_t pattern(uint8_t kind,uint32_t t)
{
    if(kind==1U) return (t<4U*(J_BEEP_FAULT_MS+J_BEEP_FAULT_GAP_MS)) &&
        (t%(J_BEEP_FAULT_MS+J_BEEP_FAULT_GAP_MS)<J_BEEP_FAULT_MS);
    if(kind==2U) return t<J_BEEP_SHORT_MS ||
        (t>=J_BEEP_SHORT_MS+J_BEEP_GAP_MS && t<J_BEEP_SHORT_MS+J_BEEP_GAP_MS+J_BEEP_LONG_MS);
    if(kind==3U) return t<J_BEEP_LONG_MS ||
        (t>=J_BEEP_LONG_MS+J_BEEP_GAP_MS && t<J_BEEP_LONG_MS+J_BEEP_GAP_MS+J_BEEP_SHORT_MS);
    return 0;
}
uint8_t JFeedback_Buzzer(JFeedback *f,uint32_t now)
{
    uint8_t fresh=f->have && (uint32_t)(now-f->last_ms)<=J_FEEDBACK_FRESH_MS;
    uint8_t fault=f->have && (!fresh || (f->status.flags&(J_STATUS_FAULT|J_STATUS_ESTOP)));
    uint8_t direction=fresh && (f->status.flags&J_STATUS_ALIGN)?f->status.align:0;
    uint8_t desired=0;
    if(f->alert_on && (uint32_t)(now-f->alert_start_ms)>=J_ALERT_MS) f->alert_on=0;
    if(f->success_on && (uint32_t)(now-f->success_ms)>=2U*J_BEEP_SHORT_MS+J_BEEP_GAP_MS) f->success_on=0;
    if(f->click_on && (uint32_t)(now-f->click_ms)>=J_BEEP_CLICK_MS) f->click_on=0;
    if(fault || direction==4U) {
        desired=1;
        if(direction==1U || direction==2U) {
            if(f->group_kind && (uint32_t)(now-f->group_ms)<J_BEEP_GROUP_MS)
                return pattern(f->group_kind,(uint32_t)(now-f->group_ms));
            f->alternate^=1U;
            desired=f->alternate?1U:(direction==1U?2U:3U);
        }
    } else if(f->alert_on) { f->group_kind=0; return 1; } /* uninterrupted continuous 3s */
    else if(direction==1U || direction==2U) desired=direction==1U?2U:3U;
    if(desired) {
        if(!f->group_kind || (uint32_t)(now-f->group_ms)>=J_BEEP_GROUP_MS ||
           (desired==1U && f->group_kind!=1U && !(direction==1U || direction==2U))) {
            f->group_kind=desired; f->group_ms=now;
        }
        return pattern(f->group_kind,(uint32_t)(now-f->group_ms));
    }
    f->group_kind=0;
    if(f->success_on) {
        uint32_t t=(uint32_t)(now-f->success_ms);
        return t<J_BEEP_SHORT_MS || (t>=J_BEEP_SHORT_MS+J_BEEP_GAP_MS && t<2U*J_BEEP_SHORT_MS+J_BEEP_GAP_MS);
    }
    if(fresh && (f->status.sensors&(J_SENSOR_LOW|J_SENSOR_HIGH))) return now%1000U<J_BEEP_HEIGHT_MS;
    return f->click_on;
}
