#include "j_core.h"
#include <string.h>
#include <math.h>

float J_Clamp(float x, float a, float b)
{ if(x<a) return a; if(x>b) return b; return x; }
float J_Abs(float x) { return x<0.0f?-x:x; }
uint8_t J_Fresh(uint32_t now, uint32_t stamp, uint32_t limit)
{ return (uint32_t)(now-stamp)<=limit; }
float J_ManualThrust(uint16_t throttle)
{
    /* The ORIGINAL mapping, including its 200-PWM intercept. Never rescale 1819 to 2000. */
    float command=200.0f+0.45f*((float)throttle-1000.0f);
    return (float)(int16_t)J_Clamp(command, 0.0f, J_THR_AUTO_MAX);
}
static float slew(float x, float target, float amount)
{ float difference=target-x; return x+J_Clamp(difference, -amount, amount); }
static uint8_t finite_value(float x)
{ return x==x && x>-1.0e8f && x<1.0e8f; }
uint8_t JCore_HeightHealthy(const JInput *i)
{
    return i->s.range_ok && finite_value(i->s.height_mm) &&
        i->s.height_mm>=J_HEIGHT_MIN_MM && i->s.height_mm<=J_HEIGHT_MAX_MM &&
        J_Fresh(i->now_ms,i->s.range_ms,J_RANGE_FRESH_MS);
}
uint8_t JCore_FlowHealthy(const JInput *i)
{
    return i->s.flow_ok && i->s.quality>=J_FLOW_MIN_QUALITY &&
        finite_value(i->s.x_px) && finite_value(i->s.y_px) &&
        J_Fresh(i->now_ms,i->s.flow_ms,J_FLOW_FRESH_MS);
}
uint8_t JCore_Stable(const JInput *i)
{
    return JCore_HeightHealthy(i) && JCore_FlowHealthy(i) && i->s.velocity_ok &&
        finite_value(i->s.vz_mm_s) && finite_value(i->s.vx_px_s) && finite_value(i->s.vy_px_s) &&
        i->s.quality>=J_FLOW_STABLE_QUALITY &&
        J_Abs(i->s.height_mm-J_TARGET_MM)<=J_STABLE_HEIGHT_MM &&
        J_Abs(i->s.vz_mm_s)<=J_STABLE_VZ_MM_S &&
        J_Abs(i->s.vx_px_s)<=J_STABLE_FLOW_PX_S &&
        J_Abs(i->s.vy_px_s)<=J_STABLE_FLOW_PX_S &&
        J_Abs(i->s.roll_deg)<=J_STABLE_TILT_DEG && J_Abs(i->s.pitch_deg)<=J_STABLE_TILT_DEG;
}
void JCore_Init(JCore *c, uint32_t now)
{
    memset(c,0,sizeof(*c));
    c->mode=J_LOCKED;
    c->phase=J_GROUND;
    c->light_eligible=1;
    c->last_tick_ms=now;
    c->led_epoch_ms=now;
    JNav_Init(&c->nav);
}
void JCore_Stop(JCore *c, JFault reason, uint32_t now)
{
    uint16_t light=c->light_event, takeover=c->takeover_event;
    uint32_t sample=c->last_light_id;
    uint32_t led_epoch=c->led_epoch_ms;
    uint8_t same_locked_state=!c->armed && c->fault==reason;
    JCore_Init(c,now);
    c->light_event=light;
    c->takeover_event=takeover;
    c->last_light_id=sample;
    c->fault=reason;
    if(same_locked_state)c->led_epoch_ms=led_epoch;
}
static void capture(JCore *c, const JInput *i)
{
    c->x_target_px=i->s.x_px;
    c->y_target_px=i->s.y_px;
    c->yaw_target=i->s.yaw_deg;
    c->flow_moving_x=c->flow_moving_y=0;
    c->flow_ix=c->flow_iy=c->prev_vx_error=c->prev_vy_error=0.0f;
    c->flow_roll=c->flow_pitch=0.0f;
}
static void cancel_nav(JCore *c)
{ c->nav_running=c->nav_pending=0; JNav_Init(&c->nav); }
static void auto_enter(JCore *c, const JInput *i, JMode mode)
{
    c->mode=mode;
    c->height_target_mm=i->s.height_mm;
    c->goal_mm=(mode==J_C3_AUTO)?J_TARGET_MM:i->s.height_mm;
    c->vz_integral=c->common-J_THR_HOVER;
    c->vz_integral=J_Clamp(c->vz_integral,-J_VZ_I_LIMIT,J_VZ_I_LIMIT);
    c->stable_active=c->hold_active=c->match_active=0;
    c->align=J_ALIGN_NONE;
    c->c1_centered=0;
    c->c1_launch_ready=c->motors_on;
    c->low_active=0;
    c->fault=J_OK;
    capture(c,i);
}
static JNavInput nav_input(const JCore *c, const JInput *i)
{
    JNavInput n;
    memset(&n,0,sizeof(n));
    n.x_mm=i->s.nav_x_mm; n.y_mm=i->s.nav_y_mm;
    n.lux=i->s.lux; n.baseline=c->baseline_lux;
    n.now_ms=i->now_ms; n.light_id=i->s.light_id;
    n.coordinate_ok=i->s.nav_reference_ok && finite_value(n.x_mm) && finite_value(n.y_mm) &&
        J_Fresh(i->now_ms,i->s.flow_ms,J_NAV_CONTINUITY_GAP_MS);
    n.sample_ok=i->s.light_ok && J_Fresh(i->now_ms,i->s.light_ms,J_LIGHT_FRESH_MS);
    n.stable=JCore_Stable(i); n.light_inside=c->light_inside;
    return n;
}
static void begin_light(JCore *c, const JInput *i, uint8_t new_event)
{
    if(new_event) { ++c->light_event; c->alert_ms=i->now_ms; c->alert_active=1; }
    auto_enter(c,i,J_LIGHT_HOLD);
    c->phase=J_LIGHT_SETTLE;
    c->goal_mm=J_TARGET_MM;
    c->light_task_ms=i->now_ms;
    c->completed=0;
    c->task_failed=0;
    c->arrival=0;
}
static void finish_light(JCore *c, const JInput *i, uint8_t success)
{
    JNavInput n=nav_input(c,i);
    c->mode=J_C3_AUTO;
    c->phase=success?J_TASK_DONE:J_TASK_FAILED;
    c->goal_mm=J_TARGET_MM;
    c->stable_active=c->hold_active=0;
    c->completed=success;
    c->task_failed=!success;
    c->fault=success?J_OK:J_ERR_LIGHT_TIMEOUT;
    if(c->nav_running) {
        JNav_HoverFinished(&c->nav,success,&n);
        if(c->nav.phase==J_NAV_B_STAGE) {
            c->completed=0;
            c->phase=J_WAIT_B;
        } else {
            c->nav_completed=success && c->nav.phase==J_NAV_DONE;
            c->nav_running=0;
        }
    }
}
static void light_samples(JCore *c, const JInput *i)
{
    uint8_t k;
    float sum=0.0f;
    uint8_t fresh=i->s.light_ok && finite_value(i->s.lux) &&
        J_Fresh(i->now_ms,i->s.light_ms,J_LIGHT_FRESH_MS);
    if(!fresh) { c->light_count=c->exit_count=0; return; }
    if(i->s.light_id==c->last_light_id) return;
    c->last_light_id=i->s.light_id;
    if(!c->armed) {
        c->ambient[c->ambient_index]=i->s.lux;
        c->ambient_index=(uint8_t)((c->ambient_index+1U)%J_AMBIENT_SAMPLES);
        if(c->ambient_count<J_AMBIENT_SAMPLES) ++c->ambient_count;
        for(k=0;k<c->ambient_count;++k) sum+=c->ambient[k];
        c->baseline_lux=sum/(float)c->ambient_count;
        return;
    }
    /* Baseline freezes on manual unlock. A new in-air mission never recaptures it. */
    if(c->light_inside) {
        if(i->s.lux<c->baseline_lux+J_LIGHT_DELTA_LUX-J_LIGHT_HYST_LUX) ++c->exit_count;
        else c->exit_count=0;
        if(c->exit_count>=J_LIGHT_CONFIRM_SAMPLES) {
            c->light_inside=0; c->arrival=0; c->light_eligible=1;
            c->light_count=c->exit_count=0;
        }
        return;
    }
    if(!c->enabled || c->phase==J_ADJUST || c->mode==J_LASER_WAIT ||
       c->mode==J_FLOW_HOLD || c->mode==J_LIGHT_HOLD || !c->light_eligible) {
        c->light_count=0; return;
    }
    if(i->s.lux>=c->baseline_lux+J_LIGHT_DELTA_LUX) ++c->light_count;
    else c->light_count=0;
    if(c->light_count<J_LIGHT_CONFIRM_SAMPLES) return;
    c->light_inside=1; c->light_eligible=0; c->light_count=0;
    if(c->nav_running) return; /* candidate sampling cannot repeatedly sound the source alarm */
    ++c->light_event; c->alert_ms=i->now_ms; c->alert_active=1;
    if(!c->nav_running && JCore_HeightHealthy(i) && JCore_FlowHealthy(i)) begin_light(c,i,0);
}
static uint8_t imu_healthy(const JInput *i)
{
    return i->s.imu_ok && J_Fresh(i->now_ms,i->s.imu_ms,J_IMU_FRESH_MS) &&
        finite_value(i->s.roll_deg) && finite_value(i->s.pitch_deg) && finite_value(i->s.yaw_deg);
}
static uint8_t sticks_centered(const JCore *c, const JInput *i)
{
    return J_Abs((float)i->channels[0]-1500.0f-(float)c->roll_center)<=J_STICK_CENTER_TOL &&
        J_Abs((float)i->channels[1]-1500.0f-(float)c->pitch_center)<=J_STICK_CENTER_TOL &&
        J_Abs((float)i->channels[3]-1500.0f)<=J_STICK_CENTER_TOL;
}
static void manual_unlock(JCore *c, const JInput *i)
{
    uint16_t thr=i->channels[2];
    if(!i->new_rc) return;
    if(c->arm_stage && (uint32_t)(i->now_ms-c->arm_ms)>J_UNLOCK_SEQUENCE_MS) c->arm_stage=0;
    if(c->arm_stage==0) {
        if(thr<1150U && i->channels[3]>=1150U) {
            c->arm_stage=1; c->arm_ms=i->now_ms; c->arm_high_active=0;
        }
    } else if(c->arm_stage==1) {
        if(thr>1800U) {
            if(!c->arm_high_active) { c->arm_high_active=1; c->arm_high_ms=i->now_ms; }
            if((uint32_t)(i->now_ms-c->arm_high_ms)>=J_UNLOCK_HIGH_HOLD_MS) c->arm_stage=2;
        } else c->arm_high_active=0;
    } else if(thr<1100U && imu_healthy(i)) {
        c->armed=1; c->mode=J_MANUAL; c->phase=J_GROUND;
        c->fault=J_OK; c->arm_stage=0;
        c->led_epoch_ms=i->now_ms;
        c->ground_valid=JCore_HeightHealthy(i);
        c->ground_mm=i->s.height_mm;
        c->roll_center=(int16_t)i->channels[0]-1500;
        c->pitch_center=(int16_t)i->channels[1]-1500;
        capture(c,i);
    }
}
static void request_align(JCore *c, const JInput *i)
{
    cancel_nav(c);
    c->resume_mode=c->mode;
    c->mode=J_ALIGN;
    c->phase=J_HOVER;
    c->height_target_mm=c->goal_mm=i->s.height_mm;
    c->match_active=0;
    c->low_active=0;
    capture(c,i);
}
static void keys(JCore *c, const JInput *i)
{
    uint8_t e=i->events;
    /* C2 and the raw-stick emergency have already been processed, independently. */
    if(c->mode==J_LIGHT_HOLD || c->mode==J_LASER_WAIT) return;
    if(c->mode==J_ALIGN) {
        if(e&J_C4) {
            c->mode=c->resume_mode; c->align=J_ALIGN_NONE; c->match_active=0;
            c->c1_centered=0;
            if(!c->motors_on)c->c1_launch_ready=0;
            if(c->mode==J_C3_AUTO) {
                c->goal_mm=J_TARGET_MM;
                if(J_Abs(i->s.height_mm-J_TARGET_MM)>J_STABLE_HEIGHT_MM) {
                    c->phase=J_ADJUST; c->adjust_ms=i->now_ms; c->stable_active=0;
                }
            }
        }
        return;
    }
    if(e&J_C4) {
        if(c->mode!=J_MANUAL) request_align(c,i);
        return;
    }
    if(e&J_C1) {
        if(c->phase==J_ADJUST) return;
        if(c->mode==J_C1_HOLD) { request_align(c,i); return; }
        if(JCore_HeightHealthy(i) && (JCore_FlowHealthy(i) ||
           (i->s.height_mm<=J_AIRBORNE_MM && i->s.module_ok))) {
            cancel_nav(c); auto_enter(c,i,J_C1_HOLD); c->phase=J_HOVER;
        } else c->fault=J_ERR_PREFLIGHT;
        return;
    }
    if((e&J_C3) && c->mode!=J_C3_AUTO) {
        uint8_t flow_ready=JCore_FlowHealthy(i) ||
            (i->s.height_mm<=J_AIRBORNE_MM && i->s.module_ok &&
             J_Fresh(i->now_ms,i->s.range_ms,J_RANGE_FRESH_MS));
        if(!JCore_HeightHealthy(i) || !flow_ready || !imu_healthy(i) ||
           c->ambient_count<J_AMBIENT_SAMPLES || !i->s.light_ok ||
           !J_Fresh(i->now_ms,i->s.light_ms,J_LIGHT_FRESH_MS)) {
            c->fault=J_ERR_PREFLIGHT; return;
        }
        cancel_nav(c);
        c->nav_requested=c->nav_pending=J_NAV_MODE!=0; c->nav_completed=0;
        auto_enter(c,i,J_C3_AUTO);
        c->phase=J_ADJUST; c->adjust_ms=i->now_ms;
        c->completed=c->task_failed=0;
        c->light_count=c->exit_count=0;
        c->light_eligible=!c->light_inside;
        c->height_alarm=0;
        c->motors_on=1;
    }
}
static void height_step(JCore *c, const JInput *i, float dt)
{
    float vel, error, proposed, raw;
    if(c->mode==J_MANUAL) {
        if(i->channels[2]>1100U) c->motors_on=1;
        if(i->channels[2]<1010U) {
            if(!c->low_active) { c->low_active=1; c->low_ms=i->now_ms; }
            if((uint32_t)(i->now_ms-c->low_ms)>=J_LOW_THR_STOP_MS) c->motors_on=0;
            if((uint32_t)(i->now_ms-c->low_ms)>=J_LOW_THR_LOCK_MS) {
                JCore_Stop(c,J_OK,i->now_ms); return;
            }
        } else c->low_active=0;
        c->common=c->motors_on?J_ManualThrust(i->channels[2]):0.0f;
        c->last_manual=c->common;
        return;
    }
    c->low_active=0;
    if(!c->motors_on) {
        c->common=0.0f;c->vz_integral=0.0f;
        if(c->mode==J_C1_HOLD) {
            if(i->channels[2]<=1100U)c->c1_launch_ready=1;
            if(c->c1_launch_ready && i->channels[2]>1100U)c->motors_on=1;
        }
        if(!c->motors_on)return;
    }
    if(c->mode==J_LASER_WAIT) return; /* last valid limited collective; NO height claim */
    if(!JCore_HeightHealthy(i) || !i->s.velocity_ok) return; /* freeze integral on gaps */
    if(c->mode==J_C1_HOLD) {
        if(i->channels[2]>1150U && i->channels[2]<1750U) c->c1_centered=1;
        if(c->c1_centered) {
            if(i->channels[2]>1750U) c->goal_mm+=J_CLIMB_MAX_MM_S*dt;
            if(i->channels[2]<1150U) c->goal_mm-=J_DESCEND_MAX_MM_S*dt;
            c->goal_mm=J_Clamp(c->goal_mm,J_HEIGHT_MIN_MM,J_HEIGHT_MAX_MM);
        }
    }
    c->height_target_mm=slew(c->height_target_mm,c->goal_mm,J_HEIGHT_TARGET_MM_S*dt);
    vel=(c->height_target_mm-i->s.height_mm)*J_HEIGHT_P;
    vel=J_Clamp(vel,-J_DESCEND_MAX_MM_S,J_CLIMB_MAX_MM_S);
    error=vel-i->s.vz_mm_s;
    proposed=c->vz_integral+J_VZ_I*error*dt;
    proposed=J_Clamp(proposed,-J_VZ_I_LIMIT,J_VZ_I_LIMIT);
    raw=J_THR_HOVER+J_VZ_P*error+proposed;
    if(!((raw>J_THR_AUTO_MAX && error>0.0f)||(raw<0.0f && error<0.0f))) c->vz_integral=proposed;
    raw=J_THR_HOVER+J_VZ_P*error+c->vz_integral;
    raw=J_Clamp(raw,0.0f,J_THR_AUTO_MAX);
    c->common=slew(c->common,raw,J_THR_SLEW_PER_S*dt);
}
static void align_step(JCore *c, const JInput *i)
{
    float candidate=(!c->motors_on && i->channels[2]<=1100U)?0.0f:J_ManualThrust(i->channels[2]);
    float common=(float)(int16_t)c->common; /* the actual limited, pre-mixer motor command */
    float minimum=c->motors_on?J_ManualThrust(1000U):0.0f;
    uint8_t in_range=common>=minimum-J_ALIGN_TOL_PWM &&
        common<=J_ManualThrust(J_MANUAL_THR_MAX_CHANNEL)+J_ALIGN_TOL_PWM;
    if(!in_range) c->align=J_ALIGN_UNREACHABLE;
    else if(candidate<common-J_ALIGN_TOL_PWM) c->align=J_ALIGN_UP;
    else if(candidate>common+J_ALIGN_TOL_PWM) c->align=J_ALIGN_DOWN;
    else c->align=J_ALIGN_MATCH;
    if(c->align!=J_ALIGN_MATCH || !sticks_centered(c,i) ||
       !i->have_rc || !J_Fresh(i->now_ms,i->rc_ms,J_ALIGN_RC_FRESH_MS) ||
       !imu_healthy(i) || (c->mode!=J_LASER_WAIT && !JCore_HeightHealthy(i))) {
        c->match_active=0; return;
    }
    if(!c->match_active) { c->match_active=1; c->match_ms=i->now_ms; }
    if((uint32_t)(i->now_ms-c->match_ms)<J_ALIGN_HOLD_MS) return;
    /* Same cycle and same formula as the next manual cycle; attitude targets slew. */
    c->common=candidate;
    c->mode=J_MANUAL; c->phase=J_HOVER;
    c->motors_on=c->motors_on || i->channels[2]>1100U;
    c->vz_integral=0.0f; c->flow_ix=c->flow_iy=0.0f;
    c->low_active=0; c->match_active=0; c->align=J_ALIGN_NONE;
    ++c->takeover_event;
}
static void flow_axis(float *target, uint8_t *moving, float position, int stick,
                      uint8_t allow_stick, float *vel)
{
    *vel=0.0f; /* always initialize BOTH axes; fixes residual one-axis command */
    if(allow_stick && (stick>250 || stick<-250)) {
        *moving=1;
        *vel=(stick>0)?-J_FLOW_MANUAL_PX_S:J_FLOW_MANUAL_PX_S;
    } else {
        if(*moving) { *target=position; *moving=0; }
        *vel=(*target-position)*J_FLOW_POS_P;
        *vel=J_Clamp(*vel,-J_FLOW_MAX_PX_S,J_FLOW_MAX_PX_S);
    }
}
static void horizontal_step(JCore *c, const JInput *i, float dt)
{
    float roll=0.0f,pitch=0.0f,flow_dt,vx=0.0f,vy=0.0f,ex,ey;
    uint8_t fixed=c->mode==J_ALIGN || c->mode==J_LIGHT_HOLD || c->nav_running;
    uint8_t use_flow=c->mode!=J_MANUAL && c->mode!=J_FLOW_HOLD && c->mode!=J_LASER_WAIT;
    if(!fixed) {
        if(i->channels[3]>1820U) c->yaw_target-=J_YAW_RATE_DEG_S*dt;
        else if(i->channels[3]<1180U) c->yaw_target+=J_YAW_RATE_DEG_S*dt;
    }
    if(use_flow && i->s.height_mm>=J_AIRBORNE_MM) {
        if(!JCore_FlowHealthy(i)) { roll=c->flow_roll; pitch=c->flow_pitch; }
        else {
            if(!c->flow_tick_started || (uint32_t)(i->now_ms-c->flow_tick_ms)>=J_FLOW_CONTROL_MS) {
                flow_dt=c->flow_tick_started?(float)(uint32_t)(i->now_ms-c->flow_tick_ms)*0.001f:0.024f;
                flow_dt=J_Clamp(flow_dt,0.003f,0.1f);
                c->flow_tick_ms=i->now_ms; c->flow_tick_started=1;
                flow_axis(&c->x_target_px,&c->flow_moving_x,i->s.x_px,
                    (int)i->channels[0]-1500-c->roll_center,!fixed,&vx);
                /* Original pitch stick moves +Y, roll stick moves -X. */
                flow_axis(&c->y_target_px,&c->flow_moving_y,i->s.y_px,
                    -((int)i->channels[1]-1500-c->pitch_center),!fixed,&vy);
                ex=vx-i->s.vx_px_s; ey=vy-i->s.vy_px_s;
                c->flow_ix+=J_FLOW_RATE_I*ex*flow_dt;
                c->flow_iy+=J_FLOW_RATE_I*ey*flow_dt;
                c->flow_ix=J_Clamp(c->flow_ix,-J_FLOW_I_LIMIT_DEG,J_FLOW_I_LIMIT_DEG);
                c->flow_iy=J_Clamp(c->flow_iy,-J_FLOW_I_LIMIT_DEG,J_FLOW_I_LIMIT_DEG);
                c->flow_roll=J_FLOW_RATE_P*ex+c->flow_ix+J_FLOW_RATE_D*(ex-c->prev_vx_error)/flow_dt;
                c->flow_pitch=J_FLOW_RATE_P*ey+c->flow_iy+J_FLOW_RATE_D*(ey-c->prev_vy_error)/flow_dt;
                c->flow_roll=J_Clamp(c->flow_roll,-J_FLOW_MAX_TILT_DEG,J_FLOW_MAX_TILT_DEG);
                c->flow_pitch=-J_Clamp(c->flow_pitch,-J_FLOW_MAX_TILT_DEG,J_FLOW_MAX_TILT_DEG);
                c->prev_vx_error=ex; c->prev_vy_error=ey;
            }
            roll=c->flow_roll; pitch=c->flow_pitch;
        }
    } else if(!fixed && (c->mode==J_MANUAL || c->mode==J_FLOW_HOLD || c->mode==J_LASER_WAIT)) {
        roll=-((float)i->channels[0]-1500.0f-c->roll_center)*0.04f;
        pitch=-((float)i->channels[1]-1500.0f-c->pitch_center)*0.04f;
    }
    c->roll_cmd=slew(c->roll_cmd,J_Clamp(roll,-20.0f,20.0f),J_ATTITUDE_SLEW_DEG_S*dt);
    c->pitch_cmd=slew(c->pitch_cmd,J_Clamp(pitch,-20.0f,20.0f),J_ATTITUDE_SLEW_DEG_S*dt);
}
static void fault_checks(JCore *c, const JInput *i)
{
    if(!J_Fresh(i->now_ms,i->s.range_ms,J_RANGE_FAIL_MS)) {
        if(c->mode==J_MANUAL) { if(c->enabled) c->fault=J_ERR_RANGE; return; }
        if(c->mode!=J_LASER_WAIT) {
            cancel_nav(c); c->mode=J_LASER_WAIT; c->phase=J_TASK_FAILED;
            c->task_failed=1; c->fault=J_ERR_RANGE; c->fault_ms=i->now_ms;
            c->arrival=c->hold_active=c->stable_active=0;
            c->match_active=0; capture(c,i);
            c->common=J_Clamp(c->common,0.0f,J_THR_AUTO_MAX);
        }
    }
    if(c->mode==J_LASER_WAIT) {
        if((uint32_t)(i->now_ms-c->fault_ms)>=J_LASER_WAIT_MS) JCore_Stop(c,J_ERR_RANGE,i->now_ms);
        return; /* sensor recovery cannot silently leave this explicit handover window */
    }
    if(i->s.height_mm>=J_AIRBORNE_MM && !JCore_FlowHealthy(i)) {
        if(!c->flow_bad_active) { c->flow_bad_active=1; c->flow_bad_ms=i->now_ms; }
    } else c->flow_bad_active=0;
    if(i->s.height_mm>=J_AIRBORNE_MM && c->flow_bad_active && (uint32_t)(i->now_ms-c->flow_bad_ms)>=J_FLOW_FAIL_MS) {
        if(c->mode==J_MANUAL) { if(c->enabled) c->fault=J_ERR_FLOW; return; }
        if(c->mode!=J_FLOW_HOLD && !(c->mode==J_ALIGN && c->resume_mode==J_FLOW_HOLD) && JCore_HeightHealthy(i)) {
            cancel_nav(c); c->mode=J_FLOW_HOLD; c->phase=J_TASK_FAILED;
            c->task_failed=1; c->fault=J_ERR_FLOW; c->match_active=0;
            c->arrival=c->hold_active=c->stable_active=0;
            c->goal_mm=c->height_target_mm=i->s.height_mm;
            c->align=J_ALIGN_NONE; c->flow_ix=c->flow_iy=0.0f;
        }
    }
}
static void navigation_step(JCore *c, const JInput *i)
{
    JNavInput n=nav_input(c,i);
    if(!c->nav_running) return;
    JNav_Step(&c->nav,&n);
    if(c->nav.phase==J_NAV_FAILED) {
        c->nav_running=0; c->task_failed=1; c->fault=J_ERR_NAV; c->phase=J_TASK_FAILED;
        c->nav_completed=0;
        if(c->mode==J_LIGHT_HOLD) { c->mode=J_C3_AUTO; c->goal_mm=J_TARGET_MM; c->hold_active=c->stable_active=0; }
        capture(c,i);
    } else if(c->nav.phase==J_NAV_FORCE && c->mode!=J_LIGHT_HOLD) {
        /* A spatially distinct B is a new source even when no dark gap was observable. */
        c->light_inside=1;
        begin_light(c,i,1);
    } else if(JNav_Active(&c->nav)) {
        float scale=J_NAV_MM_PER_PX_AT800;
        if(scale>0.0f) {
            float dx=(c->nav.target_x_mm-i->s.nav_x_mm)*J_NAV_POSITION_P;
            float dy=(c->nav.target_y_mm-i->s.nav_y_mm)*J_NAV_POSITION_P;
            float angle=i->s.nav_yaw_deg*0.01745329252f;
            float magnitude;
            float bx,by;
            dx=J_Clamp(dx,-J_NAV_SPEED_MM_S,J_NAV_SPEED_MM_S);
            dy=J_Clamp(dy,-J_NAV_SPEED_MM_S,J_NAV_SPEED_MM_S);
            magnitude=sqrtf(dx*dx+dy*dy);
            if(magnitude>J_NAV_SPEED_MM_S) { dx*=J_NAV_SPEED_MM_S/magnitude; dy*=J_NAV_SPEED_MM_S/magnitude; }
            bx=dx*cosf(angle)+dy*sinf(angle);
            by=-dx*sinf(angle)+dy*cosf(angle);
            scale*=i->s.height_mm/J_TARGET_MM;
            c->x_target_px=i->s.x_px+bx/(scale*J_FLOW_POS_P);
            c->y_target_px=i->s.y_px+by/(scale*J_FLOW_POS_P);
            c->yaw_target=i->s.nav_heading_deg;
        }
    }
}
void JCore_Step(JCore *c, const JInput *i)
{
    float dt;
    uint32_t elapsed=(uint32_t)(i->now_ms-c->last_tick_ms);
    /* Safety events are processed even on a repeated scheduler timestamp. */
    if((i->events&J_C2) || (i->have_rc && J_Fresh(i->now_ms,i->rc_ms,J_RC_LOST_MS) &&
       i->channels[2]<1050U && i->channels[3]<1150U)) {
        JCore_Stop(c,J_ERR_ESTOP,i->now_ms); return;
    }
    if(c->armed && (i->rc_gap || !i->have_rc || !J_Fresh(i->now_ms,i->rc_ms,J_RC_LOST_MS))) {
        JCore_Stop(c,J_ERR_RC,i->now_ms); return;
    }
    if(c->armed && !imu_healthy(i)) { JCore_Stop(c,J_ERR_IMU,i->now_ms); return; }
    if(c->armed && (J_Abs(i->s.roll_deg)>J_MAX_TILT_DEG || J_Abs(i->s.pitch_deg)>J_MAX_TILT_DEG)) {
        JCore_Stop(c,J_ERR_TILT,i->now_ms); return;
    }
    if(c->tick_started && elapsed==0U) return;
    if(c->armed && c->tick_started && elapsed>J_CONTROL_DEADLINE_MS) {
        JCore_Stop(c,J_ERR_DEADLINE,i->now_ms); return;
    }
    c->last_tick_ms=i->now_ms; c->tick_started=1;
    dt=(float)(elapsed?elapsed:3U)*0.001f;
    light_samples(c,i);
    if(!c->armed) { if(i->have_rc) manual_unlock(c,i); return; }
    if(JCore_HeightHealthy(i) && (c->ground_valid?
       i->s.height_mm>c->ground_mm+J_LIFTOFF_DELTA_MM:i->s.height_mm>J_AIRBORNE_MM)) c->airborne=1;
    fault_checks(c,i);
    if(!c->armed) return;
    keys(c,i);
    height_step(c,i,dt);
    if(!c->armed) return;
    if(c->mode==J_ALIGN || c->mode==J_LASER_WAIT) align_step(c,i);
    if(c->mode==J_C3_AUTO && c->phase==J_ADJUST) {
        if(JCore_Stable(i) && J_Abs(c->height_target_mm-J_TARGET_MM)<=1.0f) {
            if(!c->stable_active) { c->stable_active=1; c->stable_ms=i->now_ms; }
            if((uint32_t)(i->now_ms-c->stable_ms)>=J_STABLE_CONFIRM_MS) {
                c->enabled=1; c->phase=J_HOVER; c->stable_active=0;
                if(c->nav_pending) {
                    JNavInput n=nav_input(c,i);
                    c->nav_pending=0;
                    c->nav_running=JNav_Start(&c->nav,&n);
                    if(c->nav_running) c->phase=J_SEARCH;
                    else { c->task_failed=1; c->phase=J_TASK_FAILED; c->fault=J_ERR_NAV; }
                }
            }
        } else c->stable_active=0;
        if(c->phase==J_ADJUST && (uint32_t)(i->now_ms-c->adjust_ms)>=J_LIFTOFF_TIMEOUT_MS) {
            if(!c->airborne && i->s.height_mm<=J_AIRBORNE_MM) {
                JCore_Stop(c,J_ERR_TAKEOFF_GROUND,i->now_ms); return;
            }
            c->phase=J_TASK_FAILED; c->task_failed=1; c->fault=J_ERR_TAKEOFF_AIR;
        }
    }
    if(c->mode==J_LIGHT_HOLD && c->nav_running) navigation_step(c,i);
    if(c->mode==J_LIGHT_HOLD) {
        uint8_t evidence=i->s.light_ok && J_Fresh(i->now_ms,i->s.light_ms,J_LIGHT_FRESH_MS) &&
            c->light_inside && i->s.lux>=c->baseline_lux+J_LIGHT_DELTA_LUX;
        if(evidence && JCore_Stable(i)) {
            if(!c->stable_active) { c->stable_active=1; c->stable_ms=i->now_ms; }
            if(!c->hold_active && (uint32_t)(i->now_ms-c->stable_ms)>=J_STABLE_CONFIRM_MS) {
                c->arrival=1; c->hold_active=1; c->hold_ms=i->now_ms; c->phase=J_LIGHT_STABLE;
            }
            if(c->hold_active && (uint32_t)(i->now_ms-c->hold_ms)>=J_LIGHT_HOLD_MS) finish_light(c,i,1);
        } else {
            c->stable_active=c->hold_active=0;
            c->phase=J_LIGHT_SETTLE;
        }
        if(c->mode==J_LIGHT_HOLD && (!c->light_inside ||
           (uint32_t)(i->now_ms-c->light_task_ms)>=J_LIGHT_TIMEOUT_MS ||
           !J_Fresh(i->now_ms,i->s.light_ms,J_LIGHT_FRESH_MS))) finish_light(c,i,0);
    }
    if(c->mode==J_C3_AUTO && c->phase!=J_ADJUST) navigation_step(c,i);
    horizontal_step(c,i,dt);
    {
        int8_t alarm=0;
        if(c->enabled && c->phase!=J_ADJUST && JCore_HeightHealthy(i)) {
            if(i->s.height_mm<J_ALARM_LOW_MM) alarm=-1;
            else if(i->s.height_mm>J_ALARM_HIGH_MM) alarm=1;
        }
        if(alarm!=c->height_alarm)c->height_led_ms=i->now_ms;
        c->height_alarm=alarm;
    }
    if(c->enabled && !i->s.light_ok && c->fault==J_OK) c->fault=J_ERR_LIGHT;
}
uint8_t JCore_AlertActive(const JCore *c, uint32_t now)
{ return c->armed && c->alert_active && (uint32_t)(now-c->alert_ms)<J_ALERT_MS; }
uint8_t JCore_LEDMask(const JCore *c, uint32_t now)
{
    if(!c->armed) return (uint8_t)(1U<<(((uint32_t)(now-c->led_epoch_ms)/J_LED_IDLE_MS)%4U));
    if(JCore_AlertActive(c,now)) return ((uint32_t)(now-c->alert_ms)/J_LED_LIGHT_MS)%2U?0U:15U;
    if(c->phase==J_ADJUST) return 0;
    if(c->height_alarm) return (((uint32_t)(now-c->height_led_ms)/J_LED_HEIGHT_MS)%2U)?0U:
        (c->height_alarm<0?J_LED_RED_MASK:J_LED_BLUE_MASK);
    if(c->arrival && c->light_inside) return 15U;
    if(!c->airborne) return (((uint32_t)(now-c->led_epoch_ms)/J_LED_GROUND_MS)%2U)?J_LED_RED_MASK:J_LED_BLUE_MASK;
    return 0;
}
