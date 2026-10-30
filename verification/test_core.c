#include "j_core.h"
#include "j_sensors.h"
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
typedef struct { JCore c; JInput i; uint8_t auto_light,auto_rc,auto_module,auto_range; } Fixture;
static void init_at(Fixture *f,JMode mode,float height,uint32_t now)
{
    unsigned k;
    memset(f,0,sizeof(*f)); JCore_Init(&f->c,now);
    f->i.now_ms=f->i.rc_ms=now; f->i.have_rc=1;
    for(k=0;k<8;++k) f->i.channels[k]=(k==0||k==1||k==3)?1500:1000;
    f->i.s.range_ok=f->i.s.flow_ok=f->i.s.imu_ok=f->i.s.light_ok=1;
    f->i.s.module_ok=f->i.s.velocity_ok=f->i.s.nav_reference_ok=1;
    f->i.s.height_mm=height; f->i.s.quality=80; f->i.s.lux=100;
    f->i.s.range_ms=f->i.s.flow_ms=f->i.s.imu_ms=f->i.s.light_ms=now;
    f->i.s.module_ms=now;
    f->i.s.light_id=1;
    f->c.last_light_id=1; f->c.ambient_count=J_AMBIENT_SAMPLES; f->c.baseline_lux=100;
    f->c.mode=mode; f->c.phase=mode==J_LOCKED?J_GROUND:J_HOVER;
    f->c.armed=mode!=J_LOCKED; f->c.motors_on=f->c.armed;
    f->c.airborne=height>200; f->c.enabled=f->c.armed;
    f->c.height_target_mm=f->c.goal_mm=height;
    f->c.common=f->c.armed?500:0;
    f->auto_light=f->auto_rc=1;
    f->auto_module=f->auto_range=1;
}
static void init(Fixture *f,JMode mode,float h) { init_at(f,mode,h,1000U); }
static void tick(Fixture *f)
{
    JInput *i=&f->i;
    i->now_ms+=3U;
    if(f->auto_rc) { i->rc_ms=i->now_ms; i->new_rc=1; }
    if(f->auto_range && i->s.range_ok) i->s.range_ms=i->now_ms;
    if(f->auto_module && i->s.module_ok)i->s.module_ms=i->now_ms;
    if(i->s.flow_ok) i->s.flow_ms=i->now_ms;
    if(i->s.imu_ok) i->s.imu_ms=i->now_ms;
    if(f->auto_light && i->s.light_ok && (uint32_t)(i->now_ms-i->s.light_ms)>=200U) {
        i->s.light_ms=i->now_ms; ++i->s.light_id;
    }
    JCore_Step(&f->c,i); i->events=0; i->rc_gap=0;
}
static void advance(Fixture *f,unsigned ms) { unsigned n; for(n=0;n<ms;n+=3) tick(f); }
static void event(Fixture *f,uint8_t e) { f->i.events=e; tick(f); }
static void light(Fixture *f,float lux)
{ f->i.s.lux=lux; f->i.s.light_ms=f->i.now_ms; ++f->i.s.light_id; tick(f); }
static void detect(Fixture *f) { light(f,400); light(f,400); light(f,400); }

static void blind_ground_c3_starts_bounded_ramp(void)
{
    Fixture f;init(&f,J_LOCKED,0);
    f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=0;
    tick(&f);f.i.channels[2]=1819;advance(&f,90);
    f.i.channels[2]=1000;tick(&f);
    CHECK(f.c.armed && !f.c.motors_on);
    event(&f,J_C3);
    CHECK(f.c.mode==J_C3_AUTO && f.c.phase==J_ADJUST && f.c.fault==J_OK && f.c.motors_on);
    advance(&f,300);
    CHECK(f.c.common>0 && f.c.common<=400 && !f.c.enabled && !f.c.completed);
}
static void fresh_ground_arm(Fixture *f)
{
    init(f,J_LOCKED,0);f->i.s.range_ok=f->i.s.velocity_ok=f->i.s.flow_ok=0;
    tick(f);f->i.channels[2]=1819;advance(f,90);f->i.channels[2]=1000;tick(f);
    CHECK(f->c.armed && !f->c.motors_on && f->c.launch_available);
}
static void start_near(Fixture *f)
{
    fresh_ground_arm(f);event(f,J_C3);
    CHECK(f->c.near_active && f->c.motors_on && !f->c.launch_available && f->c.fault==J_OK);
}
static void near_entry_guards_and_one_shot_grant(void)
{
    Fixture f;init(&f,J_LOCKED,0);f.i.s.range_ok=f.i.s.flow_ok=0;event(&f,J_C3);
    CHECK(!f.c.armed && !f.c.motors_on && f.c.common==0 && !f.c.near_active);
    fresh_ground_arm(&f);f.i.s.module_ok=0;event(&f,J_C3);
    CHECK(f.c.mode==J_MANUAL && f.c.fault==J_ERR_PREFLIGHT && !f.c.motors_on);
    fresh_ground_arm(&f);f.auto_module=0;advance(&f,J_NEAR_MODULE_FRESH_MS+3U);event(&f,J_C3);
    CHECK(f.c.fault==J_ERR_PREFLIGHT && !f.c.near_active && !f.c.motors_on);
    fresh_ground_arm(&f);f.i.s.roll_deg=9;event(&f,J_C3);
    CHECK(f.c.fault==J_ERR_PREFLIGHT && !f.c.near_active && !f.c.motors_on);
    fresh_ground_arm(&f);f.i.channels[0]=1700;event(&f,J_C3);
    CHECK(f.c.fault==J_ERR_PREFLIGHT && !f.c.near_active && !f.c.motors_on);
    fresh_ground_arm(&f);f.i.channels[2]=1100;event(&f,J_C3);
    CHECK(f.c.fault==J_ERR_PREFLIGHT && !f.c.near_active && !f.c.motors_on);
    fresh_ground_arm(&f);f.i.channels[2]=1200;tick(&f);
    CHECK(f.c.motors_on && !f.c.launch_available);
    f.i.channels[2]=1000;advance(&f,903);CHECK(f.c.armed && !f.c.motors_on);
    event(&f,J_C3);CHECK(f.c.mode==J_MANUAL && f.c.fault==J_ERR_PREFLIGHT && !f.c.near_active);
    f.i.s.range_ok=f.i.s.velocity_ok=1;f.i.s.height_mm=21;
    event(&f,J_C3);CHECK(f.c.mode==J_MANUAL && !f.c.motors_on && !f.c.near_active);
    fresh_ground_arm(&f);f.i.s.range_ok=f.i.s.velocity_ok=1;f.i.s.height_mm=30;
    event(&f,J_C1);CHECK(f.c.mode==J_C1_HOLD && !f.c.motors_on && f.c.launch_available);
    f.i.channels[2]=1200;tick(&f);CHECK(f.c.motors_on && !f.c.launch_available);
    fresh_ground_arm(&f);f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;f.i.s.height_mm=300;
    event(&f,J_C3);CHECK(f.c.mode==J_C3_AUTO && !f.c.near_active && !f.c.launch_available);
}
static void near_absolute_deadline_and_no_restart(void)
{
    Fixture f;uint32_t start;start_near(&f);start=f.c.near_start_ms;
    advance(&f,1500);event(&f,J_C3);event(&f,J_C4);advance(&f,100);event(&f,J_C4);
    CHECK(f.c.near_active && f.c.near_start_ms==start && !f.c.near_align);
    while(f.c.armed && (uint32_t)(f.i.now_ms-start)<J_NEAR_TIMEOUT_MS)tick(&f);
    CHECK(!f.c.armed && !f.c.motors_on && f.c.common==0 && f.c.fault==J_ERR_TAKEOFF_GROUND);
    CHECK(!f.c.near_active && !f.c.launch_available);
    event(&f,J_C3);CHECK(!f.c.armed && f.c.common==0);
}
static void near_all_safety_paths_stop(void)
{
    Fixture f;
    start_near(&f);advance(&f,100);event(&f,J_C1|J_C2|J_C3|J_C4);
    CHECK(!f.c.armed && f.c.common==0 && f.c.fault==J_ERR_ESTOP);
    start_near(&f);f.i.channels[3]=1100;tick(&f);CHECK(!f.c.armed && f.c.fault==J_ERR_ESTOP);
    start_near(&f);f.i.s.module_ok=0;tick(&f);CHECK(!f.c.armed && f.c.common==0 && f.c.fault==J_ERR_RANGE);
    start_near(&f);f.auto_module=0;advance(&f,J_NEAR_MODULE_FRESH_MS+3U);
    CHECK(!f.c.armed && f.c.common==0 && f.c.fault==J_ERR_RANGE);
    start_near(&f);f.i.s.imu_ok=0;tick(&f);CHECK(!f.c.armed && f.c.fault==J_ERR_IMU);
    start_near(&f);f.i.s.roll_deg=16;tick(&f);CHECK(!f.c.armed && f.c.fault==J_ERR_TILT);
    start_near(&f);f.i.s.pitch_deg=-16;tick(&f);CHECK(!f.c.armed && f.c.fault==J_ERR_TILT);
    start_near(&f);f.auto_rc=0;advance(&f,501);CHECK(!f.c.armed && f.c.fault==J_ERR_RC);
    start_near(&f);f.i.rc_gap=1;tick(&f);CHECK(!f.c.armed && f.c.fault==J_ERR_RC);
    start_near(&f);f.i.now_ms+=26;JCore_Step(&f.c,&f.i);
    CHECK(!f.c.armed && f.c.common==0 && f.c.fault==J_ERR_DEADLINE);
}
static void near_ignores_stale_and_low_height(void)
{
    Fixture f;float previous;unsigned k;start_near(&f);
    f.i.s.height_mm=766;f.i.s.x_px=100000;f.i.s.y_px=-100000;
    f.i.s.vx_px_s=10000;f.i.s.vy_px_s=-10000;
    for(k=0;k<100;++k) {
        previous=f.c.common;tick(&f);
        CHECK(f.c.common>=previous && f.c.common-previous<=J_THR_SLEW_PER_S*0.003f+0.001f);
        CHECK(f.c.common<=J_NEAR_THR_PWM && !f.c.airborne && !f.c.enabled && !f.c.stable_active);
        CHECK(f.c.roll_cmd==0 && f.c.pitch_cmd==0 && f.c.height_target_mm==0);
    }
    f.i.s.range_ok=f.i.s.velocity_ok=1;f.i.s.height_mm=21;advance(&f,150);
    CHECK(f.c.near_active && !f.c.near_valid_active && !f.c.airborne && !f.c.enabled);
    f.i.s.height_mm=250;f.i.s.velocity_ok=0;advance(&f,150);
    CHECK(f.c.near_active && !f.c.near_valid_active);
}
static void near_handoff_continuity_and_range_loss(void)
{
    Fixture f;float previous,desired_yaw;start_near(&f);advance(&f,500);
    desired_yaw=f.c.yaw_target;f.i.s.yaw_deg=3;
    f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;f.i.s.height_mm=250;f.i.s.vz_mm_s=100;
    advance(&f,90);CHECK(f.c.near_active);
    f.i.s.range_ok=0;tick(&f);CHECK(!f.c.near_valid_active);
    f.i.s.range_ok=1;advance(&f,99);CHECK(f.c.near_active);
    tick(&f);CHECK(f.c.near_active); /* first fresh sample starts the 100 ms window */
    previous=f.c.common;tick(&f);
    CHECK(!f.c.near_active && f.c.mode==J_C3_AUTO && f.c.phase==J_ADJUST);
    CHECK(f.c.common==previous && f.c.height_target_mm==250 && f.c.goal_mm==800);
    CHECK(f.c.airborne && !f.c.launch_available && !f.c.enabled && f.c.yaw_target==desired_yaw);
    previous=f.c.common;tick(&f);CHECK(J_Abs(f.c.common-previous)<=J_THR_SLEW_PER_S*0.003f+0.001f);
    f.i.s.range_ok=0;advance(&f,310);
    CHECK(f.c.mode==J_LASER_WAIT && f.c.fault==J_ERR_RANGE && !f.c.near_active);
    event(&f,J_C3);CHECK(!f.c.near_active && !f.c.launch_available);
}
static void near_requires_post_start_changing_timestamps(void)
{
    Fixture f;start_near(&f);f.auto_range=0;
    f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;f.i.s.height_mm=250;
    f.i.s.range_ms=f.c.near_start_ms;advance(&f,110);
    CHECK(f.c.near_active && !f.c.near_valid_active);
    start_near(&f);advance(&f,3);f.auto_range=0;
    f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;f.i.s.height_mm=250;
    f.i.s.range_ms=f.i.now_ms;advance(&f,105);
    CHECK(f.c.near_active && f.c.near_valid_reads==1);
    f.auto_range=1;tick(&f);CHECK(!f.c.near_active && f.c.mode==J_C3_AUTO);
}
static void near_c4_holds_collective_and_matches_without_height(void)
{
    Fixture f;float held;start_near(&f);advance(&f,1200);held=f.c.common;
    event(&f,J_C4);advance(&f,30);
    CHECK(f.c.near_active && f.c.near_align && f.c.align==J_ALIGN_UP && f.c.common==held);
    f.i.channels[2]=1445;f.i.channels[0]=1700;advance(&f,210);
    CHECK(f.c.near_active && !f.c.match_active && f.c.common==held);
    f.i.channels[0]=1500;f.auto_rc=0;advance(&f,100);CHECK(f.c.near_active && !f.c.match_active);
    f.auto_rc=1;advance(&f,210);
    CHECK(f.c.mode==J_MANUAL && !f.c.near_active && f.c.common==400);
    CHECK(f.c.takeover_event==1 && !f.c.launch_available);
    advance(&f,1000);CHECK(f.c.armed && f.c.mode==J_MANUAL);
}
static void near_c4_range_recovery_uses_measured_target(void)
{
    Fixture f;float held,yaw;start_near(&f);advance(&f,600);event(&f,J_C4);held=f.c.common;
    yaw=f.c.yaw_target;f.i.s.yaw_deg=7;
    f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;f.i.s.height_mm=250;
    advance(&f,105);
    CHECK(!f.c.near_active && f.c.mode==J_ALIGN && f.c.common==held);
    CHECK(f.c.goal_mm==250 && f.c.height_target_mm==250);
    CHECK(f.c.yaw_target==yaw);
    advance(&f,2400);CHECK(f.c.armed && f.c.mode==J_ALIGN && !f.c.near_active);
}
static void near_wrap_and_repeated_scheduler(void)
{
    Fixture f;float common;uint32_t stamp;unsigned n;
    init_at(&f,J_LOCKED,0,0xFFFFFF00U);f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=0;
    tick(&f);f.i.channels[2]=1819;advance(&f,90);f.i.channels[2]=1000;tick(&f);event(&f,J_C3);
    stamp=f.c.near_start_ms;advance(&f,300);common=f.c.common;
    for(n=0;n<100;++n)JCore_Step(&f.c,&f.i);
    CHECK(f.c.near_active && f.c.common==common && f.c.near_start_ms==stamp);
    while(f.c.armed && (uint32_t)(f.i.now_ms-stamp)<J_NEAR_TIMEOUT_MS)tick(&f);
    CHECK(!f.c.armed && f.c.fault==J_ERR_TAKEOFF_GROUND && f.c.common==0);
}
static void locked_and_manual_unlock(void)
{
    Fixture f; init(&f,J_LOCKED,30); event(&f,J_C3);
    CHECK(!f.c.armed && f.c.mode==J_LOCKED);
    f.i.channels[2]=1819; advance(&f,90); f.i.channels[2]=1000; tick(&f);
    CHECK(f.c.armed && f.c.mode==J_MANUAL && !f.c.motors_on);
}
static void c3_feedback_takeoff(void)
{
    Fixture f; unsigned n; float previous,physical=0.0f;
    fresh_ground_arm(&f);
    event(&f,J_C3);
    CHECK(f.c.mode==J_C3_AUTO && f.c.phase==J_ADJUST && f.c.motors_on);
    CHECK(f.c.goal_mm==800 && f.c.common<5 && f.c.height_alarm==0);
    previous=f.i.s.height_mm;
    for(n=0;n<1700U && f.c.phase==J_ADJUST;++n) {
        if(f.c.near_active) {
            /* Independent fixture motion, not a thrust/flight simulation. */
            physical+=0.6f;
            if(physical>=J_NEAR_EXIT_MM) {
                f.i.s.height_mm=physical;f.i.s.vz_mm_s=200;
                f.i.s.range_ok=f.i.s.velocity_ok=f.i.s.flow_ok=1;
            }
        } else {
            f.i.s.height_mm=f.c.height_target_mm;
            f.i.s.vz_mm_s=(f.i.s.height_mm-previous)/0.003f;
        }
        previous=f.i.s.height_mm; tick(&f);
        CHECK(f.c.height_alarm==0);
    }
    CHECK(f.c.armed && f.c.mode==J_C3_AUTO && f.c.phase==J_HOVER && f.c.enabled);
    CHECK(J_Abs(f.c.height_target_mm-800)<1 && n>1000);
}
static void c3_repeat_and_other_mode(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); f.c.completed=1; f.c.adjust_ms=99;
    f.i.s.nav_x_mm=123; event(&f,J_C3);
    CHECK(f.c.completed && f.c.adjust_ms==99 && f.i.s.nav_x_mm==123);
    event(&f,J_C1); CHECK(f.c.mode==J_C1_HOLD);
    event(&f,J_C3); CHECK(f.c.mode==J_C3_AUTO && !f.c.completed && f.c.phase==J_ADJUST);
    CHECK(f.i.s.nav_x_mm==123 && f.c.baseline_lux==100);
}
static void c1_capture_and_center_gate(void)
{
    Fixture f; init(&f,J_C3_AUTO,1200); f.i.channels[2]=1819;
    event(&f,J_C1); CHECK(f.c.mode==J_C1_HOLD && f.c.goal_mm==1200);
    advance(&f,600); CHECK(f.c.goal_mm==1200 && !f.c.c1_centered);
    f.i.channels[2]=1500; tick(&f); CHECK(f.c.c1_centered);
    f.i.channels[2]=1819; advance(&f,300); CHECK(f.c.goal_mm>1270);
    event(&f,J_C1); CHECK(f.c.mode==J_ALIGN);
}
static void c1_ignored_during_climb(void)
{ Fixture f; init(&f,J_C3_AUTO,300); f.c.phase=J_ADJUST; f.c.goal_mm=800; f.c.adjust_ms=f.i.now_ms;
  event(&f,J_C1); CHECK(f.c.mode==J_C3_AUTO && f.c.phase==J_ADJUST); }
static void align_low_high_and_no_timeout(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); event(&f,J_C4);
    CHECK(f.c.mode==J_ALIGN && f.c.align==J_ALIGN_UP);
    advance(&f,12000); CHECK(f.c.mode==J_ALIGN && f.c.goal_mm==800 && f.c.armed);
    f.i.channels[2]=1819; tick(&f); CHECK(f.c.align==J_ALIGN_DOWN);
    f.i.channels[2]=1667; tick(&f); CHECK(f.c.align==J_ALIGN_MATCH);
    advance(&f,198); CHECK(f.c.mode==J_ALIGN);
    tick(&f); CHECK(f.c.mode==J_MANUAL && f.c.common==J_ManualThrust(1667) && f.c.takeover_event==1);
    CHECK(!f.c.low_active);
}
static void align_center_and_cancel(void)
{
    Fixture f; init(&f,J_C1_HOLD,800); event(&f,J_C4);
    f.i.channels[2]=1667; advance(&f,150); f.i.channels[0]=1800; tick(&f);
    CHECK(!f.c.match_active && f.c.mode==J_ALIGN && f.c.x_target_px==0);
    f.i.channels[0]=1500; advance(&f,150); CHECK(f.c.mode==J_ALIGN);
    f.i.channels[2]=1819; event(&f,J_C4); CHECK(f.c.mode==J_C1_HOLD && !f.c.c1_centered);
    f.i.channels[2]=1000; advance(&f,1500); CHECK(f.c.goal_mm==800 && f.c.motors_on);
}
static void align_unreachable(void)
{
    Fixture f; init(&f,J_ALIGN,800); f.c.common=700; f.c.vz_integral=200;
    f.c.resume_mode=J_C3_AUTO; f.i.channels[2]=1819; advance(&f,11000);
    CHECK(f.c.mode==J_ALIGN && f.c.align==J_ALIGN_UNREACHABLE && f.c.armed);
    CHECK(J_ManualThrust(1819)==568 && J_ManualThrust(2000)==650);
}
static void c2_priority_and_original_combo(void)
{
    Fixture f; init(&f,J_LIGHT_HOLD,800); event(&f,J_C1|J_C2|J_C3|J_C4);
    CHECK(!f.c.armed && !f.c.motors_on && f.c.common==0 && f.c.fault==J_ERR_ESTOP);
    init(&f,J_ALIGN,800); f.i.channels[2]=1000; f.i.channels[3]=1100; tick(&f);
    CHECK(!f.c.armed && f.c.fault==J_ERR_ESTOP);
}
static void tilt_and_imu_and_link(void)
{
    Fixture f; init(&f,J_C3_AUTO,30); f.c.phase=J_ADJUST; f.i.s.roll_deg=51; tick(&f);
    CHECK(!f.c.armed && f.c.fault==J_ERR_TILT);
    init(&f,J_LIGHT_HOLD,800); f.i.s.imu_ok=0; tick(&f); CHECK(!f.c.armed && f.c.fault==J_ERR_IMU);
    init(&f,J_ALIGN,800); f.auto_rc=0; advance(&f,501); CHECK(!f.c.armed && f.c.fault==J_ERR_RC);
    init(&f,J_C3_AUTO,800); f.i.rc_gap=1; tick(&f); CHECK(!f.c.armed && f.c.fault==J_ERR_RC);
}
static void baseline_freeze_and_inhibit(void)
{
    Fixture f; unsigned k; init(&f,J_LOCKED,30); f.auto_light=0; f.c.ambient_count=0;
    for(k=0;k<5;++k) light(&f,100+(float)k*10);
    CHECK(f.c.ambient_count==5 && f.c.baseline_lux==120);
    f.i.channels[2]=1000; tick(&f); f.i.channels[2]=1819; advance(&f,90); f.i.channels[2]=1000; tick(&f);
    CHECK(f.c.armed); f.c.enabled=0;
    light(&f,900); light(&f,900); light(&f,900);
    CHECK(f.c.baseline_lux==120 && !f.c.light_event && f.c.mode==J_MANUAL);
}
static void independent_samples_and_hysteresis(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); f.auto_light=0;
    light(&f,400); advance(&f,150); CHECK(f.c.light_count==1 && !f.c.light_event);
    light(&f,400); CHECK(f.c.light_count==2); light(&f,400);
    CHECK(f.c.mode==J_LIGHT_HOLD && f.c.light_event==1);
    event(&f,J_C1|J_C3|J_C4); CHECK(f.c.mode==J_LIGHT_HOLD);
    f.auto_light=1; advance(&f,3500); CHECK(f.c.completed && f.c.mode==J_C3_AUTO);
    advance(&f,900); CHECK(f.c.light_event==1);
    f.auto_light=0; light(&f,299); light(&f,299); light(&f,299);
    CHECK(!f.c.light_inside && !f.c.arrival);
    detect(&f); CHECK(f.c.light_event==2 && f.c.mode==J_LIGHT_HOLD);
}
static void independent_alert_and_stable_timers(void)
{
    Fixture f; uint32_t start; init(&f,J_C3_AUTO,800); detect(&f); start=f.c.alert_ms;
    advance(&f,3000); CHECK(!JCore_AlertActive(&f.c,f.i.now_ms) && !f.c.completed);
    CHECK((uint32_t)(f.i.now_ms-start)>=3000U && f.c.mode==J_LIGHT_HOLD);
    advance(&f,400); CHECK(f.c.completed && f.c.mode==J_C3_AUTO && f.c.arrival);
    event(&f,J_C3); CHECK(f.c.completed); /* no automatic new search or new light alarm */
}
static void stable_loss_and_ten_second_release(void)
{
    Fixture f; uint32_t start; init(&f,J_C3_AUTO,800); detect(&f); start=f.c.light_task_ms;
    advance(&f,2800); CHECK(f.c.hold_active);
    f.i.s.vz_mm_s=100; tick(&f); CHECK(!f.c.hold_active);
    f.i.s.velocity_ok=0; f.i.s.vz_mm_s=0; advance(&f,7500);
    CHECK(f.c.mode==J_C3_AUTO && f.c.task_failed && !f.c.completed);
    CHECK((uint32_t)(f.i.now_ms-start)>=J_LIGHT_TIMEOUT_MS);
}
static void manual_light_intervention_and_enable_persists(void)
{
    Fixture f; init(&f,J_C3_AUTO,1200); event(&f,J_C4);
    f.i.channels[2]=1667; advance(&f,210); CHECK(f.c.mode==J_MANUAL && f.c.enabled && f.c.height_alarm==1);
    detect(&f); CHECK(f.c.mode==J_LIGHT_HOLD && f.c.goal_mm==800 && f.c.height_target_mm>1190);
    CHECK(f.c.height_target_mm<1200); /* bounded goal slew, never instant height target jump */
}
static void flow_fault_and_takeover(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); f.i.s.flow_ok=0; advance(&f,360);
    CHECK(f.c.mode==J_FLOW_HOLD && f.c.armed && f.c.fault==J_ERR_FLOW);
    event(&f,J_C4); CHECK(f.c.mode==J_ALIGN);
    f.i.channels[2]=1667; advance(&f,210); CHECK(f.c.mode==J_MANUAL && f.c.armed);
}
static void laser_fault_wait_and_stop(void)
{
    Fixture f; uint32_t entered; init(&f,J_C3_AUTO,800); f.i.s.range_ok=0; advance(&f,310);
    CHECK(f.c.mode==J_LASER_WAIT && f.c.armed && f.c.fault==J_ERR_RANGE); entered=f.c.fault_ms;
    f.i.s.range_ok=1; advance(&f,9000); CHECK(f.c.mode==J_LASER_WAIT);
    while((uint32_t)(f.i.now_ms-entered)<J_LASER_WAIT_MS) tick(&f);
    CHECK(!f.c.armed && f.c.common==0 && f.c.fault==J_ERR_RANGE);
}
static void laser_fault_successful_handover(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); f.i.s.range_ok=0; advance(&f,310);
    f.i.channels[2]=1667; advance(&f,210);
    CHECK(f.c.mode==J_MANUAL && f.c.takeover_event==1 && f.c.armed);
    advance(&f,11000); CHECK(f.c.armed); /* explicit takeover ends the automatic fault timer */
}
static void takeoff_timeout_ground_and_air(void)
{
    Fixture f; init(&f,J_MANUAL,30); f.c.airborne=0; f.c.enabled=0; event(&f,J_C3); advance(&f,10020);
    CHECK(!f.c.armed && f.c.fault==J_ERR_TAKEOFF_GROUND);
    init(&f,J_C1_HOLD,400); event(&f,J_C3); advance(&f,10020);
    CHECK(f.c.armed && f.c.phase==J_TASK_FAILED && f.c.fault==J_ERR_TAKEOFF_AIR && f.c.mode==J_C3_AUTO);
    event(&f,J_C1); CHECK(f.c.mode==J_C1_HOLD);
}
static void led_rules_and_boundaries(void)
{
    Fixture f; init(&f,J_LOCKED,30);
    CHECK(JCore_LEDMask(&f.c,1000)==1 && JCore_LEDMask(&f.c,1200)==2 && JCore_LEDMask(&f.c,1400)==4 && JCore_LEDMask(&f.c,1600)==8);
    f.c.armed=1; f.c.airborne=0; CHECK(JCore_LEDMask(&f.c,1000)==3 && JCore_LEDMask(&f.c,1500)==12);
    f.c.phase=J_ADJUST; CHECK(JCore_LEDMask(&f.c,500)==0);
    init(&f,J_C3_AUTO,600); tick(&f); CHECK(f.c.height_alarm==0);
    f.i.s.height_mm=1000; tick(&f); CHECK(f.c.height_alarm==0);
    f.i.s.height_mm=599; tick(&f); CHECK(f.c.height_alarm==-1 && JCore_LEDMask(&f.c,f.i.now_ms)==12);
    CHECK(JCore_LEDMask(&f.c,f.i.now_ms+499)==12&&JCore_LEDMask(&f.c,f.i.now_ms+500)==0);
    f.i.s.height_mm=1001; tick(&f); CHECK(f.c.height_alarm==1 && JCore_LEDMask(&f.c,f.i.now_ms)==3);
    f.c.arrival=f.c.light_inside=1; CHECK(JCore_LEDMask(&f.c,f.i.now_ms)==3);
    f.c.alert_active=1; f.c.alert_ms=2000; f.c.light_event=1; CHECK(JCore_LEDMask(&f.c,2000)==15);
    CHECK(JCore_LEDMask(&f.c,2100)==0);
}
static void horizontal_axes_no_residual_command(void)
{
    Fixture f; init(&f,J_C3_AUTO,800); f.i.channels[0]=1800; advance(&f,60);
    CHECK(f.c.flow_moving_x && f.c.flow_roll<0);
    f.i.channels[0]=1500; f.i.channels[1]=1800; advance(&f,60);
    CHECK(!f.c.flow_moving_x && f.c.flow_moving_y && f.c.prev_vx_error==0);
    CHECK(f.c.prev_vy_error>0);
}
static void repeated_schedule_and_wrap(void)
{
    Fixture f; uint32_t stamp; init_at(&f,J_C3_AUTO,800,0xFFFFFF00U); detect(&f);
    stamp=f.c.stable_ms; JCore_Step(&f.c,&f.i); JCore_Step(&f.c,&f.i);
    CHECK(f.c.stable_ms==stamp && !f.c.completed);
    advance(&f,3400); CHECK(f.c.completed && f.c.mode==J_C3_AUTO);
    event(&f,J_C2); f.i.channels[2]=1000; tick(&f); f.i.channels[2]=1819; advance(&f,90); f.i.channels[2]=1000; tick(&f);
    CHECK(f.c.armed && !JCore_AlertActive(&f.c,f.i.now_ms));
}
static void range_units_velocity_and_invalidity(void)
{
    JSensorFilter f; JSensors_Init(&f);
    JSensors_Module(&f,800,0,0,80,1,1000); CHECK(f.value.height_mm==800 && !f.value.velocity_ok);
    JSensors_Module(&f,810,1,0,80,1,1030); CHECK(f.value.velocity_ok && J_Abs(f.value.vz_mm_s-333.3333f)<0.1f);
    JSensors_ArmReference(&f); CHECK(f.value.nav_reference_ok);
    JSensors_Module(&f,0,0,0,0,1,1036); CHECK(!f.value.range_ok && !f.value.velocity_ok && f.value.height_mm==810);
    JSensors_Module(&f,810,0,0,0,1,1042); CHECK(f.value.x_px!=0 && !f.value.flow_ok);
    JSensors_Module(&f,810,0,0,80,1,1400); CHECK(!f.value.nav_reference_ok);
    JSensors_Module(&f,810,0,0,80,1,1600); CHECK(!f.value.velocity_ok);
}
static void physical_80mm_is_not_c3_800mm(void)
{
    Fixture f;JSensorFilter sensors;init(&f,J_MANUAL,80);f.c.enabled=0;
    JSensors_Init(&sensors);sensors.value=f.i.s;
    JSensors_Module(&sensors,80,0,0,80,1,1000);
    JSensors_Module(&sensors,80,0,0,80,1,1030);
    f.i.s=sensors.value;f.i.now_ms=1030;f.c.last_tick_ms=1030;
    event(&f,J_C3);advance(&f,600);
    CHECK(f.c.mode==J_C3_AUTO && f.c.phase==J_ADJUST && !f.c.enabled);
    CHECK(f.c.goal_mm==800 && f.c.height_target_mm>80 && f.c.height_target_mm<300);
}
static void alignment_requires_recent_control_data(void)
{
    Fixture f;init(&f,J_ALIGN,800);f.c.resume_mode=J_C3_AUTO;f.i.channels[2]=1667;
    f.auto_rc=0;advance(&f,210);CHECK(f.c.armed&&f.c.mode==J_ALIGN&&!f.c.match_active);
    f.auto_rc=1;advance(&f,210);CHECK(f.c.mode==J_MANUAL);
}
static void low_airborne_timeout_and_near_ground_flow_zone(void)
{
    Fixture f;init(&f,J_MANUAL,30);f.c.ground_valid=1;f.c.ground_mm=30;f.c.airborne=0;f.c.enabled=0;
    f.i.s.flow_ok=0;f.i.s.quality=0;event(&f,J_C3);
    CHECK(f.c.mode==J_C3_AUTO);f.i.s.height_mm=150;advance(&f,10020);
    CHECK(f.c.armed&&f.c.airborne&&f.c.fault==J_ERR_TAKEOFF_AIR&&f.c.phase==J_TASK_FAILED);
}
static void height_equilibrium_has_no_collective_drift(void)
{ Fixture f;init(&f,J_C3_AUTO,800);advance(&f,3000);CHECK(f.c.common==500&&f.c.vz_integral==0); }
static void ground_c1_preserved_and_zero_output_handover(void)
{
    Fixture f;init(&f,J_MANUAL,30);f.c.motors_on=0;f.c.common=0;f.c.airborne=0;f.c.enabled=0;
    f.i.s.flow_ok=0;f.i.s.quality=0;event(&f,J_C1);
    CHECK(f.c.mode==J_C1_HOLD&&!f.c.motors_on&&f.c.common==0&&f.c.goal_mm==30);
    event(&f,J_C1);CHECK(f.c.mode==J_ALIGN&&f.c.align==J_ALIGN_MATCH);
    advance(&f,210);CHECK(f.c.mode==J_MANUAL&&!f.c.motors_on&&f.c.common==0);
    event(&f,J_C1);f.i.channels[2]=1819;tick(&f);
    CHECK(f.c.mode==J_C1_HOLD&&f.c.motors_on&&f.c.common<5&&f.c.goal_mm==30);
    f.i.channels[2]=1500;tick(&f);f.i.channels[2]=1819;advance(&f,300);CHECK(f.c.goal_mm>100);
}
int main(void)
{
    RUN(blind_ground_c3_starts_bounded_ramp);
    RUN(near_entry_guards_and_one_shot_grant);RUN(near_absolute_deadline_and_no_restart);
    RUN(near_all_safety_paths_stop);RUN(near_ignores_stale_and_low_height);
    RUN(near_handoff_continuity_and_range_loss);RUN(near_requires_post_start_changing_timestamps);
    RUN(near_c4_holds_collective_and_matches_without_height);RUN(near_c4_range_recovery_uses_measured_target);
    RUN(near_wrap_and_repeated_scheduler);
    RUN(physical_80mm_is_not_c3_800mm);
    RUN(locked_and_manual_unlock); RUN(c3_feedback_takeoff); RUN(c3_repeat_and_other_mode);
    RUN(c1_capture_and_center_gate); RUN(c1_ignored_during_climb); RUN(align_low_high_and_no_timeout);
    RUN(align_center_and_cancel); RUN(align_unreachable); RUN(c2_priority_and_original_combo);
    RUN(tilt_and_imu_and_link); RUN(baseline_freeze_and_inhibit); RUN(independent_samples_and_hysteresis);
    RUN(independent_alert_and_stable_timers); RUN(stable_loss_and_ten_second_release);
    RUN(manual_light_intervention_and_enable_persists); RUN(flow_fault_and_takeover);
    RUN(laser_fault_wait_and_stop); RUN(laser_fault_successful_handover); RUN(takeoff_timeout_ground_and_air);
    RUN(led_rules_and_boundaries); RUN(horizontal_axes_no_residual_command); RUN(repeated_schedule_and_wrap);
    RUN(range_units_velocity_and_invalidity);
    RUN(alignment_requires_recent_control_data);RUN(low_airborne_timeout_and_near_ground_flow_zone);RUN(height_equilibrium_has_no_collective_drift);
    RUN(ground_c1_preserved_and_zero_output_handover);
    printf("RESULT cases=%u checks=%u\n",cases,checks); return 0;
}
