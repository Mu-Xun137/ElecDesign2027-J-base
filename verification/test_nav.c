#include "j_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks,cases;
static const char *current;
#define CHECK(x) do { ++checks; if(!(x)) { printf("FAIL %s:%d: %s\n",current,__LINE__,#x); exit(2); } } while(0)
#define RUN(f) do { current=#f; f(); ++cases; printf("PASS %s\n",current); } while(0)
static float gaussian(float x,float y,float px,float py,float sigma,float amplitude)
{ float dx=x-px,dy=y-py;return 100.0f+amplitude*(float)exp(-(double)(dx*dx+dy*dy)/(2.0*sigma*sigma)); }
typedef struct { JNav n;JNavInput i;uint32_t sample_ms,hold_ms;uint8_t holding;unsigned arrivals,steps;int scene;uint32_t switch_delay; } Simulation;
static float scene(int mode,uint8_t stage,uint32_t elapsed,float x,float y,uint32_t light_id)
{
    if(mode==1)return 100; /* not found */
    if(mode==2)return 700; /* flat field */
    if(mode==3)return gaussian(x,y,0,0,250,800); /* A peripheral light, never a second peak */
    if(mode==7)return 600.0f+0.30f*x; /* monotone gradient, not an arrival */
    if(mode==8)return light_id%3U==0U?100.0f:800.0f; /* intermittent spike, lacks 3-sample support */
    if(stage==1)return gaussian(x,y,0,0,90,800);
    if(mode==4)return gaussian(x,y,0,0,90,800); /* A has not switched */
    if(mode==5)return 100; /* B never lights */
    if(mode==6)return gaussian(x,y,80,0,90,800); /* B inside configured A exclusion area */
    if(mode==9 && elapsed<500U)return 100; /* observable dark gap, permitted but not required */
    return gaussian(x,y,-350,350,90,800);
}
static void sim_init_at(Simulation *s,int mode,uint32_t now)
{
    memset(s,0,sizeof(*s));s->scene=mode;s->i.now_ms=now;s->sample_ms=now;
    s->i.coordinate_ok=s->i.sample_ok=s->i.stable=1;s->i.baseline=100;s->i.lux=100;s->i.light_id=1;
    CHECK(JNav_Start(&s->n,&s->i));
}
static void sim_init(Simulation *s,int mode) { sim_init_at(s,mode,1000U); }
static void sim_step(Simulation *s)
{
    float dx=s->n.target_x_mm-s->i.x_mm,dy=s->n.target_y_mm-s->i.y_mm;
    float d=(float)sqrt(dx*dx+dy*dy),step=J_NAV_SPEED_MM_S*0.020f;
    if(d>step){s->i.x_mm+=dx*step/d;s->i.y_mm+=dy*step/d;s->i.stable=0;}
    else{s->i.x_mm=s->n.target_x_mm;s->i.y_mm=s->n.target_y_mm;s->i.stable=1;}
    s->i.now_ms+=20U;++s->steps;
    if((uint32_t)(s->i.now_ms-s->sample_ms)>=200U) {
        s->sample_ms=s->i.now_ms;++s->i.light_id;
        if(s->n.stage==2U && (uint32_t)(s->i.now_ms-s->n.stage_ms)<s->switch_delay)
            s->i.lux=gaussian(s->i.x_mm,s->i.y_mm,0,0,90,800);
        else s->i.lux=scene(s->scene,s->n.stage,(uint32_t)(s->i.now_ms-s->n.stage_ms),s->i.x_mm,s->i.y_mm,s->i.light_id);
    }
    JNav_Step(&s->n,&s->i);
    CHECK(J_Abs(s->i.x_mm)<J_NAV_HALF_SIZE_MM&&J_Abs(s->i.y_mm)<J_NAV_HALF_SIZE_MM);
    if(s->n.phase==J_NAV_FORCE) {
        if(!s->holding){s->holding=1;s->hold_ms=s->i.now_ms;++s->arrivals;}
        if((uint32_t)(s->i.now_ms-s->hold_ms)>=3300U) { JNav_HoverFinished(&s->n,1,&s->i);s->holding=0; }
    }
}
static void sim_finish(Simulation *s)
{ while(s->steps<7000U && s->n.phase!=J_NAV_DONE && s->n.phase!=J_NAV_FAILED)sim_step(s);CHECK(s->steps<7000U); }
#if J_NAV_CALIBRATED
static void bounded_active_sampling_single_or_ab(void)
{
    Simulation s;sim_init(&s,0);s.i.light_inside=1;sim_finish(&s);
    CHECK(s.n.phase==J_NAV_DONE&&s.arrivals==(J_NAV_MODE==2?2U:1U));
    CHECK(s.steps>1000U); /* actual travel/sample/dwell time, not a threshold alias */
    if(J_NAV_MODE==2)CHECK(s.n.have_a&&s.n.stage==2&&J_Abs(s.n.a_x)<30&&J_Abs(s.n.a_y)<30);
}
static void no_source_and_flat_and_monotone_fields(void)
{
    Simulation s;int kind;
    for(kind=1;kind<=2;++kind){sim_init(&s,kind);sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals);}
    sim_init(&s,7);sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals);
    sim_init(&s,8);sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals);
}
static void coordinate_boundary_and_missing_evidence(void)
{
    Simulation s;sim_init(&s,0);s.i.coordinate_ok=0;sim_step(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals);
    sim_init(&s,0);s.i.x_mm=J_NAV_HALF_SIZE_MM+1;JNav_Step(&s.n,&s.i);CHECK(s.n.phase==J_NAV_FAILED);
    sim_init(&s,0);s.i.sample_ok=0;sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals);
}
static void point_dwell_and_new_sample_ids(void)
{
    Simulation s;unsigned k;sim_init(&s,0);s.i.x_mm=s.n.target_x_mm;s.i.y_mm=s.n.target_y_mm;s.i.lux=900;
    JNav_Step(&s.n,&s.i);s.i.now_ms+=300;
    for(k=0;k<100;++k)JNav_Step(&s.n,&s.i);CHECK(s.n.samples_at_point==0&&s.n.phase==J_NAV_SCAN);
    ++s.i.light_id;s.i.now_ms+=200;JNav_Step(&s.n,&s.i);CHECK(s.n.samples_at_point==1);
    for(k=0;k<100;++k)JNav_Step(&s.n,&s.i);CHECK(s.n.samples_at_point==1);
    s.i.stable=0;JNav_Step(&s.n,&s.i);CHECK(!s.n.samples_at_point&&!s.n.waypoint_active);
}
static void budget_includes_forced_hold_and_wrap(void)
{
    Simulation s;sim_init_at(&s,0,0xFFFFFF00U);s.n.phase=J_NAV_FORCE;
    s.i.now_ms=s.n.stage_ms+J_NAV_BUDGET_MS-1;JNav_Step(&s.n,&s.i);CHECK(s.n.phase==J_NAV_FORCE);
    ++s.i.now_ms;JNav_Step(&s.n,&s.i);CHECK(s.n.phase==J_NAV_FAILED);
    sim_init_at(&s,0,0xFFFF8000U);sim_finish(&s);CHECK(s.n.phase==J_NAV_DONE);
}
static void start_reference_is_not_moved_to_current_air_position(void)
{
    JNav n;JNavInput i={0};i.now_ms=1000;i.coordinate_ok=1;i.x_mm=200;i.y_mm=100;
    CHECK(JNav_Start(&n,&i));CHECK(n.target_x_mm==-J_NAV_GRID_STEP_MM&&n.target_y_mm==-J_NAV_GRID_STEP_MM);
    CHECK(i.x_mm==200&&i.y_mm==100);
    i.coordinate_ok=0;CHECK(!JNav_Start(&n,&i));
}
static void force_requires_cross_samples_and_recheck(void)
{
    Simulation s;sim_init(&s,0);
    while(s.steps<4000U&&s.n.phase!=J_NAV_FORCE&&s.n.phase!=J_NAV_FAILED)sim_step(&s);
    CHECK(s.n.phase==J_NAV_FORCE&&s.n.peak_valid);
    CHECK(s.n.samples[0]>=350);
    for(unsigned k=1;k<=4U;++k) {
        float dx=s.n.sample_x[k]-s.n.sample_x[0],dy=s.n.sample_y[k]-s.n.sample_y[0];
        CHECK(dx*dx+dy*dy>=3600&&s.n.samples[0]>=s.n.samples[k]+J_NAV_PEAK_PROMINENCE_LUX);
    }
}
#if J_NAV_MODE==2
static void ab_observable_dark_gap_and_delayed_switch(void)
{
    Simulation s;sim_init(&s,9);sim_finish(&s);CHECK(s.n.phase==J_NAV_DONE&&s.arrivals==2);
    sim_init(&s,0);s.switch_delay=15000;sim_finish(&s);CHECK(s.n.phase==J_NAV_DONE&&s.arrivals==2);
}
static void ab_a_still_on_b_off_or_too_close_not_complete(void)
{
    Simulation s;unsigned kind;
    for(kind=4;kind<=6;++kind){sim_init(&s,(int)kind);sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&s.arrivals==1&&s.n.have_a);}
}
static void ab_a_peripheral_brightness_not_another_peak(void)
{
    Simulation s;sim_init(&s,3);
    s.n.phase=J_NAV_FORCE;s.i.x_mm=s.i.y_mm=0;s.i.lux=900;
    JNav_HoverFinished(&s.n,1,&s.i);CHECK(s.n.phase==J_NAV_B_STAGE&&s.n.have_a);
    sim_finish(&s);CHECK(s.n.phase==J_NAV_FAILED&&!s.arrivals&&s.n.stage==2);
}
static void ab_second_budget_has_no_extra_dark_wait(void)
{
    Simulation s;uint32_t start;sim_init(&s,0);s.n.phase=J_NAV_FORCE;
    JNav_HoverFinished(&s.n,1,&s.i);start=s.i.now_ms;
    CHECK(s.n.phase==J_NAV_B_STAGE&&s.n.stage_ms==start);
    s.i.light_inside=1;JNav_Step(&s.n,&s.i);CHECK(s.n.phase==J_NAV_SCAN);
    s.i.now_ms=start+J_NAV_BUDGET_MS;JNav_Step(&s.n,&s.i);CHECK(s.n.phase==J_NAV_FAILED);
}
#endif
typedef struct { JCore c;JInput i;uint32_t light_ms;unsigned steps; } CoreSim;
static void core_init(CoreSim *f)
{
    unsigned k;memset(f,0,sizeof(*f));JCore_Init(&f->c,1000);f->i.now_ms=f->light_ms=1000;
    for(k=0;k<8;++k)f->i.channels[k]=k==0||k==1||k==3?1500:1000;
    f->i.have_rc=f->i.new_rc=1;f->i.s.height_mm=800;f->i.s.quality=80;
    f->i.s.range_ok=f->i.s.flow_ok=f->i.s.imu_ok=f->i.s.light_ok=f->i.s.velocity_ok=f->i.s.module_ok=f->i.s.nav_reference_ok=1;
    f->i.s.light_id=1;f->i.s.lux=100;f->i.s.light_ms=1000;
    f->c.armed=f->c.motors_on=f->c.airborne=1;f->c.mode=J_C1_HOLD;f->c.phase=J_HOVER;
    f->c.common=500;f->c.height_target_mm=f->c.goal_mm=800;
    f->c.ambient_count=5;f->c.baseline_lux=100;f->c.last_light_id=1;
    f->i.events=J_C3;
}
static void core_step(CoreSim *f)
{
    float vx=(f->c.x_target_px-f->i.s.x_px)*J_FLOW_POS_P;
    float vy=(f->c.y_target_px-f->i.s.y_px)*J_FLOW_POS_P;
    float speed=(float)sqrt(vx*vx+vy*vy);
    if(speed>J_NAV_SPEED_MM_S){vx*=J_NAV_SPEED_MM_S/speed;vy*=J_NAV_SPEED_MM_S/speed;}
    /* Ideal coordinate/velocity plant fixture. No aerodynamics or flight accuracy claim. */
    f->i.s.nav_x_mm+=vx*0.003f;f->i.s.nav_y_mm+=vy*0.003f;
    f->i.s.x_px=f->i.s.nav_x_mm;f->i.s.y_px=f->i.s.nav_y_mm;
    f->i.s.vx_px_s=vx;f->i.s.vy_px_s=vy;
    f->i.now_ms+=3;f->i.rc_ms=f->i.now_ms;f->i.new_rc=1;
    f->i.s.imu_ms=f->i.s.range_ms=f->i.s.flow_ms=f->i.now_ms;
    if((uint32_t)(f->i.now_ms-f->light_ms)>=200U) {
        f->light_ms=f->i.s.light_ms=f->i.now_ms;++f->i.s.light_id;
        f->i.s.lux=scene(0,f->c.nav.have_a?2:1,0,f->i.s.nav_x_mm,f->i.s.nav_y_mm,f->i.s.light_id);
    }
    JCore_Step(&f->c,&f->i);f->i.events=0;++f->steps;
}
static void real_core_navigation_and_stable_holds(void)
{
    CoreSim f;unsigned force_entries=0;uint8_t was_force=0;
    core_init(&f);
    while(f.steps<40000U&&!f.c.nav_completed&&!f.c.task_failed) {
        core_step(&f);
        if(f.c.mode==J_LIGHT_HOLD&&!was_force){++force_entries;CHECK(f.c.light_task_ms==f.i.now_ms);}
        was_force=f.c.mode==J_LIGHT_HOLD;
        CHECK(f.c.armed&&f.c.goal_mm==800);
    }
    CHECK(f.c.nav_completed&&f.c.completed&&f.c.mode==J_C3_AUTO);
    CHECK(force_entries==(J_NAV_MODE==2?2U:1U)&&f.c.light_event==force_entries);
    for(unsigned k=0;k<100;++k){f.i.events=J_C3;core_step(&f);}
    CHECK(f.c.nav_completed&&!f.c.nav_running&&f.c.nav.phase==J_NAV_DONE);
}
static void c4_cancellation_never_internally_restarts_search(void)
{
    CoreSim f;core_init(&f);while(f.steps<500U&&!f.c.nav_running)core_step(&f);
    CHECK(f.c.nav_running);f.i.events=J_C4;core_step(&f);CHECK(f.c.mode==J_ALIGN&&!f.c.nav_running&&!f.c.nav_pending);
    f.i.s.height_mm=1000;f.i.events=J_C4;core_step(&f);CHECK(f.c.mode==J_C3_AUTO&&f.c.phase==J_ADJUST);
    for(unsigned k=0;k<3000;++k){f.i.s.height_mm=f.c.height_target_mm;core_step(&f);}
    CHECK(!f.c.nav_running&&!f.c.nav_pending&&!f.c.nav_completed);
}
static void c1_and_c2_abort_navigation(void)
{
    CoreSim f;core_init(&f);while(f.steps<500U&&!f.c.nav_running)core_step(&f);
    f.i.events=J_C1;core_step(&f);CHECK(f.c.mode==J_C1_HOLD&&!f.c.nav_running);
    f.i.events=J_C3;core_step(&f);while(f.steps<1000U&&!f.c.nav_running)core_step(&f);
    CHECK(f.c.nav_running);f.i.events=J_C2|J_C3;core_step(&f);CHECK(!f.c.armed&&!f.c.nav_running&&!f.c.nav_completed);
}
#else
static void uncalibrated_navigation_is_rejected(void)
{ JNav n;JNavInput i={0};i.coordinate_ok=1;CHECK(!JNav_Start(&n,&i));CHECK(n.phase==J_NAV_FAILED); }
#endif
int main(void)
{
#if J_NAV_CALIBRATED
    RUN(bounded_active_sampling_single_or_ab);RUN(no_source_and_flat_and_monotone_fields);
    RUN(coordinate_boundary_and_missing_evidence);RUN(point_dwell_and_new_sample_ids);
    RUN(budget_includes_forced_hold_and_wrap);RUN(start_reference_is_not_moved_to_current_air_position);
    RUN(force_requires_cross_samples_and_recheck);
#if J_NAV_MODE==2
    RUN(ab_observable_dark_gap_and_delayed_switch);RUN(ab_a_still_on_b_off_or_too_close_not_complete);
    RUN(ab_a_peripheral_brightness_not_another_peak);RUN(ab_second_budget_has_no_extra_dark_wait);
#endif
    RUN(real_core_navigation_and_stable_holds);RUN(c4_cancellation_never_internally_restarts_search);RUN(c1_and_c2_abort_navigation);
#else
    RUN(uncalibrated_navigation_is_rejected);
#endif
    printf("RESULT cases=%u checks=%u\n",cases,checks);return 0;
}
