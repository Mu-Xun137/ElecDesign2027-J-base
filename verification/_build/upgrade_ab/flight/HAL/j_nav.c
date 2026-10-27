/* Experimental active sampling navigation. Same interfaces/storage as the base.
 * Local optical peaks are arrival proxies, NOT absolute position or source identity. */
#include "j_nav.h"
#include "j_config.h"
#include <string.h>
enum { NAV_BAD_REFERENCE=1,NAV_BOUNDARY,NAV_TIMEOUT,NAV_NO_PEAK,NAV_UNCALIBRATED };
static float absolute(float x) { return x<0.0f?-x:x; }
static uint8_t finite_value(float x) { return x==x && x>-1.0e8f && x<1.0e8f; }
static float distance2(float x,float y,float a,float b) { x-=a;y-=b;return x*x+y*y; }
static void fail(JNav *n,uint8_t why) { n->phase=J_NAV_FAILED;n->failure=why;n->force_requested=0; }
void JNav_Init(JNav *n) { memset(n,0,sizeof(*n)); }
static uint8_t outside_a(const JNav *n,float x,float y)
{ return !n->have_a || distance2(x,y,n->a_x,n->a_y)>J_NAV_A_EXCLUDE_MM*J_NAV_A_EXCLUDE_MM; }
static void reset_point(JNav *n)
{ n->waypoint_active=n->settled=n->samples_at_point=0;n->point_sum=0.0f;n->point_min=1.0e8f; }
static uint8_t target(JNav *n,float x,float y)
{
    if(absolute(x)>=J_NAV_HALF_SIZE_MM || absolute(y)>=J_NAV_HALF_SIZE_MM) { fail(n,NAV_BOUNDARY);return 0; }
    n->target_x_mm=x;n->target_y_mm=y;reset_point(n);return 1;
}
static void grid(JNav *n)
{
    uint8_t row=(uint8_t)(n->index/3U),column=(uint8_t)(n->index%3U);
    float x,y;
    if(row&1U)column=(uint8_t)(2U-column);
    x=((float)column-1.0f)*J_NAV_GRID_STEP_MM;
    y=((float)row-1.0f)*J_NAV_GRID_STEP_MM;
    target(n,x,y);
}
uint8_t JNav_Start(JNav *n,const JNavInput *i)
{
    JNav_Init(n);
#if !J_NAV_CALIBRATED
    (void)i;fail(n,NAV_UNCALIBRATED);return 0;
#else
    if(!i->coordinate_ok || !finite_value(i->x_mm) || !finite_value(i->y_mm)) { fail(n,NAV_BAD_REFERENCE);return 0; }
    if(absolute(i->x_mm)>=J_NAV_HALF_SIZE_MM || absolute(i->y_mm)>=J_NAV_HALF_SIZE_MM) { fail(n,NAV_BOUNDARY);return 0; }
    n->phase=J_NAV_SCAN;n->stage=1;n->stage_ms=i->now_ms;grid(n);
    return n->phase!=J_NAV_FAILED;
#endif
}
/* Arrival -> 300ms continuous stable dwell -> three NEW one-shot samples.
 * An old sample, a repeated scheduler call or movement never advances this counter. */
static uint8_t point(JNav *n,const JNavInput *i,float *mean,float *minimum)
{
    float tolerance=J_NAV_WAYPOINT_MM*J_NAV_WAYPOINT_MM;
    if(!i->stable || !i->sample_ok || !finite_value(i->lux) || i->lux<0.0f ||
       distance2(i->x_mm,i->y_mm,n->target_x_mm,n->target_y_mm)>tolerance) {
        reset_point(n);return 0;
    }
    if(!n->waypoint_active) {
        n->waypoint_active=1;n->waypoint_ms=i->now_ms;n->last_light_id=i->light_id;return 0;
    }
    if((uint32_t)(i->now_ms-n->waypoint_ms)<J_NAV_SAMPLE_SETTLE_MS) { n->last_light_id=i->light_id;return 0; }
    if(i->light_id==n->last_light_id) return 0;
    n->last_light_id=i->light_id;n->point_sum+=i->lux;
    if(i->lux<n->point_min)n->point_min=i->lux;
    ++n->samples_at_point;
    if(n->samples_at_point<J_NAV_POINT_SAMPLES)return 0;
    *mean=n->point_sum/(float)n->samples_at_point;*minimum=n->point_min;
    reset_point(n);return 1;
}
static void resume_scan(JNav *n)
{
    ++n->index;
    if(n->index>=9U) {
        if(n->stage==1U) { fail(n,NAV_NO_PEAK);return; }
        n->index=0; /* Stage B observation/search shares ONE 60s budget; no dark gate. */
    }
    n->phase=J_NAV_SCAN;n->peak_valid=0;n->refine_count=0;grid(n);
}
static void probe_target(JNav *n)
{
    float x=n->peak_x,y=n->peak_y;
    switch(n->probe_index) {
        case 1:x+=J_NAV_PROBE_RADIUS_MM;break;case 2:x-=J_NAV_PROBE_RADIUS_MM;break;
        case 3:y+=J_NAV_PROBE_RADIUS_MM;break;case 4:y-=J_NAV_PROBE_RADIUS_MM;break;
        default:break;
    }
    target(n,x,y);
}
static void candidate(JNav *n,const JNavInput *i,float mean)
{
    n->peak_x=i->x_mm;n->peak_y=i->y_mm;n->peak_lux=mean;
    n->samples[0]=mean;n->sample_x[0]=i->x_mm;n->sample_y[0]=i->y_mm;
    n->phase=J_NAV_PROBE;n->probe_index=1;probe_target(n);
}
static void evaluate(JNav *n,const JNavInput *i)
{
    uint8_t k,best=0;
    float best_value=n->samples[0],minimum_radius=J_NAV_PROBE_RADIUS_MM*0.60f;
    for(k=1;k<=4U;++k) {
        if(distance2(n->sample_x[k],n->sample_y[k],n->sample_x[0],n->sample_y[0])<minimum_radius*minimum_radius) {
            resume_scan(n);return; /* insufficient spatial separation */
        }
        if(n->samples[k]>best_value){best_value=n->samples[k];best=k;}
    }
    if(best && best_value>=n->samples[0]+J_NAV_PEAK_PROMINENCE_LUX && n->refine_count<J_NAV_REFINE_STEPS) {
        if(!outside_a(n,n->sample_x[best],n->sample_y[best])){resume_scan(n);return;}
        ++n->refine_count;
        n->peak_x=n->sample_x[best];n->peak_y=n->sample_y[best];n->peak_lux=best_value;
        n->phase=J_NAV_VERIFY;n->peak_valid=0;target(n,n->peak_x,n->peak_y);return;
    }
    for(k=1;k<=4U;++k)if(n->samples[0]<n->samples[k]+J_NAV_PEAK_PROMINENCE_LUX){resume_scan(n);return;}
    if(!outside_a(n,n->peak_x,n->peak_y)){resume_scan(n);return;}
    n->peak_valid=1;n->phase=J_NAV_VERIFY;target(n,n->peak_x,n->peak_y);
    (void)i;
}
void JNav_Step(JNav *n,const JNavInput *i)
{
    float mean,minimum,threshold=i->baseline+J_LIGHT_DELTA_LUX;
    if(n->phase==J_NAV_IDLE || n->phase==J_NAV_DONE || n->phase==J_NAV_FAILED)return;
    if(!i->coordinate_ok || !finite_value(i->x_mm) || !finite_value(i->y_mm)){fail(n,NAV_BAD_REFERENCE);return;}
    if(absolute(i->x_mm)>=J_NAV_HALF_SIZE_MM || absolute(i->y_mm)>=J_NAV_HALF_SIZE_MM){fail(n,NAV_BOUNDARY);return;}
    if((uint32_t)(i->now_ms-n->stage_ms)>=J_NAV_BUDGET_MS){fail(n,NAV_TIMEOUT);return;}
    if(n->phase==J_NAV_FORCE)return; /* core owns ONE unbroken <=10s forced hold */
    if(n->phase==J_NAV_B_STAGE){n->phase=J_NAV_SCAN;n->index=0;grid(n);}
    if(!point(n,i,&mean,&minimum))return;
    if(n->phase==J_NAV_SCAN) {
        if(minimum>=threshold && outside_a(n,i->x_mm,i->y_mm)) { n->refine_count=0;candidate(n,i,mean); }
        else resume_scan(n);
    } else if(n->phase==J_NAV_PROBE) {
        uint8_t k=n->probe_index;
        n->samples[k]=mean;n->sample_x[k]=i->x_mm;n->sample_y[k]=i->y_mm;
        if(k<4U){++n->probe_index;probe_target(n);}else evaluate(n,i);
    } else if(n->phase==J_NAV_VERIFY) {
        if(minimum<threshold || absolute(mean-n->peak_lux)>J_NAV_RECHECK_TOL_LUX || !outside_a(n,i->x_mm,i->y_mm)) {
            resume_scan(n);return;
        }
        if(!n->peak_valid){candidate(n,i,mean);return;}
        n->phase=J_NAV_FORCE;n->force_requested=1;
    }
}
void JNav_HoverFinished(JNav *n,uint8_t success,const JNavInput *i)
{
    if(n->phase!=J_NAV_FORCE || !success){fail(n,NAV_NO_PEAK);return;}
    if(J_NAV_MODE==2 && n->stage==1U) {
        n->a_x=i->x_mm;n->a_y=i->y_mm;n->a_lux=i->lux;n->have_a=1;
        n->stage=2;n->stage_ms=i->now_ms;n->phase=J_NAV_B_STAGE;n->index=0;
        n->peak_valid=n->force_requested=n->refine_count=0;reset_point(n);
    } else {n->phase=J_NAV_DONE;n->force_requested=0;}
}
uint8_t JNav_Active(const JNav *n)
{ return n->phase==J_NAV_SCAN || n->phase==J_NAV_PROBE || n->phase==J_NAV_VERIFY || n->phase==J_NAV_B_STAGE; }
