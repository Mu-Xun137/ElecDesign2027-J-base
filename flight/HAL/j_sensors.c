#include "j_sensors.h"
#include <math.h>
#include <string.h>
void JSensors_Init(JSensorFilter *f) { memset(f,0,sizeof(*f)); }
void JSensors_ArmReference(JSensorFilter *f)
{
    f->reference_established=1;
    f->value.nav_reference_ok=f->value.module_ok;
    f->nav_heading=f->value.yaw_deg;
    f->value.nav_heading_deg=f->nav_heading;
    f->nav_x=f->nav_y=0.0f;
    f->value.nav_x_mm=f->value.nav_y_mm=0.0f;
    f->last_flow_good_ms=f->value.range_ms;
    f->in_air=0;
}
void JSensors_Module(JSensorFilter *f,uint16_t height_mm,int8_t dx,int8_t dy,
                      uint8_t quality,uint8_t ok,uint32_t now)
{
    JSensors *s=&f->value;
    /* This board returns mm: about 300 mm physically yielded raw 291..294.
     * Do not apply the conflicting vendor diagram's cm label to this module. */
    float h=(float)height_mm;
    uint32_t gap;
    s->module_ok=ok;
    if(ok)s->module_ms=now;
    s->quality=quality;
    if(ok && height_mm && h>=J_HEIGHT_MIN_MM && h<=J_HEIGHT_MAX_MM) {
        gap=(uint32_t)(now-s->range_ms);
        s->range_ms=now; s->range_ok=1; s->height_mm=h;
        if(!f->height_started || gap>J_RANGE_FRESH_MS) {
            f->height_started=1; f->height_base=h; f->height_base_ms=now;
            s->velocity_ok=0; /* old/invalid zero velocity NEVER counts as stable */
        } else if((uint32_t)(now-f->height_base_ms)>=J_HEIGHT_VEL_WINDOW_MS) {
            float dt=(float)(uint32_t)(now-f->height_base_ms)*0.001f;
            float v=(h-f->height_base)/dt;
            if(!s->velocity_ok) s->vz_mm_s=v;
            else s->vz_mm_s+=J_HEIGHT_VEL_ALPHA*(v-s->vz_mm_s);
            s->velocity_ok=1;
            f->height_base=h; f->height_base_ms=now;
        }
    } else { s->range_ok=0; s->velocity_ok=0; }
    if(s->height_mm>J_AIRBORNE_MM && s->range_ok && !f->in_air) {
        f->in_air=1; f->last_flow_good_ms=now; /* finite allowance for the normal near-ground quality zone */
    }
    if(f->reference_established && f->in_air &&
       (uint32_t)(now-f->last_flow_good_ms)>J_NAV_CONTINUITY_GAP_MS) s->nav_reference_ok=0;
    if(ok && s->range_ok && quality>=J_FLOW_MIN_QUALITY && h>=80.0f &&
       dx!=-128 && dy!=-128 && dx!=127 && dy!=127) {
        float angle=(s->yaw_deg-f->nav_heading)*0.01745329252f;
        float step_x=(float)dx*J_NAV_MM_PER_PX_AT800*h/J_TARGET_MM;
        float step_y=(float)dy*J_NAV_MM_PER_PX_AT800*h/J_TARGET_MM;
        float tilt_x=J_FLOW_TILT_PX_AT800*(h/J_TARGET_MM)*tanf(s->roll_deg*0.01745329252f);
        float tilt_y=J_FLOW_TILT_PX_AT800*(h/J_TARGET_MM)*tanf(s->pitch_deg*0.01745329252f);
        float dt;
        gap=(uint32_t)(now-s->flow_ms);
        f->raw_x+=(float)dx; f->raw_y+=(float)dy;
        if(!f->flow_started || gap>J_FLOW_FRESH_MS) {
            f->filtered_x=f->raw_x+tilt_x; f->filtered_y=f->raw_y-tilt_y;
            f->last_x=f->filtered_x; f->last_y=f->filtered_y;
            f->flow_base_ms=now; f->flow_started=1;
            s->vx_px_s=s->vy_px_s=0.0f;
            s->flow_ok=0; /* need a second independent read to establish velocity */
        } else {
            f->filtered_x+=0.10f*(f->raw_x+tilt_x-f->filtered_x);
            f->filtered_y+=0.10f*(f->raw_y-tilt_y-f->filtered_y);
            dt=(float)(uint32_t)(now-f->flow_base_ms)*0.001f;
            if(dt>0.0f) {
                s->vx_px_s+=0.15f*((f->filtered_x-f->last_x)/dt-s->vx_px_s);
                s->vy_px_s+=0.15f*((f->filtered_y-f->last_y)/dt-s->vy_px_s);
                f->last_x=f->filtered_x; f->last_y=f->filtered_y; f->flow_base_ms=now;
                s->flow_ok=1;
            }
        }
        s->x_px=f->filtered_x; s->y_px=f->filtered_y;
        s->flow_ms=now; f->last_flow_good_ms=now;
        if(f->reference_established && s->nav_reference_ok) {
            f->nav_x+=step_x*cosf(angle)-step_y*sinf(angle);
            f->nav_y+=step_x*sinf(angle)+step_y*cosf(angle);
            s->nav_x_mm=f->nav_x; s->nav_y_mm=f->nav_y;
        }
        s->nav_yaw_deg=s->yaw_deg-f->nav_heading;
    } else s->flow_ok=0; /* NEVER clear position to manufacture a new reference */
    if(f->reference_established && f->in_air &&
       (uint32_t)(now-f->last_flow_good_ms)>J_NAV_CONTINUITY_GAP_MS) s->nav_reference_ok=0;
}
