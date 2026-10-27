#ifndef J_NAV_H
#define J_NAV_H
#include <stdint.h>
/* Fixed storage; the upgrade replaces j_nav.c and enables J_NAV_MODE. */
typedef struct {
    float x_mm, y_mm, lux, baseline;
    uint32_t now_ms, light_id;
    uint8_t coordinate_ok, sample_ok, stable, light_inside;
} JNavInput;
typedef enum { J_NAV_IDLE, J_NAV_SCAN, J_NAV_PROBE, J_NAV_VERIFY,
               J_NAV_FORCE, J_NAV_B_STAGE, J_NAV_DONE, J_NAV_FAILED } JNavPhase;
typedef struct {
    JNavPhase phase;
    uint32_t stage_ms, waypoint_ms, last_light_id;
    float target_x_mm, target_y_mm, peak_x, peak_y, peak_lux;
    float a_x, a_y, a_lux, samples[5], scan_peak;
    float sample_x[5],sample_y[5],point_sum,point_min;
    uint8_t index, probe_index, waypoint_active, settled, samples_at_point;
    uint8_t stage, have_a, peak_valid, force_requested, failure,refine_count;
} JNav;
void JNav_Init(JNav *nav);
uint8_t JNav_Start(JNav *nav, const JNavInput *in);
void JNav_Step(JNav *nav, const JNavInput *in);
void JNav_HoverFinished(JNav *nav, uint8_t success, const JNavInput *in);
uint8_t JNav_Active(const JNav *nav);
#endif
