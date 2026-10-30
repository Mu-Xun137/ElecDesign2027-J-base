#ifndef J_CORE_H
#define J_CORE_H
#include <stdint.h>
#include "j_config.h"
#include "j_nav.h"

enum { J_C1=1, J_C2=2, J_C3=4, J_C4=8 };
typedef enum { J_LOCKED, J_MANUAL, J_C1_HOLD, J_C3_AUTO, J_ALIGN,
               J_LIGHT_HOLD, J_FLOW_HOLD, J_LASER_WAIT } JMode;
typedef enum { J_GROUND, J_ADJUST, J_HOVER, J_LIGHT_SETTLE,
               J_LIGHT_STABLE, J_TASK_DONE, J_TASK_FAILED, J_SEARCH, J_WAIT_B } JPhase;
typedef enum { J_OK, J_ERR_ESTOP, J_ERR_RC, J_ERR_IMU, J_ERR_TILT,
               J_ERR_RANGE, J_ERR_FLOW, J_ERR_LIGHT, J_ERR_PREFLIGHT,
               J_ERR_TAKEOFF_GROUND, J_ERR_TAKEOFF_AIR, J_ERR_LIGHT_TIMEOUT,
               J_ERR_NAV, J_ERR_DEADLINE } JFault;
typedef enum { J_ALIGN_NONE, J_ALIGN_UP, J_ALIGN_DOWN, J_ALIGN_MATCH,
               J_ALIGN_UNREACHABLE } JAlign;

typedef struct {
    uint32_t range_ms, flow_ms, imu_ms, light_ms, light_id;
    float height_mm, vz_mm_s, x_px, y_px, vx_px_s, vy_px_s;
    float nav_x_mm, nav_y_mm, nav_yaw_deg, nav_heading_deg, lux, roll_deg, pitch_deg, yaw_deg;
    uint8_t range_ok, velocity_ok, flow_ok, imu_ok, light_ok, quality;
    uint8_t module_ok, nav_reference_ok;
    uint32_t module_ms; /* successful LC08 transaction, even when height is invalid */
} JSensors;

typedef struct {
    uint32_t now_ms, rc_ms;
    uint16_t channels[8]; /* roll pitch thr yaw C1 C2 C3 C4 */
    uint8_t have_rc, new_rc, events, rc_gap;
    JSensors s;
} JInput;

typedef struct {
    JMode mode, resume_mode;
    JPhase phase;
    JFault fault;
    JAlign align;
    uint8_t armed, motors_on, airborne, enabled, arrival, completed, alert_active;
    uint8_t light_inside, light_eligible, light_count, exit_count;
    uint8_t ambient_count, ambient_index, c1_centered, prev_armed,c1_launch_ready;
    uint8_t stable_active, hold_active, match_active, low_active;
    uint8_t arm_stage, arm_high_active, flow_moving_x, flow_moving_y;
    uint8_t tick_started, flow_tick_started, task_failed, nav_running,nav_requested,nav_completed,nav_pending;
    uint8_t flow_bad_active,ground_valid;
    int8_t height_alarm;
    int16_t roll_center, pitch_center;
    uint16_t light_event, takeover_event;
    uint32_t last_tick_ms, adjust_ms, alert_ms, light_task_ms,led_epoch_ms,height_led_ms;
    uint32_t stable_ms, hold_ms, match_ms, fault_ms, low_ms;
    uint32_t arm_ms, arm_high_ms, last_light_id, flow_tick_ms,flow_bad_ms;
    float ambient[J_AMBIENT_SAMPLES], baseline_lux;
    float height_target_mm, goal_mm, x_target_px, y_target_px, yaw_target,ground_mm;
    float common, last_manual, vz_integral;
    float roll_cmd, pitch_cmd, flow_roll, flow_pitch;
    float flow_ix, flow_iy, prev_vx_error, prev_vy_error;
    JNav nav;
    /* One grant per real manual unlock; every first motor start consumes it. */
    uint8_t launch_available, near_active, near_align, near_valid_active, near_valid_reads;
    uint32_t near_start_ms, near_valid_ms, near_last_range_ms;
} JCore;

float J_Clamp(float value, float low, float high);
float J_Abs(float value);
uint8_t J_Fresh(uint32_t now, uint32_t stamp, uint32_t limit);
float J_ManualThrust(uint16_t throttle);
void JCore_Init(JCore *c, uint32_t now);
void JCore_Stop(JCore *c, JFault reason, uint32_t now);
void JCore_Step(JCore *c, const JInput *in);
uint8_t JCore_LEDMask(const JCore *c, uint32_t now);
uint8_t JCore_AlertActive(const JCore *c, uint32_t now);
uint8_t JCore_HeightHealthy(const JInput *in);
uint8_t JCore_FlowHealthy(const JInput *in);
uint8_t JCore_Stable(const JInput *in);
#endif
