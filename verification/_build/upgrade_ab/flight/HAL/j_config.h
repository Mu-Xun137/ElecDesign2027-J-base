#ifndef J_CONFIG_H
#define J_CONFIG_H
/* All heights are mm, vertical velocities mm/s, flow is native pixels/s.
 * Initial engineering values: calibrate thrust, PID, optics and flow on hardware. */
#define J_VERSION                 0x31U
#define J_TARGET_MM               800.0f
#define J_ALARM_LOW_MM            600.0f
#define J_ALARM_HIGH_MM           1000.0f
#define J_HEIGHT_MIN_MM           10.0f
#define J_HEIGHT_MAX_MM           3500.0f
#define J_AIRBORNE_MM             200.0f
#define J_LIFTOFF_DELTA_MM        80.0f
#define J_LIFTOFF_TIMEOUT_MS      10000U
#define J_ALERT_MS                3000U
#define J_STABLE_CONFIRM_MS       300U
#define J_LIGHT_HOLD_MS           3000U
#define J_LIGHT_TIMEOUT_MS        10000U
#define J_LASER_WAIT_MS           10000U
#define J_RC_LOST_MS              500U
#define J_IMU_FRESH_MS            30U
#define J_RANGE_FRESH_MS          120U
#define J_RANGE_FAIL_MS           300U
#define J_FLOW_FRESH_MS           150U
#define J_FLOW_FAIL_MS            350U
#define J_LIGHT_FRESH_MS          650U
#define J_CONTROL_DEADLINE_MS     25U
#define J_MAX_TILT_DEG            50.0f
#define J_STABLE_TILT_DEG         8.0f
#define J_STABLE_HEIGHT_MM        50.0f
#define J_STABLE_VZ_MM_S          50.0f
#define J_STABLE_FLOW_PX_S        50.0f
#define J_FLOW_MIN_QUALITY        31U
#define J_FLOW_STABLE_QUALITY     40U
#define J_AMBIENT_SAMPLES         5U
#define J_LIGHT_DELTA_LUX         250.0f
#define J_LIGHT_HYST_LUX          50.0f
#define J_LIGHT_CONFIRM_SAMPLES   3U
#define J_BH_CONVERSION_MS        185U
#define J_BH_PERIOD_MS            200U
#define J_BH_RETRY_MS             500U
#define J_HEIGHT_VEL_WINDOW_MS    30U
#define J_HEIGHT_VEL_ALPHA        0.20f
#define J_THR_HOVER               500.0f
#define J_THR_AUTO_MAX            800.0f
#define J_THR_SLEW_PER_S          450.0f
#define J_HEIGHT_TARGET_MM_S      250.0f
#define J_CLIMB_MAX_MM_S          250.0f
#define J_DESCEND_MAX_MM_S        350.0f
#define J_HEIGHT_P                1.2f
#define J_VZ_P                    0.55f
#define J_VZ_I                    0.08f
#define J_VZ_I_LIMIT              200.0f
#define J_ALIGN_TOL_PWM           12.0f
#define J_ALIGN_HOLD_MS           200U
#define J_ALIGN_RC_FRESH_MS       60U
#define J_STICK_CENTER_TOL        100
#define J_MANUAL_THR_MAX_CHANNEL  1819U
#define J_LOW_THR_STOP_MS         900U
#define J_LOW_THR_LOCK_MS         9000U
#define J_UNLOCK_HIGH_HOLD_MS     60U
#define J_UNLOCK_SEQUENCE_MS      10000U
#define J_FLOW_CONTROL_MS        24U
#define J_FLOW_MANUAL_PX_S        625.0f
#define J_FLOW_MAX_PX_S           833.3333f
#define J_FLOW_POS_P              0.8333333f
#define J_FLOW_RATE_P             0.0036f
#define J_FLOW_RATE_I             0.00096f
#define J_FLOW_RATE_D             0.00084f
#define J_FLOW_I_LIMIT_DEG        3.0f
#define J_FLOW_MAX_TILT_DEG       15.0f
#define J_ATTITUDE_SLEW_DEG_S     90.0f
#define J_RATE_XY_KP              3.0f
#define J_RATE_XY_KI              0.05f
#define J_RATE_XY_KD              0.2f
#define J_RATE_YAW_KP             8.0f
#define J_RATE_YAW_KI             0.0f
#define J_RATE_YAW_KD             0.4f
#define J_ANGLE_XY_KP             8.0f
#define J_ANGLE_YAW_KP            6.0f
#define J_RATE_XY_PWM_LIMIT       250.0f
#define J_RATE_YAW_PWM_LIMIT      150.0f
#define J_RATE_XY_I_LIMIT         200.0f
#define J_RATE_YAW_I_LIMIT        100.0f
#define J_ANGLE_XY_RATE_LIMIT     250.0f
#define J_ANGLE_YAW_RATE_LIMIT    180.0f
#define J_YAW_RATE_DEG_S          50.0f
#define J_YAW_TARGET_SLEW_DEG_S   90.0f
/* Native-pixel tilt compensation, inherited 700px/rad at 800mm; NOT mm. */
#define J_FLOW_TILT_PX_AT800      700.0f
#define J_LED_ACTIVE_LOW          1
#define J_LED_LF_CHANNEL          2U
#define J_LED_RF_CHANNEL          1U
#define J_LED_RR_CHANNEL          4U
#define J_LED_LR_CHANNEL          3U
#define J_LED_BLUE_MASK           0x03U /* bits: LF RF RR LR */
#define J_LED_RED_MASK            0x0CU
#define J_LED_IDLE_MS             200U
#define J_LED_GROUND_MS           500U
#define J_LED_HEIGHT_MS           500U
#define J_LED_LIGHT_MS            100U
#define J_RC_PERIOD_MS            20U
#define J_REPLY_DELAY_US          1800U
#define J_RADIO_TX_TIMEOUT_US     3000U
#define J_FEEDBACK_FRESH_MS       500U
#define J_FEEDBACK_ECHO_WINDOW    5U
#define J_FEEDBACK_ECHO_MAX_MS    100U
#define J_KEY_DEBOUNCE_MS         20U
#define J_BEEP_CLICK_MS           30U
#define J_BEEP_SHORT_MS           90U
#define J_BEEP_LONG_MS            250U
#define J_BEEP_GAP_MS             100U
#define J_BEEP_GROUP_MS           1000U
#define J_BEEP_FAULT_MS           60U
#define J_BEEP_FAULT_GAP_MS       60U
#define J_BEEP_HEIGHT_MS          100U
#ifndef J_NAV_MODE
#define J_NAV_MODE                2 /* base=0; upgrade single=1, A/B=2 */
#endif
#ifndef J_NAV_CALIBRATED
#define J_NAV_CALIBRATED          0 /* MUST measure pixel scale before enabling */
#endif
#define J_NAV_MM_PER_PX_AT800     1.0f /* uncalibrated initial placeholder */
#define J_NAV_BUDGET_MS           60000U
#define J_NAV_HALF_SIZE_MM        500.0f
#define J_NAV_GRID_STEP_MM        350.0f
#define J_NAV_SPEED_MM_S          180.0f
#define J_NAV_POSITION_P          3.0f
#define J_NAV_WAYPOINT_MM         30.0f
#define J_NAV_SAMPLE_SETTLE_MS    300U
#define J_NAV_POINT_SAMPLES       3U
#define J_NAV_PEAK_PROMINENCE_LUX 60.0f
#define J_NAV_RECHECK_TOL_LUX     100.0f
#define J_NAV_PROBE_RADIUS_MM     100.0f
#define J_NAV_REFINE_STEPS        2U
#define J_NAV_A_EXCLUDE_MM        180.0f
#define J_NAV_CONTINUITY_GAP_MS   350U
#endif
