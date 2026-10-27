#ifndef J_PROTOCOL_H
#define J_PROTOCOL_H
#include <stdint.h>
#include "j_config.h"
#define J_PKT_SIZE 32
#define J_STATUS_ARMED     0x01U
#define J_STATUS_ALERT     0x02U
#define J_STATUS_ARRIVAL   0x04U
#define J_STATUS_FAILED    0x08U
#define J_STATUS_ALIGN     0x10U
#define J_STATUS_FAULT     0x20U
#define J_STATUS_COMPLETE  0x40U
#define J_STATUS_ESTOP     0x80U
#define J_SENSOR_LOW       0x10U
#define J_SENSOR_HIGH      0x20U
typedef struct {
    uint16_t channel[8], seq;
    uint32_t session;
    uint8_t key_count[4], pressed, reply,ready_keys;
} JControlPacket;
typedef struct {
    uint32_t session, previous_session, last_ms;
    uint16_t seq;
    uint8_t count[4], ready;
} JReceiver;
typedef struct {
    uint32_t control_session, flight_session;
    uint16_t echo_seq, telemetry_seq, light_event, takeover_event;
    uint16_t alert_elapsed_ms, height_mm;
    uint8_t mode, phase, flags, align, fault, sensors;
} JStatusPacket;
uint8_t JPacket_Check(const uint8_t *p, uint8_t type);
void JControl_Encode(uint8_t *p, const JControlPacket *control);
uint8_t JControl_Decode(const uint8_t *p, JControlPacket *control);
/* 1: valid NEW frame; ordinary events=0 on first frame, session change or link recovery.
 * A C2 level that passed real physical debounce always stops, including at boot.
 * gap=1 tells the motor state machine that a >=500ms outage MUST be latched. */
uint8_t JReceiver_Accept(JReceiver *rx, const uint8_t *p, uint32_t now,
                        JControlPacket *control, uint8_t *events, uint8_t *gap);
void JStatus_Encode(uint8_t *p, const JStatusPacket *status);
uint8_t JStatus_Decode(const uint8_t *p, JStatusPacket *status);
#endif
