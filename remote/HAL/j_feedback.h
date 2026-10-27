#ifndef J_FEEDBACK_H
#define J_FEEDBACK_H
#include "j_protocol.h"
typedef struct {
    JStatusPacket status;
    uint32_t last_ms, retired_flight_session, alert_start_ms, success_ms, click_ms, group_ms;
    uint16_t light_seen, takeover_seen;
    uint8_t have, alert_on, success_on, click_on, group_kind, alternate;
} JFeedback;
uint8_t JFeedback_Accept(JFeedback *feedback,const uint8_t *p,uint32_t now,
                         uint32_t remote_session,uint16_t latest_sent_seq,uint32_t latest_sent_ms);
void JFeedback_Click(JFeedback *feedback,uint32_t now);
uint8_t JFeedback_Buzzer(JFeedback *feedback,uint32_t now);
#endif
