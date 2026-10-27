#ifndef J_SENSORS_H
#define J_SENSORS_H
#include "j_core.h"
typedef struct {
    JSensors value;
    uint32_t height_base_ms, flow_base_ms, last_flow_good_ms;
    float height_base, raw_x, raw_y, filtered_x, filtered_y;
    float last_x, last_y, nav_heading, nav_x, nav_y;
    uint8_t height_started, flow_started, reference_established, in_air;
} JSensorFilter;
void JSensors_Init(JSensorFilter *f);
void JSensors_ArmReference(JSensorFilter *f);
void JSensors_Module(JSensorFilter *f,uint16_t height_cm,int8_t dx,int8_t dy,
                      uint8_t quality,uint8_t io_ok,uint32_t now);
#endif
