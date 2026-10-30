#include "ALL_DEFINE.h"
#include "flow.h"
#include "j_platform.h"
FLow flow_data;
uint32_t VL53L01_high;
float flow_x_lpf_att_i,flow_y_lpf_att_i,flow_x_vel_lpf_i,flow_y_vel_lpf_i;
void flow_get_data(void)
{
    uint8_t b[5]={0};
    uint8_t ok=IIC_read_Bytes(0xC4,0x33,b,5)==0;
    uint16_t height_mm=(uint16_t)((uint16_t)b[0]<<8|b[1]);
    JSensors_Module(&j_sensor,height_mm,(int8_t)b[2],(int8_t)b[3],b[4],ok,SysTick_count);
    if(j_sensor.value.range_ok) VL53L01_high=(uint32_t)j_sensor.value.height_mm;
    flow_data.vel_x=(int8_t)b[2]; flow_data.vel_y=(int8_t)b[3]; flow_data.squal=b[4];
    flow_data.pos_x_i=j_sensor.value.x_px; flow_data.pos_y_i=j_sensor.value.y_px;
    flow_x_lpf_att_i=j_sensor.value.x_px; flow_y_lpf_att_i=j_sensor.value.y_px;
    flow_x_vel_lpf_i=j_sensor.value.vx_px_s; flow_y_vel_lpf_i=j_sensor.value.vy_px_s;
    sins_high=j_sensor.value.height_mm; sins_vel=j_sensor.value.vz_mm_s;
}
void flow_data_sins(void) { /* Filtering is performed once per successful module transaction. */ }
void flow_sins_rest(void)
{
    /* No in-air coordinate reset. A lost reference requires landing/rearming. */
    if(j_flight.armed) j_sensor.value.nav_reference_ok=0;
    else JSensors_Init(&j_sensor);
}
