#include "ALL_DEFINE.h"
#include "j_platform.h"
#include "j_protocol.h"
#include "j_boot.h"
#include "j_bh1750.h"
#include "flow.h"
#include <string.h>
extern uint8_t JRadio_StartTx(uint8_t *p,uint32_t now_us);
extern void JRadio_Poll(uint32_t now_us);
extern volatile uint8_t j_radio_tx_busy,j_mpu_ready,j_mpu_sample_ok;
extern volatile uint32_t j_mpu_last_ms;
JCore j_flight;
JSensorFilter j_sensor;
volatile uint32_t j_last_motor_ms,j_max_isr_us,j_deadline_misses;
volatile uint8_t j_deadline_fault;
static JReceiver rx;
static JInput command;
static uint32_t boot_session,reply_start_us;
static uint8_t pending_reply;
static uint16_t telemetry_seq;
void JPlatform_Init(void)
{
    uint8_t k;
    JCore_Init(&j_flight,SysTick_count); JSensors_Init(&j_sensor); JBH_Init();
    memset(&rx,0,sizeof(rx)); memset(&command,0,sizeof(command));
    for(k=0;k<8U;++k) command.channels[k]=k==0U||k==1U||k==3U?1500U:1000U;
    boot_session=JBoot_Session(GetSysTime_us());
    j_last_motor_ms=SysTick_count;
}
void JPlatform_ZeroMotors(void)
{
    TIM2->CCR1=0; TIM2->CCR2=0; TIM2->CCR3=0; TIM2->CCR4=0;
    motor_PWM_Value[0]=motor_PWM_Value[1]=motor_PWM_Value[2]=motor_PWM_Value[3]=0;
}
void JPlatform_ApplyLED(uint8_t mask)
{
    const uint8_t channels[4]={J_LED_LF_CHANNEL,J_LED_RF_CHANNEL,J_LED_RR_CHANNEL,J_LED_LR_CHANNEL};
    uint8_t k;
    for(k=0;k<4U;++k) {
        uint16_t value=(mask&(1U<<k))?(J_LED_ACTIVE_LOW?0U:1000U):(J_LED_ACTIVE_LOW?1000U:0U);
        switch(channels[k]) {
            case 1:TIM3->CCR1=value;break; case 2:TIM3->CCR2=value;break;
            case 3:TIM3->CCR3=value;break; case 4:TIM3->CCR4=value;break;
            default:break;
        }
    }
}
void JPlatform_ControlTick(float dt,uint8_t module_tick)
{
    JInput in;
    uint8_t was_armed=j_flight.armed;
    uint32_t mask;
    MpuGetData();
    if(j_mpu_sample_ok) GetAngle(&MPU6050,&Angle,dt);
    j_sensor.value.roll_deg=Angle.roll; j_sensor.value.pitch_deg=Angle.pitch;
    j_sensor.value.yaw_deg=Angle.yaw; j_sensor.value.imu_ms=j_mpu_last_ms;
    j_sensor.value.imu_ok=j_mpu_ready && J_Fresh(SysTick_count,j_mpu_last_ms,J_IMU_FRESH_MS);
    if(module_tick) { flow_get_data(); JBH_Tick(&j_sensor.value,SysTick_count); }
    mask=__get_PRIMASK(); __disable_irq();
    in=command; command.events=command.new_rc=command.rc_gap=0;
    __set_PRIMASK(mask);
    in.now_ms=SysTick_count; in.s=j_sensor.value;
    if(j_deadline_fault) JCore_Stop(&j_flight,J_ERR_DEADLINE,in.now_ms);
    else JCore_Step(&j_flight,&in);
    if(!was_armed && j_flight.armed) JSensors_ArmReference(&j_sensor);
    ALL_flag.unlock=j_flight.armed;
    ALL_flag.height_lock=j_flight.armed && j_flight.mode!=J_MANUAL && j_flight.mode!=J_LASER_WAIT;
    ALL_flag.flow_control=ALL_flag.height_lock && j_flight.mode!=J_FLOW_HOLD;
    pidHeightHigh.measured=in.s.height_mm; pidHeightHigh.desired=j_flight.height_target_mm;
    pidHeightRate.measured=in.s.vz_mm_s;
    if(j_mpu_sample_ok) FlightPidControl(dt);
    MotorControl();
    j_last_motor_ms=SysTick_count;
    JPlatform_ApplyLED(JCore_LEDMask(&j_flight,in.now_ms));
}
void JPlatform_OnControl(const uint8_t *packet,uint32_t now)
{
    JControlPacket p;
    uint8_t events,gap,k;
    uint32_t mask;
    if(!JReceiver_Accept(&rx,packet,now,&p,&events,&gap)) return;
    mask=__get_PRIMASK(); __disable_irq();
    for(k=0;k<8U;++k) command.channels[k]=p.channel[k];
    command.have_rc=1; command.new_rc=1; command.rc_ms=now;
    command.events|=events; command.rc_gap|=gap;
    Remote.roll=p.channel[0]; Remote.pitch=p.channel[1]; Remote.thr=p.channel[2]; Remote.yaw=p.channel[3];
    Remote.AUX1=p.channel[4]; Remote.AUX2=p.channel[5]; Remote.AUX3=p.channel[6]; Remote.AUX4=p.channel[7];
    /* C2 stops even before the next 3ms motor cycle, and cannot be swallowed by C1. */
    if(events&J_C2) { JCore_Stop(&j_flight,J_ERR_ESTOP,now); ALL_flag.unlock=0; JPlatform_ZeroMotors(); }
    __set_PRIMASK(mask);
    if(p.reply) { pending_reply=1; reply_start_us=GetSysTime_us(); }
}
static void send_status(void)
{
    JStatusPacket s;
    uint8_t packet[32];
    uint32_t mask,now;
    memset(&s,0,sizeof(s));
    mask=__get_PRIMASK(); __disable_irq();
    now=SysTick_count; /* timestamp and event/state snapshot must share the same critical section */
    s.control_session=rx.session; s.echo_seq=rx.seq; s.flight_session=boot_session;
    s.telemetry_seq=++telemetry_seq;
    s.mode=(uint8_t)j_flight.mode; s.phase=(uint8_t)j_flight.phase;
    s.flags=(j_flight.armed?J_STATUS_ARMED:0U)|(JCore_AlertActive(&j_flight,now)?J_STATUS_ALERT:0U)|
        (j_flight.arrival?J_STATUS_ARRIVAL:0U)|(j_flight.task_failed?J_STATUS_FAILED:0U)|
        ((j_flight.mode==J_ALIGN||j_flight.mode==J_LASER_WAIT)?J_STATUS_ALIGN:0U)|
        (j_flight.fault!=J_OK?J_STATUS_FAULT:0U)|
        ((j_flight.nav_requested?j_flight.nav_completed:j_flight.completed)?J_STATUS_COMPLETE:0U)|
        (j_flight.fault==J_ERR_ESTOP?J_STATUS_ESTOP:0U);
    s.align=(uint8_t)j_flight.align; s.fault=(uint8_t)j_flight.fault;
    s.light_event=j_flight.light_event; s.takeover_event=j_flight.takeover_event;
    s.alert_elapsed_ms=(uint16_t)J_Clamp((float)(uint32_t)(now-j_flight.alert_ms),0.0f,65535.0f);
    s.height_mm=(uint16_t)J_Clamp(j_sensor.value.height_mm,0.0f,65535.0f);
    s.sensors=((j_sensor.value.range_ok&&J_Fresh(now,j_sensor.value.range_ms,J_RANGE_FRESH_MS))?1U:0U)|
        ((j_sensor.value.flow_ok&&J_Fresh(now,j_sensor.value.flow_ms,J_FLOW_FRESH_MS))?2U:0U)|
        ((j_sensor.value.imu_ok&&J_Fresh(now,j_sensor.value.imu_ms,J_IMU_FRESH_MS))?4U:0U)|
        ((j_sensor.value.light_ok&&J_Fresh(now,j_sensor.value.light_ms,J_LIGHT_FRESH_MS))?8U:0U)|
        (j_flight.height_alarm<0?J_SENSOR_LOW:0U)|(j_flight.height_alarm>0?J_SENSOR_HIGH:0U);
    __set_PRIMASK(mask);
    JStatus_Encode(packet,&s);
    JRadio_StartTx(packet,GetSysTime_us());
}
void JPlatform_RadioPoll(void)
{
    uint8_t packet[32],k;
    uint32_t now_us=GetSysTime_us();
    JRadio_Poll(now_us);
    for(k=0;k<3U;++k) {
        if(NRF24L01_RxPacket(packet)) break;
        JPlatform_OnControl(packet,SysTick_count);
    }
    now_us=GetSysTime_us();
    if(pending_reply && !j_radio_tx_busy && (uint32_t)(now_us-reply_start_us)>=J_REPLY_DELAY_US) {
        pending_reply=0; send_status();
    }
}
