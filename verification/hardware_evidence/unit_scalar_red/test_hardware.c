/* Compile the REAL firmware adapters/drivers against a register/I2C fixture.
 * Only physical I/O and the attitude estimator/accelerometer filter are mocked. */
#include "ALL_DEFINE.h"
#include "j_platform.h"
#include "j_bh1750.h"
#include "j_bh_i2c.h"
#include "j_protocol.h"
#include "control.h"
#include "flow.h"
#include "kalman.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
JMockTimer mock_tim1,mock_tim2,mock_tim3;
JMockSysTick mock_systick={71999,71999};
uint32_t SystemCoreClock=72000000,mock_primask;
volatile uint32_t SysTick_count;
_st_Mpu MPU6050;
_st_AngE Angle;
_st_Remote Remote;
_st_ALL_flag ALL_flag;
PidObject pidRateX,pidRateY,pidRateZ,pidPitch,pidRoll,pidYaw;
PidObject pidHeightRate,pidHeightHigh,pidPositionX,pidPositionY,pidPosRateX,pidPosRateY;
int16_t motor_PWM_Value[4];
extern volatile uint8_t j_mpu_ready,j_mpu_sample_ok;
extern volatile uint32_t j_mpu_last_ms;
extern int16_t MpuOffset[6];
extern void SysTick_Handler(void),TIM1_UP_IRQHandler(void);
static unsigned checks,cases,range_reads,bh_reads,command_count,bh_bus_inits;
static const char *current;
static uint8_t imu_bad,lc_bad,bh_bad,tim_pending;
static uint16_t range_cm=80,bh_raw=480;
static uint8_t commands[32],reply[32],reply_count;
static uint32_t session_seed=0x55667788U;
#define CHECK(x) do { ++checks; if(!(x)) { printf("FAIL %s:%d: %s\n",current,__LINE__,#x); exit(2); } } while(0)
#define RUN(f) do { current=#f; f(); ++cases; printf("PASS %s\n",current); } while(0)
int SysTick_Config(uint32_t ticks) { mock_systick.LOAD=ticks-1U;mock_systick.VAL=ticks-1U;return 0; }
int TIM_GetITStatus(JMockTimer *t,int b) { (void)t;(void)b;return tim_pending; }
void TIM_ClearITPendingBit(JMockTimer *t,int b) { (void)t;(void)b;tim_pending=0; }
void TIM_ITConfig(JMockTimer *t,int b,int e) { (void)t;(void)b;(void)e; }
void GetAngle(const _st_Mpu *m,_st_AngE *a,float dt) { (void)m;(void)a;(void)dt; }
void kalman_1(struct _1_ekf_filter *f,float x) { f->out=x; }
void FLASH_read(int16_t *p,uint8_t n) { memset(p,0,n*sizeof(*p)); }
void FLASH_write(int16_t *p,uint8_t n) { (void)p;(void)n; }
int8_t IIC_Write_One_Byte(uint8_t a,uint8_t r,uint8_t d)
{ (void)a;(void)r;(void)d;return imu_bad; }
int8_t IIC_read_Bytes(uint8_t a,uint8_t r,uint8_t *b,uint8_t n)
{
    memset(b,0,n);
    if(a==0xD0 && r==0x3B && n==14) { if(imu_bad)return 1; b[4]=0x20;return 0; }
    if(a==0xD0 && r==0x75 && n==1) { if(imu_bad)return 1;b[0]=0x68;return 0; }
    if(a==0xC4 && r==0x33 && n==5) { ++range_reads;if(lc_bad)return 1;b[0]=(uint8_t)(range_cm>>8);b[1]=(uint8_t)range_cm;b[4]=80;return 0; }
    return 1;
}
void JBHI2C_Init(void) { ++bh_bus_inits; }
int8_t JBHI2C_SendCommand(uint8_t command)
{ if(command_count<32)commands[command_count++]=command;return bh_bad; }
int8_t JBHI2C_Receive(uint8_t *b,uint8_t n)
{ CHECK(n==2);++bh_reads;if(bh_bad)return 1;b[0]=(uint8_t)(bh_raw>>8);b[1]=(uint8_t)bh_raw;return 0; }
uint32_t JBoot_Session(uint32_t entropy) { (void)entropy;return ++session_seed; }
uint8_t NRF24L01_RxPacket(uint8_t *p) { (void)p;return 1; }
volatile uint8_t j_radio_tx_busy;
void JRadio_Poll(uint32_t t) { (void)t; }
uint8_t JRadio_StartTx(uint8_t *p,uint32_t t) { (void)t;memcpy(reply,p,32);++reply_count;return 1; }
static void fixture(void)
{
    SysTick_count=1000;imu_bad=lc_bad=bh_bad=0;range_cm=80;bh_raw=480;
    memset(&MPU6050,0,sizeof(MPU6050));memset(&Angle,0,sizeof(Angle));memset((void *)&ALL_flag,0,sizeof(ALL_flag));
    memset(MpuOffset,0,12);j_mpu_ready=1;j_mpu_last_ms=SysTick_count;
    j_deadline_fault=0;range_reads=bh_reads=command_count=reply_count=bh_bus_inits=0;
    JPlatform_Init();JPlatform_ZeroMotors();
}
static void limits(PidObject *p,float out,float integ)
{ memset((void *)p,0,sizeof(*p));p->OutLimitHigh=out;p->OutLimitLow=-out;p->IntegLimitHigh=integ;p->IntegLimitLow=-integ; }
static JControlPacket control(void)
{
    JControlPacket c;unsigned k;memset(&c,0,sizeof(c));
    for(k=0;k<8;++k)c.channel[k]=(k==0||k==1||k==3)?1500:1000;
    c.session=123;c.seq=1;c.reply=1;return c;
}
static void deliver(JControlPacket *c)
{ uint8_t p[32];JControl_Encode(p,c);JPlatform_OnControl(p,SysTick_count); }
static void pwm_mixer_and_lock(void)
{
    fixture();j_flight.armed=j_flight.motors_on=1;j_flight.common=J_ManualThrust(1667);
    pidRateX.out=20;pidRateY.out=-10;pidRateZ.out=5;MotorControl();
    CHECK(TIM2->CCR1==515&&TIM2->CCR2==525&&TIM2->CCR3==465&&TIM2->CCR4==495);
    j_flight.armed=0;MotorControl();CHECK(!(TIM2->CCR1|TIM2->CCR2|TIM2->CCR3|TIM2->CCR4));
    j_flight.armed=1;j_deadline_fault=1;MotorControl();CHECK(TIM2->CCR1==0);
}
static void c2_real_parser_clears_pwm_before_control_tick(void)
{
    JControlPacket c=control();fixture();deliver(&c);
    j_flight.armed=j_flight.motors_on=1;j_flight.mode=J_LIGHT_HOLD;j_flight.common=500;
    TIM2->CCR1=TIM2->CCR2=TIM2->CCR3=TIM2->CCR4=600;
    ++c.seq;++c.key_count[0];++c.key_count[1];deliver(&c);
    CHECK(!j_flight.armed&&j_flight.fault==J_ERR_ESTOP);
    CHECK(!(TIM2->CCR1|TIM2->CCR2|TIM2->CCR3|TIM2->CCR4)&&!ALL_flag.unlock);
    CHECK(mock_primask==0);
}
static void actual_led_channel_mapping_and_low_polarity(void)
{
    fixture();JPlatform_ApplyLED(1);CHECK(TIM3->CCR2==0&&TIM3->CCR1==1000&&TIM3->CCR4==1000&&TIM3->CCR3==1000);
    JPlatform_ApplyLED(2);CHECK(TIM3->CCR1==0&&TIM3->CCR2==1000);
    JPlatform_ApplyLED(4);CHECK(TIM3->CCR4==0);JPlatform_ApplyLED(8);CHECK(TIM3->CCR3==0);
    JPlatform_ApplyLED(15);CHECK(!(TIM3->CCR1|TIM3->CCR2|TIM3->CCR3|TIM3->CCR4));
}
static void real_lc08_driver_units_and_no_silent_mode_exit(void)
{
    fixture();flow_get_data();CHECK(VL53L01_high==800&&j_sensor.value.height_mm==800);
    ALL_flag.height_lock=ALL_flag.flow_control=1;lc_bad=1;flow_get_data();
    CHECK(!j_sensor.value.range_ok&&ALL_flag.height_lock&&ALL_flag.flow_control&&VL53L01_high==800);
    lc_bad=0;range_cm=0;flow_get_data();CHECK(!j_sensor.value.range_ok);
}
static void measured_millimetre_frames_and_airborne_c1(void)
{
    JControlPacket c=control();unsigned k;
    fixture();range_cm=294;flow_get_data();
    /* Real board at about 300 mm returned 294; the old firmware displayed 2940. */
    CHECK(j_sensor.value.range_ok && j_sensor.value.height_mm==294);
    range_cm=800;
    for(k=0;k<8;++k) { SysTick_count+=6;flow_get_data(); }
    CHECK(j_sensor.value.range_ok && j_sensor.value.velocity_ok && j_sensor.value.flow_ok);
    CHECK(j_sensor.value.height_mm==800 && J_Abs(j_sensor.value.vz_mm_s)<0.01f);
    deliver(&c);
    j_flight.armed=j_flight.motors_on=1;j_flight.mode=J_MANUAL;
    j_flight.common=J_ManualThrust(1667);
    ++c.seq;++c.key_count[0];deliver(&c);SysTick_count+=3;
    JPlatform_ControlTick(0.003f,0);
    CHECK(j_flight.mode==J_C1_HOLD && j_flight.fault==J_OK);
    CHECK(j_flight.goal_mm==800 && j_flight.height_target_mm==800);
}
static void real_bh_driver_distinct_one_shot_samples(void)
{
    JSensors s={0};fixture();JBH_Init();
    JBH_Tick(&s,0);JBH_Tick(&s,6);JBH_Tick(&s,12);
    CHECK(command_count==3&&commands[0]==1&&commands[1]==7&&commands[2]==0x20);
    JBH_Tick(&s,196);CHECK(!bh_reads&&!s.light_id);
    JBH_Tick(&s,197);CHECK(bh_reads==1&&s.light_id==1&&s.light_ok&&J_Abs(s.lux-400)<0.01f);
    JBH_Tick(&s,197);JBH_Tick(&s,198);CHECK(bh_reads==1&&s.light_id==1);
    JBH_Tick(&s,212);JBH_Tick(&s,397);CHECK(s.light_id==2&&bh_reads==2);
}
static void bh_saturation_io_failure_and_retry(void)
{
    JSensors s={0};fixture();JBH_Init();JBH_Tick(&s,0);JBH_Tick(&s,6);JBH_Tick(&s,12);
    bh_raw=65535;JBH_Tick(&s,197);CHECK(!s.light_ok&&!s.light_id);
    JBH_Tick(&s,696);CHECK(command_count==3);JBH_Tick(&s,697);CHECK(command_count==4);
    bh_bad=1;JBH_Tick(&s,703);CHECK(!s.light_ok);
}
static void bh_sample_period_survives_zero_timestamp_wrap(void)
{
    JSensors s={0};fixture();JBH_Init();JBH_Tick(&s,0xFFFFFFF4U);JBH_Tick(&s,0xFFFFFFFAU);JBH_Tick(&s,0);
    CHECK(command_count==3);JBH_Tick(&s,185);CHECK(s.light_id==1);
    JBH_Tick(&s,186);CHECK(command_count==3);JBH_Tick(&s,199);CHECK(command_count==3);
    JBH_Tick(&s,200);CHECK(command_count==4);
}
static void separate_light_bus_failure_keeps_range_and_imu_valid(void)
{
    unsigned k;fixture();CHECK(bh_bus_inits==1);bh_bad=1;
    for(k=0;k<12;++k) { SysTick_count+=6;JPlatform_ControlTick(0.006f,1); }
    CHECK(!j_sensor.value.light_ok&&j_sensor.value.range_ok&&j_sensor.value.imu_ok);
    CHECK(range_reads==12&&j_sensor.value.height_mm==800&&j_mpu_sample_ok);
}
static void imu_failed_transaction_does_not_publish_garbage(void)
{
    _st_Mpu before;uint32_t stamp;fixture();MpuGetData();CHECK(j_mpu_sample_ok&&MPU6050.accZ==8192);
    before=MPU6050;stamp=j_mpu_last_ms;imu_bad=1;SysTick_count+=3;MpuGetData();
    CHECK(!j_mpu_sample_ok&&memcmp(&before,&MPU6050,sizeof(before))==0&&j_mpu_last_ms==stamp);
}
static void whole_bus_failure_stops_without_laser_handover(void)
{
    JControlPacket c=control();unsigned k;fixture();deliver(&c);JPlatform_ControlTick(0.003f,1);
    j_flight.armed=j_flight.motors_on=1;j_flight.mode=J_C3_AUTO;j_flight.phase=J_HOVER;
    j_flight.height_target_mm=j_flight.goal_mm=800;j_flight.common=500;
    imu_bad=lc_bad=bh_bad=1;
    for(k=0;k<12;++k) { SysTick_count+=3;JPlatform_ControlTick(0.003f,1); }
    CHECK(!j_flight.armed&&j_flight.fault==J_ERR_IMU&&TIM2->CCR1==0);
}
static void pid_limits_antiwindup_and_derivative_measurement(void)
{
    PidObject p;unsigned k;limits(&p,100,50);p.kp=3;p.ki=1;p.kd=5;p.desired=1000;
    for(k=0;k<1000;++k)pidUpdate(&p,0.003f);
    CHECK(p.out==100&&p.integ==0);
    limits(&p,100,50);p.kp=3;p.kd=5;p.desired=10;pidUpdate(&p,0.003f);CHECK(p.out==30);
    p.desired=20;pidUpdate(&p,0.003f);CHECK(p.out==60); /* no derivative kick on target switch */
}
static void actual_systick_and_scheduler_have_one_clock_owner(void)
{
    unsigned k;fixture();SysTick_count=0;j_flight.armed=0;
    for(k=0;k<3000;++k)SysTick_Handler();CHECK(SysTick_count==3000);
    for(k=0;k<4;++k){ uint32_t before=SysTick_count;tim_pending=1;TIM1_UP_IRQHandler();CHECK(SysTick_count==before);SysTick_count+=3; }
    CHECK(range_reads==2); /* module I2C every 6ms; no independent foreground bus owner */
}
static void actual_deadline_handler_clears_pwm(void)
{
    unsigned k;fixture();SysTick_count=0;j_last_motor_ms=0;j_flight.armed=j_flight.motors_on=1;
    TIM2->CCR1=TIM2->CCR2=TIM2->CCR3=TIM2->CCR4=500;
    for(k=0;k<26;++k)SysTick_Handler();CHECK(j_deadline_fault&&TIM2->CCR1==0&&TIM2->CCR4==0);
    MotorControl();CHECK(TIM2->CCR1==0);JPlatform_ControlTick(0.003f,0);
    CHECK(!j_flight.armed&&j_flight.fault==J_ERR_DEADLINE);
}
static void telemetry_turnaround_delay_not_unsigned_underflow(void)
{
    JControlPacket c=control();JStatusPacket s;fixture();deliver(&c);JPlatform_RadioPoll();CHECK(!reply_count);
    SysTick_count+=1;JPlatform_RadioPoll();CHECK(!reply_count);
    SysTick_count+=1;JPlatform_RadioPoll();CHECK(reply_count==1&&JStatus_Decode(reply,&s));
    CHECK(s.control_session==123&&s.echo_seq==1);
}
int main(void)
{
    RUN(measured_millimetre_frames_and_airborne_c1);
    RUN(pwm_mixer_and_lock);RUN(c2_real_parser_clears_pwm_before_control_tick);
    RUN(actual_led_channel_mapping_and_low_polarity);RUN(real_lc08_driver_units_and_no_silent_mode_exit);
    RUN(real_bh_driver_distinct_one_shot_samples);RUN(bh_saturation_io_failure_and_retry);
    RUN(bh_sample_period_survives_zero_timestamp_wrap);
    RUN(separate_light_bus_failure_keeps_range_and_imu_valid);
    RUN(imu_failed_transaction_does_not_publish_garbage);RUN(whole_bus_failure_stops_without_laser_handover);
    RUN(pid_limits_antiwindup_and_derivative_measurement);RUN(actual_systick_and_scheduler_have_one_clock_owner);
    RUN(actual_deadline_handler_clears_pwm);RUN(telemetry_turnaround_delay_not_unsigned_underflow);
    printf("RESULT cases=%u checks=%u\n",cases,checks);return 0;
}
