/* Attitude cascade and original motor mixer retained; JCore owns collective/mode. */
#include "ALL_DEFINE.h"
#include "pid.h"
#include "j_platform.h"
#include "flow.h"
static PidObject *attitude[6]={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw};
float sins_high,sins_vel;
uint16_t thr_hold,low_thr_cnt;
void FlightPidControl(float dt)
{
    static uint8_t was_armed;
    static float yaw_command;
    float yaw_difference;
    uint8_t k;
    if(!j_flight.armed || !j_flight.motors_on) {
        pidRest(attitude,6); was_armed=0; return;
    }
    pidRoll.desired=j_flight.roll_cmd; pidPitch.desired=j_flight.pitch_cmd;
    if(!was_armed) yaw_command=j_flight.yaw_target;
    yaw_difference=j_flight.yaw_target-yaw_command;
    yaw_command+=J_Clamp(yaw_difference,-J_YAW_TARGET_SLEW_DEG_S*dt,J_YAW_TARGET_SLEW_DEG_S*dt);
    pidYaw.desired=yaw_command;
    pidRoll.measured=Angle.roll; pidPitch.measured=Angle.pitch; pidYaw.measured=Angle.yaw;
    pidRateX.measured=MPU6050.gyroX*Gyro_G;
    pidRateY.measured=MPU6050.gyroY*Gyro_G;
    pidRateZ.measured=MPU6050.gyroZ*Gyro_G;
    if(!was_armed) {
        for(k=0;k<6U;++k) { attitude[k]->prevMeasured=attitude[k]->measured; attitude[k]->initialized=1; }
        was_armed=1;
    }
    CascadePID(&pidRateX,&pidRoll,dt);
    CascadePID(&pidRateY,&pidPitch,dt);
    CascadePID(&pidRateZ,&pidYaw,dt);
}
void MotorControl(void)
{
    float m[4],common;
    uint8_t k;
    uint32_t mask;
    if(!j_flight.armed || !j_flight.motors_on || j_deadline_fault) {
        low_thr_cnt=0; JPlatform_ZeroMotors(); return;
    }
    common=(float)(int16_t)J_Clamp(j_flight.common,0.0f,J_THR_AUTO_MAX);
    m[0]=common+pidRateX.out+pidRateY.out+pidRateZ.out;
    m[1]=common+pidRateX.out-pidRateY.out-pidRateZ.out;
    m[2]=common-pidRateX.out+pidRateY.out-pidRateZ.out;
    m[3]=common-pidRateX.out-pidRateY.out+pidRateZ.out;
    for(k=0;k<4U;++k) {
        if(m[k]!=m[k]) { JCore_Stop(&j_flight,J_ERR_IMU,SysTick_count); JPlatform_ZeroMotors(); return; }
        motor_PWM_Value[k]=(int16_t)J_Clamp(m[k],0.0f,1000.0f);
    }
    /* Atomic publication: a preempting safety ISR cannot clear PWM and then be undone. */
    mask=__get_PRIMASK(); __disable_irq();
    if(j_deadline_fault || !j_flight.armed) { JPlatform_ZeroMotors(); __set_PRIMASK(mask); return; }
    TIM2->CCR1=motor_PWM_Value[0]; TIM2->CCR2=motor_PWM_Value[1];
    TIM2->CCR3=motor_PWM_Value[2]; TIM2->CCR4=motor_PWM_Value[3];
    __set_PRIMASK(mask);
}
