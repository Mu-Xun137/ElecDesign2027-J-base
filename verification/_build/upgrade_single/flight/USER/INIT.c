#include "ALL_DEFINE.h"
#include "j_platform.h"
volatile uint32_t SysTick_count;
_st_Mpu MPU6050;
_st_AngE Angle;
_st_Remote Remote;
_st_ALL_flag ALL_flag;
PidObject pidRateX,pidRateY,pidRateZ,pidPitch,pidRoll,pidYaw;
PidObject pidHeightRate,pidHeightHigh,pidPositionX,pidPositionY,pidPosRateX,pidPosRateY;
int16_t motor_PWM_Value[4];
static void limits(PidObject *p,float out,float integral)
{
    p->OutLimitLow=-out; p->OutLimitHigh=out;
    p->IntegLimitLow=-integral; p->IntegLimitHigh=integral;
}
void pid_param_Init(void)
{
    pidRateX.kp=pidRateY.kp=J_RATE_XY_KP; pidRateZ.kp=J_RATE_YAW_KP;
    pidRateX.ki=pidRateY.ki=J_RATE_XY_KI; pidRateZ.ki=J_RATE_YAW_KI;
    pidRateX.kd=pidRateY.kd=J_RATE_XY_KD; pidRateZ.kd=J_RATE_YAW_KD;
    pidPitch.kp=pidRoll.kp=J_ANGLE_XY_KP; pidYaw.kp=J_ANGLE_YAW_KP;
    limits(&pidRateX,J_RATE_XY_PWM_LIMIT,J_RATE_XY_I_LIMIT); limits(&pidRateY,J_RATE_XY_PWM_LIMIT,J_RATE_XY_I_LIMIT);
    limits(&pidRateZ,J_RATE_YAW_PWM_LIMIT,J_RATE_YAW_I_LIMIT); limits(&pidPitch,J_ANGLE_XY_RATE_LIMIT,0.0f);
    limits(&pidRoll,J_ANGLE_XY_RATE_LIMIT,0.0f); limits(&pidYaw,J_ANGLE_YAW_RATE_LIMIT,0.0f);
}
void ALL_Init(void)
{
    TIM2_PWM_Config(); TIM3_PWM_Config(); JPlatform_ZeroMotors();
    IIC_Init(); pid_param_Init();
    MpuInit(); /* bounded attempts; failure is reported and blocks arming */
    NRF24L01_init();
    JPlatform_Init();
    TIM1_Config();
}
