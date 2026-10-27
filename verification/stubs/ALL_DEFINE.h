#ifndef __ALL_DEFINE_H__
#define __ALL_DEFINE_H__
#include "stm32f10x.h"
#include "../../flight/USER/ALL_DATA.h"
#define Gyro_G 0.06103515625f
void MpuGetData(void);
void GetAngle(const _st_Mpu *mpu,_st_AngE *angle,float dt);
void FlightPidControl(float dt);
void MotorControl(void);
uint32_t GetSysTime_us(void);
void delay_ms(uint16_t ms);
void delay_us(unsigned us);
int8_t IIC_Write_One_Byte(uint8_t a,uint8_t r,uint8_t d);
int8_t IIC_read_Bytes(uint8_t a,uint8_t r,uint8_t *b,uint8_t n);
uint8_t NRF24L01_RxPacket(uint8_t *packet);
#include "control.h"
#endif
