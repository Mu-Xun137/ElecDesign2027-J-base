/* MPU configuration and filtering retain the specified baseline's ranges/gains.
 * Invalid transactions never publish uninitialized bytes. */
#include "ALL_DEFINE.h"
#include "kalman.h"
#include "flash.h"
#include <string.h>
int16_t MpuOffset[6];
volatile uint8_t j_mpu_ready,j_mpu_sample_ok;
volatile uint32_t j_mpu_last_ms;
int8_t mpu6050_rest(void)
{ return IIC_Write_One_Byte(0xD0,0x6B,0x80); }
int8_t MpuInit(void)
{
    uint8_t k,id=0,bad=0;
    j_mpu_ready=0;
    for(k=0;k<3U;++k) {
        bad=IIC_Write_One_Byte(0xD0,0x6B,0x80)!=0; delay_ms(30);
        bad|=IIC_Write_One_Byte(0xD0,0x19,0x02)!=0;
        bad|=IIC_Write_One_Byte(0xD0,0x6B,0x03)!=0;
        bad|=IIC_Write_One_Byte(0xD0,0x1A,0x03)!=0;
        bad|=IIC_Write_One_Byte(0xD0,0x1B,0x18)!=0;
        bad|=IIC_Write_One_Byte(0xD0,0x1C,0x09)!=0;
        bad|=IIC_read_Bytes(0xD0,0x75,&id,1)!=0;
        if(!bad && id==0x68U) break;
    }
    if(bad || id!=0x68U) return 1;
    FLASH_read(MpuOffset,6);
    for(k=0;k<6U;++k) if(MpuOffset[k]!=(int16_t)0xFFFF) break;
    if(k==6U) memset(MpuOffset,0,sizeof(MpuOffset));
    j_mpu_ready=1;
    return 0;
}
void MpuGetData(void)
{
    uint8_t b[14],k;
    int16_t raw[6];
    int16_t *p=(int16_t *)&MPU6050;
    static float gyro[3];
    static struct _1_ekf_filter ekf[3]={{0.02f,0,0,0,0.001f,0.543f},
        {0.02f,0,0,0,0.001f,0.543f},{0.02f,0,0,0,0.001f,0.543f}};
    j_mpu_sample_ok=0;
    if(!j_mpu_ready || IIC_read_Bytes(0xD0,0x3B,b,14)) return;
    for(k=0;k<6U;++k) {
        uint8_t offset=(uint8_t)(k<3U?2U*k:2U*k+2U); /* skip temperature */
        raw[k]=(int16_t)((uint16_t)b[offset]<<8|b[offset+1U]);
    }
    /* A floating/broken bus returning all-0/all-FF is not an IMU measurement. */
    if((raw[0]==0 && raw[1]==0 && raw[2]==0) ||
       (raw[0]==-1 && raw[1]==-1 && raw[2]==-1 && raw[3]==-1)) return;
    for(k=0;k<6U;++k) {
        float value=(float)raw[k]-(float)MpuOffset[k];
        if(k<3U) { kalman_1(&ekf[k],value); p[k]=(int16_t)ekf[k].out; }
        else { gyro[k-3U]+=0.15f*(value-gyro[k-3U]); p[k]=(int16_t)gyro[k-3U]; }
    }
    j_mpu_last_ms=SysTick_count;
    j_mpu_sample_ok=1;
}
void MpuGetOffset(void)
{
    /* Deliberate ground-only calibration. Never called by a mode/key event. */
    int32_t sum[6]={0};
    uint16_t n;
    uint8_t k;
    if(ALL_flag.unlock) return;
    TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
    memset(MpuOffset,0,sizeof(MpuOffset));
    for(n=0;n<256U;++n) {
        delay_ms(4); MpuGetData();
        if(!j_mpu_sample_ok) { TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); return; }
        for(k=0;k<6U;++k) sum[k]+=((int16_t *)&MPU6050)[k];
    }
    for(k=0;k<6U;++k) MpuOffset[k]=(int16_t)(sum[k]/256);
    MpuOffset[2]-=8192;
    FLASH_write(MpuOffset,6);
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
}
