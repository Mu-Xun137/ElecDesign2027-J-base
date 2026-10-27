#include "stm32f10x.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "j_boot.h"
uint32_t JBoot_Session(uint32_t entropy)
{
    uint32_t count,id;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    count=((uint32_t)BKP_ReadBackupRegister(BKP_DR2)<<16)|BKP_ReadBackupRegister(BKP_DR1);
    ++count;
    BKP_WriteBackupRegister(BKP_DR1,(uint16_t)count);
    BKP_WriteBackupRegister(BKP_DR2,(uint16_t)(count>>16));
    id=*(const uint32_t *)0x1FFFF7E8U ^ *(const uint32_t *)0x1FFFF7ECU;
    id^=count*2654435761U ^ entropy ^ SysTick->VAL;
    id^=id>>16; id*=2246822519U; id^=id>>13;
    return id?id:1U; /* best-effort session identity, not a security/authentication token */
}
