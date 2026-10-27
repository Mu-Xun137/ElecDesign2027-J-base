/* Dedicated GY-302 bus. PB6/PB7 and their MPU6050/LC08 driver are untouched.
 * TIM1 is the only transaction owner after JBH_Init, before TIM1 is enabled.
 * High means release the open-drain pin; every SCL wait has a fixed bound. */
#include "j_bh_i2c.h"
#include "stm32f10x.h"

#define BH_SCL GPIO_Pin_9
#define BH_SDA GPIO_Pin_10
#define BH_STRETCH_POLLS 24U
static uint8_t busy,failed;
volatile uint32_t j_bh_i2c_errors;

static void bit_delay(void)
{
    volatile unsigned k=14;
    while(k--) {}
}
static uint8_t clock_high(void)
{
    uint8_t n=BH_STRETCH_POLLS;
    GPIO_SetBits(GPIOA,BH_SCL);
    while(GPIO_ReadInputDataBit(GPIOA,BH_SCL)==Bit_RESET) {
        if(--n==0U) { failed=1; return 1; }
        bit_delay();
    }
    bit_delay();
    return 0;
}
static void stop(void)
{
    GPIO_ResetBits(GPIOA,BH_SCL|BH_SDA);
    bit_delay();
    clock_high();
    GPIO_SetBits(GPIOA,BH_SDA);
    bit_delay();
}
static uint8_t start(void)
{
    GPIO_SetBits(GPIOA,BH_SDA);
    bit_delay();
    if(clock_high()) return 1;
    if(GPIO_ReadInputDataBit(GPIOA,BH_SDA)==Bit_RESET) { failed=1; return 1; }
    GPIO_ResetBits(GPIOA,BH_SDA);
    bit_delay();
    GPIO_ResetBits(GPIOA,BH_SCL);
    return 0;
}
static void send_byte(uint8_t value)
{
    uint8_t k;
    for(k=0;k<8U;++k) {
        if(value&0x80U) GPIO_SetBits(GPIOA,BH_SDA);
        else GPIO_ResetBits(GPIOA,BH_SDA);
        bit_delay();
        if(clock_high()) return;
        GPIO_ResetBits(GPIOA,BH_SCL);
        value<<=1;
    }
    GPIO_SetBits(GPIOA,BH_SDA);
    bit_delay();
    if(clock_high()) return;
    if(GPIO_ReadInputDataBit(GPIOA,BH_SDA)!=Bit_RESET) failed=1;
    GPIO_ResetBits(GPIOA,BH_SCL);
}
static uint8_t read_byte(uint8_t last)
{
    uint8_t k,value=0;
    GPIO_SetBits(GPIOA,BH_SDA);
    for(k=0;k<8U;++k) {
        bit_delay();
        if(clock_high()) return 0;
        value=(uint8_t)((value<<1)|GPIO_ReadInputDataBit(GPIOA,BH_SDA));
        GPIO_ResetBits(GPIOA,BH_SCL);
    }
    if(last) GPIO_SetBits(GPIOA,BH_SDA);
    else GPIO_ResetBits(GPIOA,BH_SDA);
    bit_delay();
    clock_high();
    GPIO_ResetBits(GPIOA,BH_SCL);
    GPIO_SetBits(GPIOA,BH_SDA);
    return value;
}
static uint8_t enter(void)
{
    if(busy) { ++j_bh_i2c_errors; return 1; }
    busy=1; failed=0;
    if(start()) {
        stop(); busy=0; ++j_bh_i2c_errors;
        return 1;
    }
    return 0;
}
static int8_t leave(void)
{
    stop(); busy=0;
    if(failed) { ++j_bh_i2c_errors; return 1; }
    return 0;
}
void JBHI2C_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    /* Set the output latch before enabling OD outputs to avoid a low glitch. */
    GPIO_SetBits(GPIOA,BH_SCL|BH_SDA);
    g.GPIO_Pin=BH_SCL|BH_SDA;
    g.GPIO_Mode=GPIO_Mode_Out_OD;
    g.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&g);
    busy=failed=0;
    j_bh_i2c_errors=0;
}
int8_t JBHI2C_SendCommand(uint8_t command)
{
    if(enter()) return 1;
    send_byte((uint8_t)(J_BH_I2C_ADDRESS_7BIT<<1));
    if(!failed) send_byte(command);
    return leave();
}
int8_t JBHI2C_Receive(uint8_t *data,uint8_t len)
{
    uint8_t k;
    if(!data || !len || enter()) return 1;
    send_byte((uint8_t)((J_BH_I2C_ADDRESS_7BIT<<1)|1U));
    for(k=0;k<len && !failed;++k) data[k]=read_byte(k+1U==len);
    return leave();
}
