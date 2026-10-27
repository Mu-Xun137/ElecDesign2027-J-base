/* Shared PB6/PB7 software I2C. After init, TIM1 is the ONLY bus owner.
 * Every ACK, repeated START and SCL stretch is checked. No unbounded waits. */
#include "I2C.h"
static uint8_t bus_busy, failed;
volatile uint32_t j_iic_errors;
static void bit_delay(void) { volatile unsigned k=14; while(k--) {} }
static void clock_high(void)
{
    uint8_t n=24;
    SCL_H;
    while(!(SCL_read) && n--) bit_delay();
    if(!(SCL_read)) failed=1;
    bit_delay();
}
static void stop(void)
{
    SCL_L; SDA_L; bit_delay(); clock_high(); SDA_H; bit_delay();
}
static uint8_t start(void)
{
    SDA_H; clock_high();
    if(!(SDA_read)) { failed=1; return 1; }
    SDA_L; bit_delay(); SCL_L; return failed;
}
static void send_byte(uint8_t x)
{
    uint8_t k;
    for(k=0;k<8U && !failed;++k) {
        SCL_L;
        if(x&128U) { SDA_H; } else { SDA_L; }
        bit_delay(); clock_high(); SCL_L; x<<=1;
    }
}
static uint8_t ack(void)
{
    uint8_t bad;
    SDA_H; bit_delay(); clock_high(); bad=(SDA_read)?1U:0U; SCL_L;
    if(bad) failed=1;
    return bad;
}
static uint8_t read_byte(uint8_t last)
{
    uint8_t k,x=0;
    SDA_H;
    for(k=0;k<8U && !failed;++k) {
        SCL_L; bit_delay(); clock_high(); x=(uint8_t)((x<<1)|((SDA_read)?1U:0U));
    }
    SCL_L;
    if(last) { SDA_H; } else { SDA_L; }
    bit_delay(); clock_high(); SCL_L; SDA_H;
    return x;
}
static uint8_t enter(void)
{
    if(bus_busy) { ++j_iic_errors; return 1; }
    bus_busy=1; failed=0;
    if(start()) { stop(); bus_busy=0; ++j_iic_errors; return 1; }
    return 0;
}
static int8_t leave(void)
{
    uint8_t bad;
    stop(); bad=failed; bus_busy=0;
    if(bad) ++j_iic_errors;
    return bad?1:0;
}
void IIC_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    g.GPIO_Pin=SCL_PIN|SDA_PIN; g.GPIO_Mode=GPIO_Mode_Out_OD; g.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&g); SDA_H; SCL_H;
}
int8_t IIC_Write_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    uint8_t k;
    if(enter()) return 1;
    send_byte(addr&0xFEU); ack();
    if(!failed) { send_byte(reg); ack(); }
    for(k=0;k<len && !failed;++k) { send_byte(data[k]); ack(); }
    return leave();
}
int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{ return IIC_Write_Bytes(addr,reg,&data,1); }
int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    uint8_t k;
    if(!len || enter()) return 1;
    send_byte(addr&0xFEU); ack();
    if(!failed) { send_byte(reg); ack(); }
    if(!failed) start(); /* repeated START; no extra address/register confusion */
    if(!failed) { send_byte(addr|1U); ack(); }
    for(k=0;k<len && !failed;++k) data[k]=read_byte(k+1U==len);
    return leave();
}
int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
{ uint8_t x; return IIC_read_Bytes(addr,reg,&x,1)?1:(int8_t)x; }
int8_t IIC_Send_Command(uint8_t addr,uint8_t cmd)
{
    if(enter()) return 1;
    send_byte(addr&0xFEU); ack();
    if(!failed) { send_byte(cmd); ack(); }
    return leave();
}
int8_t IIC_Receive_Bytes(uint8_t addr,uint8_t *data,uint8_t len)
{
    uint8_t k;
    if(!len || enter()) return 1;
    send_byte(addr|1U); ack();
    for(k=0;k<len && !failed;++k) data[k]=read_byte(k+1U==len);
    return leave();
}
