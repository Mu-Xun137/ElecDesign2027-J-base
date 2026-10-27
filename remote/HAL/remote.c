#include "stm32f10x.h"
#include "remote.h"
#include "ADC_DMA_Config.h"
#include "nrf24l01.h"
#include "eeprom.h"
#include "delay.h"
#include "j_protocol.h"
#include "j_keys.h"
#include "j_feedback.h"
#include "j_boot.h"
#include <string.h>
extern void JRadio_Poll(uint32_t us);
extern uint8_t JRadio_StartTx(uint8_t *p,uint32_t us);
extern volatile uint8_t j_radio_tx_busy;
_st_Remote Remote={1500,1500,1000,1500,1000,1000,1000,1000};
JFeedback j_remote_feedback;
static JKeys keys;
static uint32_t session,last_send_ms,last_sample_ms,last_buzzer_ms,last_tx_ms;
static uint16_t last_seq;
static int16_t roll_trim,pitch_trim;
static uint16_t filtered[4];
static uint8_t samples_started,trim_previous,have_tx;
void key_init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    g.GPIO_Mode=GPIO_Mode_IPU; g.GPIO_Speed=GPIO_Speed_50MHz;
    g.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15; GPIO_Init(GPIOA,&g);
    g.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11; GPIO_Init(GPIOB,&g);
    g.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_15; GPIO_Init(GPIOC,&g);
}
void RC_INIT(void)
{
    uint32_t entropy;
    key_init();
    roll_trim=read_AT24C02(ROLL_ADDR_OFFSET); pitch_trim=read_AT24C02(PITCH_ADDR_OFFSET);
    if(roll_trim<-250 || roll_trim>250) roll_trim=0;
    if(pitch_trim<-250 || pitch_trim>250) pitch_trim=0;
    entropy=((uint32_t)ADC_ConvertedValue[0]<<20)^((uint32_t)ADC_ConvertedValue[1]<<8)^ADC_ConvertedValue[2];
    session=JBoot_Session(entropy^GetSysTime_us());
    last_send_ms=SysTick_count; last_sample_ms=SysTick_count-10U;
}
static uint8_t physical_keys(void)
{
    uint8_t x=0;
    if(!(GPIOA->IDR&GPIO_Pin_15)) x|=1U;
    if(!(GPIOB->IDR&GPIO_Pin_11)) x|=2U; /* independent C2, even if all other keys are held */
    if(!(GPIOC->IDR&GPIO_Pin_14)) x|=4U;
    if(!(GPIOC->IDR&GPIO_Pin_15)) x|=8U;
    return x;
}
static uint16_t channel_limit(int32_t x) { return x<1000?1000U:(x>2000?2000U:(uint16_t)x); }
static void sample_controls(uint32_t now)
{
    int32_t raw[4];
    uint8_t k,event,trim=0,new_trim;
    raw[0]=1000+(ADC_ConvertedValue[3]/4)+roll_trim;
    raw[1]=1000+(ADC_ConvertedValue[2]/4)+pitch_trim;
    raw[2]=1000+(ADC_ConvertedValue[1]/5); /* original 0.20 ADC gain: max is 1819 */
    raw[3]=1000+(ADC_ConvertedValue[0]/4);
    for(k=0;k<4U;++k) {
        uint16_t x=channel_limit(raw[k]);
        if(!samples_started) filtered[k]=x;
        else filtered[k]=(uint16_t)(((uint32_t)x+3U*filtered[k])/4U);
    }
    samples_started=1;
    Remote.roll=filtered[0]; Remote.pitch=filtered[1]; Remote.thr=filtered[2]; Remote.yaw=filtered[3];
    event=JKeys_Step(&keys,physical_keys(),now);
    Remote.AUX1=(keys.stable&1U)?2000U:1000U; Remote.AUX2=(keys.stable&2U)?2000U:1000U;
    Remote.AUX3=(keys.stable&4U)?2000U:1000U; Remote.AUX4=(keys.stable&8U)?2000U:1000U;
    if(!(GPIOA->IDR&GPIO_Pin_6)) trim|=1U;
    if(!(GPIOA->IDR&GPIO_Pin_7)) trim|=2U;
    if(!(GPIOB->IDR&GPIO_Pin_0)) trim|=4U;
    if(!(GPIOB->IDR&GPIO_Pin_1)) trim|=8U;
    new_trim=trim&~trim_previous; trim_previous=trim;
    if(new_trim&1U) { if(roll_trim>-250) { roll_trim-=5; write_AT24C02(ROLL_ADDR_OFFSET,roll_trim); } }
    if(new_trim&2U) { if(roll_trim<250) { roll_trim+=5; write_AT24C02(ROLL_ADDR_OFFSET,roll_trim); } }
    if(new_trim&4U) { if(pitch_trim>-250) { pitch_trim-=5; write_AT24C02(PITCH_ADDR_OFFSET,pitch_trim); } }
    if(new_trim&8U) { if(pitch_trim<250) { pitch_trim+=5; write_AT24C02(PITCH_ADDR_OFFSET,pitch_trim); } }
    if(event || new_trim) JFeedback_Click(&j_remote_feedback,now);
}
void NRF_SEND(void)
{
    uint8_t p[32],k;
    JControlPacket c;
    memset(&c,0,sizeof(c));
    c.channel[0]=Remote.roll; c.channel[1]=Remote.pitch; c.channel[2]=Remote.thr; c.channel[3]=Remote.yaw;
    c.channel[4]=Remote.AUX1; c.channel[5]=Remote.AUX2; c.channel[6]=Remote.AUX3; c.channel[7]=Remote.AUX4;
    c.session=session; c.seq=(uint16_t)(last_seq+1U); c.pressed=keys.stable; c.reply=1;c.ready_keys=keys.validated;
    for(k=0;k<4U;++k) c.key_count[k]=keys.count[k];
    JControl_Encode(p,&c);
    if(JRadio_StartTx(p,GetSysTime_us())) { last_seq=c.seq;last_tx_ms=SysTick_count;have_tx=1; }
}
void RC_Analy(void)
{
    uint8_t p[32],k;
    uint32_t now=SysTick_count;
    JRadio_Poll(GetSysTime_us());
    for(k=0;k<3U;++k) {
        if(NRF24L01_RxPacket(p)) break;
        if(have_tx) JFeedback_Accept(&j_remote_feedback,p,now,session,last_seq,last_tx_ms);
    }
    if((uint32_t)(now-last_sample_ms)>=10U) { last_sample_ms=now; sample_controls(now); }
    if((uint32_t)(now-last_send_ms)>=J_RC_PERIOD_MS && !j_radio_tx_busy) { last_send_ms=now; NRF_SEND(); }
    if(now!=last_buzzer_ms) {
        last_buzzer_ms=now;
        if(JFeedback_Buzzer(&j_remote_feedback,now)) GPIOA->BSRR=GPIO_Pin_12;
        else GPIOA->BRR=GPIO_Pin_12;
    }
}
void key(void) { /* Runtime key sampling belongs to sample_controls, one scheduler only. */ }
