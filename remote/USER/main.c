#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "remote.h"
static void clock_init(void)
{
    RCC_DeInit(); RCC_HSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)==RESET) {}
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1); RCC_PCLK1Config(RCC_HCLK_Div2); RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_12); RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET) {}
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource()!=0x08U) {}
    SystemCoreClock=48000000U;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|
        RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
}
int main(void)
{
    GPIO_InitTypeDef g;
    clock_init(); cycleCounterInit();
    SysTick_Config(48000000U/1000U);
    g.GPIO_Pin=GPIO_Pin_12; g.GPIO_Mode=GPIO_Mode_Out_PP; g.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&g); GPIOA->BRR=GPIO_Pin_12;
    _g_Init_sys();
    for(;;) RC_Analy();
}
