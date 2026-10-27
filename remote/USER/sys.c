#include "sys.h"
#include "ADC_DMA_Config.h"
#include "I2C.h"
#include "remote.h"
#include "nrf24l01.h"
volatile uint32_t SysTick_count;
void _g_Init_sys(void)
{
    IIC_Init(); ADC1_GPIO_Config(); ADC1_Mode_Config();
    NRF24L01_init(); RC_INIT();
}
void SysTick_Handler(void) { ++SysTick_count; } /* never wait for radio IRQ inside SysTick */
