#ifndef J_NATIVE_STM32_H
#define J_NATIVE_STM32_H
#include <stdint.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
typedef struct { uint16_t ODR; } GPIO_TypeDef;
typedef struct { uint16_t GPIO_Pin; uint8_t GPIO_Speed,GPIO_Mode; } GPIO_InitTypeDef;
extern GPIO_TypeDef mock_gpioa,mock_gpiob;
#define GPIOA (&mock_gpioa)
#define GPIOB (&mock_gpiob)
#define GPIO_Pin_9 0x0200U
#define GPIO_Pin_10 0x0400U
#define RCC_APB2Periph_GPIOA 4U
#define GPIO_Mode_Out_OD 0x14U
#define GPIO_Speed_50MHz 3U
#define Bit_RESET 0U
void RCC_APB2PeriphClockCmd(uint32_t clock,int enable);
void GPIO_Init(GPIO_TypeDef *gpio,GPIO_InitTypeDef *config);
void GPIO_SetBits(GPIO_TypeDef *gpio,uint16_t pins);
void GPIO_ResetBits(GPIO_TypeDef *gpio,uint16_t pins);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *gpio,uint16_t pin);
typedef struct { uint32_t CCR1,CCR2,CCR3,CCR4; } JMockTimer;
typedef struct { uint32_t LOAD,VAL; } JMockSysTick;
extern JMockTimer mock_tim1,mock_tim2,mock_tim3;
extern JMockSysTick mock_systick;
extern uint32_t SystemCoreClock,mock_primask;
#define TIM1 (&mock_tim1)
#define TIM2 (&mock_tim2)
#define TIM3 (&mock_tim3)
#define SysTick (&mock_systick)
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define TIM_IT_Update 1
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask=1; }
static inline void __set_PRIMASK(uint32_t x) { mock_primask=x; }
int TIM_GetITStatus(JMockTimer *t,int bit);
void TIM_ClearITPendingBit(JMockTimer *t,int bit);
void TIM_ITConfig(JMockTimer *t,int bit,int enable);
int SysTick_Config(uint32_t ticks);
#endif
