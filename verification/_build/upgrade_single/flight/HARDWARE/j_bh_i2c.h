#ifndef J_BH_I2C_H
#define J_BH_I2C_H
#include <stdint.h>

/* GY-302 on UART1 header: TX1/PA9 = SCL, RX1/PA10 = SDA.
 * ADDR must be grounded. This header is no longer available to USART1. */
#define J_BH_I2C_ADDRESS_7BIT 0x23U
extern volatile uint32_t j_bh_i2c_errors;
void JBHI2C_Init(void);
int8_t JBHI2C_SendCommand(uint8_t command);
int8_t JBHI2C_Receive(uint8_t *data,uint8_t len);
#endif
