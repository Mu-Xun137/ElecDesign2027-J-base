/* Baseline wiring/address/channel retained. All runtime operations are nonblocking. */
#include "nrf24l01.h"
#include "SPI.h"
#include "j_config.h"
const uint8_t TX_ADDRESS[5]={0xE1,0xE2,0xE3,0xE4,0xE5};
const uint8_t RX_ADDRESS[5]={0xE1,0xE2,0xE3,0xE4,0xE5};
#define CS_H (GPIOB->BSRR=GPIO_Pin_12)
#define CS_L (GPIOB->BRR=GPIO_Pin_12)
#ifdef J_REMOTE
#define CE_H (GPIOA->BSRR=GPIO_Pin_11)
#define CE_L (GPIOA->BRR=GPIO_Pin_11)
#else
#define CE_H (GPIOB->BSRR=GPIO_Pin_8)
#define CE_L (GPIOB->BRR=GPIO_Pin_8)
#endif
static uint32_t tx_start_us;
volatile uint8_t j_radio_ready,j_radio_tx_busy;
volatile uint32_t j_radio_timeouts;
void NRF24L01_Configuration(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
    g.GPIO_Pin=GPIO_Pin_12; g.GPIO_Mode=GPIO_Mode_Out_PP; g.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&g);
#ifdef J_REMOTE
    g.GPIO_Pin=GPIO_Pin_11; GPIO_Init(GPIOA,&g);
#else
    g.GPIO_Pin=GPIO_Pin_8; GPIO_Init(GPIOB,&g);
#endif
    g.GPIO_Pin=GPIO_Pin_8; g.GPIO_Mode=GPIO_Mode_IPU; GPIO_Init(GPIOA,&g);
    CE_L; CS_H; SPI_Config();
}
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{ u8 status; CS_L; status=SPI_RW(reg); SPI_RW(value); CS_H; return status; }
u8 NRF24L01_Read_Reg(u8 reg)
{ u8 value; CS_L; SPI_RW(reg); value=SPI_RW(0xFF); CS_H; return value; }
u8 NRF24L01_Write_Buf(u8 reg,u8 *p,u8 len)
{ u8 status; CS_L; status=SPI_RW(reg); while(len--) SPI_RW(*p++); CS_H; return status; }
u8 NRF24L01_Read_Buf(u8 reg,u8 *p,u8 len)
{ u8 status; CS_L; status=SPI_RW(reg); while(len--) *p++=SPI_RW(0xFF); CS_H; return status; }
u8 NRF24L01_Check(void)
{
    u8 b[5],k,test[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
    NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,test,5);
    NRF24L01_Read_Buf(TX_ADDR,b,5);
    for(k=0;k<5U;++k) if(b[k]!=test[k]) return 1;
    return 0;
}
void RX_Mode(void)
{ CE_L; NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0F); CE_H; }
void TX_Mode(void)
{ CE_L; NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0E); }
void NRF24L01_init(void)
{
    uint8_t k;
    NRF24L01_Configuration(); j_radio_ready=0; j_radio_tx_busy=0;
    for(k=0;k<3U;++k) if(!NRF24L01_Check()) { j_radio_ready=1; break; }
    NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8 *)TX_ADDRESS,5);
    NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8 *)RX_ADDRESS,5);
    NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0); /* no hardware ACK; explicit telemetry slots */
    NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,1);
    NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_AW,3);
    NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0);
    NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,1);
    NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x07);
    NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,32);
    NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x70);
    NRF24L01_Write_Reg(FLUSH_RX,0xFF); NRF24L01_Write_Reg(FLUSH_TX,0xFF);
    RX_Mode();
}
uint8_t JRadio_StartTx(uint8_t *p,uint32_t now_us)
{
    if(!j_radio_ready || j_radio_tx_busy) return 0;
    TX_Mode(); NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x30);
    NRF24L01_Write_Reg(FLUSH_TX,0xFF);
    NRF24L01_Write_Buf(WR_TX_PLOAD,p,32);
    tx_start_us=now_us; j_radio_tx_busy=1; CE_H;
    return 1;
}
void JRadio_Poll(uint32_t now_us)
{
    uint8_t status;
    if(!j_radio_tx_busy) return;
    status=NRF24L01_Read_Reg(STATUS);
    if((status&0x30U) || (uint32_t)(now_us-tx_start_us)>=J_RADIO_TX_TIMEOUT_US) {
        CE_L;
        if(!(status&0x20U)) { ++j_radio_timeouts; NRF24L01_Write_Reg(FLUSH_TX,0xFF); }
        NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x30);
        j_radio_tx_busy=0; RX_Mode();
    }
}
u8 NRF24L01_RxPacket(u8 *p)
{
    if(!j_radio_ready || j_radio_tx_busy || (NRF24L01_Read_Reg(FIFO_STATUS)&1U)) return 1;
    NRF24L01_Read_Buf(RD_RX_PLOAD,p,32);
    NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x40);
    return 0; /* do not flush other queued control packets */
}
/* Legacy entry now starts a transfer. Runtime callers use the explicit poll API. */
u8 NRF24L01_TxPacket(u8 *p)
{ extern uint32_t GetSysTime_us(void); return JRadio_StartTx(p,GetSysTime_us())?0U:1U; }
