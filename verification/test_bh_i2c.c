/* Run the REAL PA9/PA10 bit-bang driver against an open-drain I2C slave.
 * Decode pins/edges/bytes and inject NACKs, stuck lines and clock stretching. */
#include "stm32f10x.h"
#include "j_bh_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SCL GPIO_Pin_9
#define SDA GPIO_Pin_10
GPIO_TypeDef mock_gpioa,mock_gpiob;
static unsigned cases,checks,starts,stops,edges,reads,clocks,stretch;
static uint8_t active,scl_level,slave_low,stuck_scl,stuck_sda,configured,read_ended;
static uint8_t tx[4],incoming[2],master_ack[2],shift;
static int nack_byte;
static const char *current;
#define CHECK(x) do { ++checks; if(!(x)) { printf("FAIL %s:%d: %s\n",current,__LINE__,#x);exit(2); } } while(0)
#define RUN(f) do { current=#f;f();++cases;printf("PASS %s\n",current); } while(0)

static void rising(void)
{
    unsigned byte,bit;
    if(!active) return;
    byte=edges/9U;bit=edges%9U;
    slave_low=0;
    if(read_ended) { ++edges;return; }
    if(byte==0U || (tx[0]&1U)==0U) {
        if(byte<2U) {
            if(bit<8U) shift=(uint8_t)((shift<<1)|((mock_gpioa.ODR&SDA)?1U:0U));
            else {
                CHECK(mock_gpioa.ODR&SDA); /* master releases SDA for slave ACK */
                tx[byte]=shift;shift=0;
                slave_low=(int)byte!=nack_byte;
                read_ended=!slave_low;
            }
        }
    } else if(byte<=2U && !read_ended) {
        if(bit<8U) {
            CHECK(mock_gpioa.ODR&SDA); /* master must release all receive bits */
            slave_low=(incoming[byte-1U]&(0x80U>>bit))==0U;
        } else {
            master_ack[byte-1U]=(mock_gpioa.ODR&SDA)?1U:0U;
            read_ended=master_ack[byte-1U];
        }
    }
    ++edges;
}
static void update_clock(void)
{
    uint8_t high=(mock_gpioa.ODR&SCL)&&!stuck_scl&&!stretch;
    if(high&&!scl_level) rising();
    if(!high) slave_low=0;
    scl_level=high;
}
void RCC_APB2PeriphClockCmd(uint32_t clock,int enable)
{ CHECK(clock==RCC_APB2Periph_GPIOA&&enable==ENABLE);++clocks; }
void GPIO_Init(GPIO_TypeDef *gpio,GPIO_InitTypeDef *g)
{
    CHECK(gpio==GPIOA&&g->GPIO_Pin==(SCL|SDA));
    CHECK(g->GPIO_Mode==GPIO_Mode_Out_OD&&g->GPIO_Speed==GPIO_Speed_50MHz);
    CHECK((gpio->ODR&(SCL|SDA))==(SCL|SDA)); /* released before output enable */
    configured=1;
}
static void set_pins(GPIO_TypeDef *gpio,uint16_t pins,uint8_t high)
{
    uint16_t previous;
    CHECK(gpio==GPIOA&&pins!=0U&&(pins&~(SCL|SDA))==0U);
    previous=gpio->ODR;
    if(high) gpio->ODR|=pins;else gpio->ODR&=(uint16_t)~pins;
    update_clock();
    if((pins&SDA)&&scl_level) {
        if((previous&SDA)&&!(gpio->ODR&SDA)) {
            ++starts;active=1;edges=0;shift=0;slave_low=read_ended=0;
        } else if(!(previous&SDA)&&(gpio->ODR&SDA)&&active) {
            ++stops;active=0;slave_low=0;
        }
    }
}
void GPIO_SetBits(GPIO_TypeDef *gpio,uint16_t pins) { set_pins(gpio,pins,1); }
void GPIO_ResetBits(GPIO_TypeDef *gpio,uint16_t pins) { set_pins(gpio,pins,0); }
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *gpio,uint16_t pin)
{
    CHECK(gpio==GPIOA&&(pin==SCL||pin==SDA));
    CHECK(++reads<500U); /* a permanently low line must not hang the control ISR */
    if(pin==SCL) { if(stretch)--stretch;update_clock();return scl_level; }
    return (gpio->ODR&SDA)&&!slave_low&&!stuck_sda;
}
static void fixture(void)
{
    memset(&mock_gpioa,0,sizeof(mock_gpioa));mock_gpiob.ODR=0x5A5AU;
    starts=stops=edges=reads=clocks=stretch=0;
    active=scl_level=slave_low=stuck_scl=stuck_sda=configured=read_ended=0;
    memset(tx,0,sizeof(tx));memset(master_ack,0xFF,sizeof(master_ack));
    incoming[0]=0x01;incoming[1]=0xE0;nack_byte=-1;
    JBHI2C_Init();
}
static void released(void)
{
    CHECK((mock_gpioa.ODR&(SCL|SDA))==(SCL|SDA));
    CHECK(mock_gpiob.ODR==0x5A5AU);
}
static void correct_open_drain_pins_and_idle(void)
{
    fixture();CHECK(configured&&clocks==1&&!starts&&!j_bh_i2c_errors);released();
}
static void command_frame_address_ack_and_stop(void)
{
    fixture();CHECK(JBHI2C_SendCommand(0x20)==0);
    CHECK(starts==1&&stops==1&&edges==19); /* 18 data/ACK clocks + STOP setup */
    CHECK(tx[0]==0x46&&tx[1]==0x20&&!j_bh_i2c_errors);released();
}
static void receive_two_bytes_ack_then_final_nack(void)
{
    uint8_t b[2]={0};fixture();CHECK(JBHI2C_Receive(b,2)==0);
    CHECK(tx[0]==0x47&&b[0]==0x01&&b[1]==0xE0);
    CHECK(master_ack[0]==0&&master_ack[1]==1);
    CHECK(starts==1&&stops==1&&edges==28&&!j_bh_i2c_errors);released();
}
static void receive_single_byte_sends_nack(void)
{
    uint8_t b=0;fixture();CHECK(JBHI2C_Receive(&b,1)==0);
    CHECK(b==1&&master_ack[0]==1&&stops==1);released();
}
static void address_nack_stops_and_next_transaction_recovers(void)
{
    fixture();nack_byte=0;CHECK(JBHI2C_SendCommand(1)!=0);
    CHECK(j_bh_i2c_errors==1&&stops==1&&edges==10);released();
    nack_byte=-1;CHECK(JBHI2C_SendCommand(7)==0);
    CHECK(starts==2&&stops==2&&tx[1]==7&&j_bh_i2c_errors==1);released();
}
static void command_and_read_address_nacks_are_errors(void)
{
    uint8_t b[2]={0xAA,0xBB};fixture();nack_byte=1;
    CHECK(JBHI2C_SendCommand(0x20)!=0&&j_bh_i2c_errors==1&&stops==1);released();
    nack_byte=0;CHECK(JBHI2C_Receive(b,2)!=0);
    CHECK(b[0]==0xAA&&b[1]==0xBB&&j_bh_i2c_errors==2);released();
}
static void stuck_scl_is_bounded_and_releases_pins(void)
{
    fixture();stuck_scl=1;CHECK(JBHI2C_SendCommand(1)!=0);
    CHECK(reads<=50&&j_bh_i2c_errors==1);released();
    stuck_scl=0;CHECK(JBHI2C_SendCommand(1)==0);released();
}
static void stuck_sda_is_rejected_and_can_retry(void)
{
    fixture();stuck_sda=1;CHECK(JBHI2C_SendCommand(1)!=0);
    CHECK(j_bh_i2c_errors==1);released();
    stuck_sda=0;CHECK(JBHI2C_SendCommand(1)==0);released();
}
static void finite_clock_stretch_and_invalid_receive(void)
{
    uint8_t b;fixture();stretch=4;scl_level=0;
    CHECK(JBHI2C_SendCommand(0x20)==0&&!stretch&&!j_bh_i2c_errors);released();
    CHECK(JBHI2C_Receive(&b,0)!=0&&JBHI2C_Receive(0,2)!=0);
    CHECK(starts==1&&stops==1);released();
}
int main(void)
{
    RUN(correct_open_drain_pins_and_idle);RUN(command_frame_address_ack_and_stop);
    RUN(receive_two_bytes_ack_then_final_nack);RUN(receive_single_byte_sends_nack);
    RUN(address_nack_stops_and_next_transaction_recovers);RUN(command_and_read_address_nacks_are_errors);
    RUN(stuck_scl_is_bounded_and_releases_pins);RUN(stuck_sda_is_rejected_and_can_retry);
    RUN(finite_clock_stretch_and_invalid_receive);
    printf("RESULT cases=%u checks=%u\n",cases,checks);return 0;
}
