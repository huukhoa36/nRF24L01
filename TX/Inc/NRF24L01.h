#ifndef CTR_NRF
#define CTR_NRF 100

#include "stm32f1xx_hal.h"

// added
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_7
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_3
#define NRF_SCK_GPIO_Port GPIOA
#define NRF_MOSI_Pin GPIO_PIN_4
#define NRF_MOSI_GPIO_Port GPIOA
#define NRF_MISO_Pin GPIO_PIN_5
#define NRF_MISO_GPIO_Port GPIOA
//End added

#define TX_ADDR_WIDTH    5
#define RX_ADDR_WIDTH    5
#define W_TX_PAYLOAD_WIDTH  32
#define R_RX_PAYLOAD_WIDTH  32


#define R_REGISTER        0x00
#define W_REGISTER        0x20
#define R_RX_PAYLOAD      0x61
#define W_TX_PAYLOAD      0xA0
#define FLUSH_TX          0xE1
#define FLUSH_RX          0xE2
#define REUSE_TX_PL       0xE3
#define NOP               0xFF

#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define CTR_STATUS          0x07
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17  

uint8_t NRF24_SPIWrite(uint8_t Buff);
unsigned char CTR_spiRead(unsigned char reg);
uint8_t NRF24_WriteReg(uint8_t reg, uint8_t value);

unsigned char CTR_spiReadBuff(unsigned char reg, unsigned char *buff, unsigned char uchars);
uint8_t NRF24_WriteBuff(uint8_t reg, uint8_t *buff, uint8_t length);

void NRF24_Init(void);
void CTR_nrfSetRX(void);
void NRF24_SetTX(void);
unsigned char CTR_nrfGetPacket(unsigned char* rx_buf);
void NRF24_Transmit(uint8_t *data);

#endif

