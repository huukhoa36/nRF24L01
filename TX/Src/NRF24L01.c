#include "NRF24L01.h"

unsigned char  TX_ADDRESS[TX_ADDR_WIDTH]= {0x68,0x31,0x08,0x10,0x01};
unsigned char  RX_ADDRESS[RX_ADDR_WIDTH]= {0x68,0x31,0x08,0x10,0x01};

uint8_t NRF24_SPIWrite(uint8_t Buff)
{
	uint8_t index = 0;
	for(index = 0; index < 8; index++)
	{
		if( (Buff&0x80) == 0x80)
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_RESET);
		
		Buff =  Buff<<1;
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_SET);

		Buff |= HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port,NRF_MISO_Pin);
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);		
	}
	return (Buff);
}

unsigned char CTR_spiRead(unsigned char reg)
{
	uint8_t reg_value=0;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);	
	NRF24_SPIWrite(reg);
	reg_value = NRF24_SPIWrite(0x00);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return reg_value;
}

uint8_t NRF24_WriteReg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	
	status = NRF24_SPIWrite(reg);
	NRF24_SPIWrite(value);
	
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
	
	return status;
}

unsigned char CTR_spiReadBuff(unsigned char reg, unsigned char *buff, unsigned char uchars)
{
	uint8_t status;
	uint8_t index = 0;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	status = NRF24_SPIWrite(reg);
	for(index = 0; index < uchars; index++)
	{
		buff[index] = NRF24_SPIWrite(0x00);
	}
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return status;
}

uint8_t NRF24_WriteBuff(uint8_t reg, uint8_t *buff, uint8_t length)
{
	uint8_t status;
	uint8_t index = 0;
	
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	
	status = NRF24_SPIWrite(reg);
	for(index = 0; index < length; index++)
	{
		NRF24_SPIWrite(*buff++);
	}
	
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
	
	return status;
}

void NRF24_Init(void)
{
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);
	
	NRF24_WriteBuff( (W_REGISTER | TX_ADDR) , TX_ADDRESS, TX_ADDR_WIDTH); //Transmit address
  NRF24_WriteBuff( (W_REGISTER | RX_ADDR_P0) , RX_ADDRESS, RX_ADDR_WIDTH); //Receive 5 bytes address data
	NRF24_WriteReg(W_REGISTER + CONFIG, 0); //config later in TX and RX mode
  NRF24_WriteReg(W_REGISTER + EN_AA, 0x01); //Enable auto acknowledgement data pipe 0   
  NRF24_WriteReg(W_REGISTER + EN_RXADDR, 0x01); //Enable data pipe 0.
  NRF24_WriteReg(W_REGISTER + RF_CH, 0); //wil be set up later
  NRF24_WriteReg(W_REGISTER + RX_PW_P0, R_RX_PAYLOAD_WIDTH); //RX payload: 32bytes
  NRF24_WriteReg(W_REGISTER + RF_SETUP, 0x07); //Power TX mode: 0dBm, Data rate: 1Mbps
}

void CTR_nrfSetRX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	NRF24_WriteReg(W_REGISTER + CONFIG, 0x0f);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
	HAL_Delay(130);
}

void NRF24_SetTX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	
	NRF24_WriteReg(W_REGISTER + CONFIG, 0x0E); //Enable CRC, Power up
	
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
	HAL_Delay(130);
}

unsigned char CTR_nrfGetPacket(unsigned char* rx_buf)
{
	uint8_t value=0,sta=0;
	sta = CTR_spiRead(CTR_STATUS);
	if((sta&0x40) != 0)
	{
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
		CTR_spiReadBuff(R_RX_PAYLOAD, rx_buf,W_TX_PAYLOAD_WIDTH);
		value = 1;
	}
	NRF24_WriteReg(W_REGISTER+CTR_STATUS,sta);
	return value;
}

void NRF24_Transmit(uint8_t *data)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET); // CE select
	
	NRF24_WriteBuff( (W_REGISTER | RX_ADDR_P0) , TX_ADDRESS, RX_ADDR_WIDTH);
	NRF24_WriteBuff(W_TX_PAYLOAD, data, W_TX_PAYLOAD_WIDTH); //send the payload
	NRF24_WriteReg( (W_REGISTER | CONFIG) , 0x0E); //Enable CRC, Power up
	
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET); // CE unselect
}


