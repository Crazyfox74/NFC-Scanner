/*
 * spi.c
 *
 *  Created on: 24 мар. 2023 г.
 *      Author: crazy
 */


#include "spi.h"


typedef enum{
	SPI_8BIT,
	SPI_16BIT
} SPI_Bit_t;


static uint8_t *pBufTx, *pBufRx;
static uint16_t usBufPosRx, usBufPosTx;
static uint16_t usBufCnt;


static void SD_SPI_SetDataSize(SPI_TypeDef * SPIx, SPI_Bit_t bitnum);

static void SPI_Set_SPI_RCC(SPI_TypeDef *SPIx);
static void SPI_Set_GPIO_RCC(GPIO_TypeDef *GPIO);

static void SPI_SetDataSize(SPI_TypeDef * SPIx, SPI_Bit_t bitnum)
{
	SPIx->CR1 &= ~(SPI_CR1_SPE);
	if(bitnum == SPI_8BIT){
		SPIx->CR1 &= ~(SPI_CR1_DFF);
	}
	else if(bitnum == SPI_16BIT){
		SPIx->CR1 |= SPI_CR1_DFF;
	}
	SPIx->CR1 |= SPI_CR1_SPE;
}



static void SPI_Set_SPI_RCC(SPI_TypeDef *SPIx)
{
	if(SPIx == SPI1){
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	}
	else if(SPIx == SPI2){
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
}

static void SPI_Set_GPIO_RCC(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA){
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	}
	else if(GPIOx == GPIOB){
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	}
	else if(GPIOx == GPIOC){
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}
	else if(GPIOx == GPIOD){
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	}
	else if(GPIOx == GPIOE){
		RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	}
}


void SPI_Config(SPI_TypeDef *SPIx, uint8_t CS_Pin, GPIO_TypeDef *CS_Port)
{

	SPI_Set_GPIO_RCC(CS_Port);

	CS_Port->CRL &= ~(GPIO_CRL_MODE0 << (CS_Pin << 2));
	CS_Port->CRL |= (GPIO_CRL_MODE0 << (CS_Pin << 2));
	CS_Port->CRL &= ~(GPIO_CRL_CNF0 << (CS_Pin << 2));



	SPI_Set_SPI_RCC(SPIx);

	SPIx->CR1 = 0;
	SPIx->CR2 = 0;

	SPIx->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_CPOL);
	SPIx->CR1 |= SPI_CR1_SPE;



}

void SPI_Set_Speed(SPI_TypeDef *SPIx, uint32_t clk)
{
	SPIx->CR1 &= ~(SPI_CR1_BR);
	if(clk >= 50000000)
		SPIx->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2);
	else if(clk >= 24000000 && clk < 50000000)
		SPIx->CR1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_2);
	else if(clk >= 12000000 && clk < 24000000)
		SPIx->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_2);
	else if(clk >= 6000000 && clk < 12000000)
		SPIx->CR1 |= (SPI_CR1_BR_2);
	else
		SPIx->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1);
}


void SPI_CS_Set(GPIO_TypeDef *port, uint8_t pin)
{
	port->BSRR = 1 << pin;
}


void SPI_CS_Reset(GPIO_TypeDef *port, uint8_t pin)
{
	port->BRR = 1 << pin;
}


SPI_Status_t SPI_Send8Data(SPI_TypeDef * SPIx, uint8_t * data, uint32_t len)
{
	SPI_SetDataSize(SPIx, SPI_8BIT);
	for(uint32_t i = 0; i < len; ++i )
	{
		while(!(SPIx->SR & SPI_SR_TXE));
		  *(__IO uint8_t*)&SPIx->DR = data[i];
		while(!(SPIx->SR & SPI_SR_RXNE));
		 *(__IO uint8_t*)&SPIx->DR;
	}
	while(SPIx->SR & SPI_SR_BSY){};
	return SPI_OK;
}

SPI_Status_t SPI_Receive8Data(SPI_TypeDef * SPIx, uint8_t * data, uint32_t len)
{
	SPI_SetDataSize(SPIx, SPI_8BIT);
	for(uint32_t i = 0; i < len; ++i )
	{
		while(!(SPIx->SR & SPI_SR_TXE));
		*(__IO uint8_t*)&SPIx->DR = 0xFF;
		while(!(SPIx->SR & SPI_SR_RXNE));
		data[i] = *(__IO uint8_t*)&SPIx->DR;
	}
	while(SPIx->SR & SPI_SR_BSY);
	return SPI_OK;
}


SPI_Status_t SPI_Send16Data(SPI_TypeDef * SPIx, uint16_t * data, uint32_t len)
{
	SPI_SetDataSize(SPIx, SPI_16BIT);
	for(uint32_t i = 0; i < len; ++i)
	{
		while(!(SPIx->SR & SPI_SR_TXE));
		uint16_t tmp = (data[i] << 8) | (data[i] >> 8);
		SPIx->DR = tmp;
		while(!(SPIx->SR & SPI_SR_RXNE));
		SPIx->DR;
	}
	while(SPIx->SR & SPI_SR_BSY);
	return SPI_OK;
}

SPI_Status_t SPI_Receive16Data(SPI_TypeDef * SPIx, uint16_t * data, uint32_t len)
{
	SPI_SetDataSize(SPIx, SPI_16BIT);
	for(uint32_t i = 0; i < len; ++i)
	{
		while(!(SPIx->SR & SPI_SR_TXE));
		SPIx->DR = 0xFFFF;
		while(!(SPIx->SR & SPI_SR_RXNE));
		uint16_t tmp = SPIx->DR;
		data[i] = (tmp >> 8) | (tmp << 8);
	}
	while((SPIx->SR & SPI_SR_BSY));
	return SPI_OK;
}


void SPI_Send_Recv(uint8_t *buf_tx, uint8_t buf_rx, uint16_t len)
{
	pBufTx = buf_tx;
	pBufRx = buf_rx;
	usBufPosRx = 0;
	usBufPosTx = 1;
	usBufCnt = len;

	while (!(SPI1->SR & SPI_SR_TXE)); //ожидание установки 1 в TXE(окончание передачи)
	if (pBufTx)
		*(uint8_t*)&SPI1->DR = *pBufTx;
	//_SPI->DR = 5;
	else
		*(uint8_t*)&SPI1->DR = SPI_EMPTY_BYTE;
}


void SpiSendRecvFlash(uint8_t **a_buf_tx, uint8_t **a_buf_rx, uint16_t *a_buf_len, uint8_t cnt)
{
			for (uint8_t i = 0; i < cnt; i++)
			{

				SPI_Send_Recv(a_buf_tx[i], a_buf_rx[i], a_buf_len[i]);

			//	while(SpiActive);
			}
}

