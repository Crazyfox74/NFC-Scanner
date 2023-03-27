/*
 * spi.h
 *
 *  Created on: 24 мар. 2023 г.
 *      Author: crazy
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f10x.h"

typedef enum
{
	SPI_OK,
	SPI_ERROR
} SPI_Status_t;

#define SPI_EMPTY_BYTE					0xff

void SPI_Config(SPI_TypeDef *SPIx, uint8_t CS_Pin, GPIO_TypeDef *CS_Port);
void SPI_Set_Speed(SPI_TypeDef *SPIx, uint32_t clk);

void SPI_CS_Set(GPIO_TypeDef *port, uint8_t pin);
void SPI_CS_Reset(GPIO_TypeDef *port, uint8_t pin);


SPI_Status_t SPI_Send8Data(SPI_TypeDef * SPIx, uint8_t * data, uint32_t len);
SPI_Status_t SPI_Receive8Data(SPI_TypeDef * SPIx, uint8_t * data, uint32_t len);
SPI_Status_t SPI_Send16Data(SPI_TypeDef * SPIx, uint16_t * data, uint32_t len);
SPI_Status_t SPI_Receive16Data(SPI_TypeDef * SPIx, uint16_t * data, uint32_t len);

void SPI_Send_Recv(uint8_t *buf_tx, uint8_t buf_rx, uint16_t len);
void SpiSendRecvFlash(uint8_t **a_buf_tx, uint8_t **a_buf_rx, uint16_t *a_buf_len, uint8_t cnt);

#endif /* SPI_H_ */
