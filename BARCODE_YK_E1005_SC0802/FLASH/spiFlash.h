/*
 * spiFlash.h
 *
 *  Created on: 24 мар. 2023 г.
 *      Author: crazy
 */

#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#include "stm32f10x.h"

#define FLASH_RES_OK				0
#define FLASH_RES_ERROR_BUSY		1
#define FLASH_RES_ERROR_WRITE_ENABLE	2
#define FLASH_RES_ERROR_AGAIN		10

void spiFlash_Init(void);
void spiFlash_Deinit(void);
uint8_t spiFlash_readStatus(void);
void spiFlash_Read(uint32_t dwAddrRead, uint16_t bCnt, uint8_t *pBuf);
uint8_t spiFlash_write( uint32_t dwAddrWrite, uint16_t bCnt, uint8_t* pBuf );
uint8_t spiFlash_eraseSector( uint32_t dwSectAddr );
uint8_t spiFlash_wrtEnbl();

uint32_t spiFlash_readJEDECDesc( void );


#endif /* SPIFLASH_H_ */
