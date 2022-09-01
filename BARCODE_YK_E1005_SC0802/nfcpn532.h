/*
 * nfcpn532.h
 *
 *  Created on: 14 апр. 2022 г.
 *      Author: User
 */
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <string.h>



//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_spi.h"
//#include "stm32f10x_exti.h"

#ifndef NFCPN532_H_
#define NFCPN532_H_


//#define DEBUG_PN532
//******************************************************************************
#define PN532_PREAMBLE (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

#define PN532_HOSTTOPN532 (0xD4)
#define PN532_PN532TOHOST (0xD5)

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_GETGENERALSTATUS (0x04)
#define PN532_COMMAND_READREGISTER (0x06)
#define PN532_COMMAND_WRITEREGISTER (0x08)
#define PN532_COMMAND_READGPIO (0x0C)
#define PN532_COMMAND_WRITEGPIO (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE (0x10)
#define PN532_COMMAND_SETPARAMETERS (0x12)
#define PN532_COMMAND_SAMCONFIGURATION (0x14)
#define PN532_COMMAND_POWERDOWN (0x16)
#define PN532_COMMAND_RFCONFIGURATION (0x32)
#define PN532_COMMAND_RFREGULATIONTEST (0x58)
#define PN532_COMMAND_INJUMPFORDEP (0x56)
#define PN532_COMMAND_INJUMPFORPSL (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
#define PN532_COMMAND_INATR (0x50)
#define PN532_COMMAND_INPSL (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU (0x42)
#define PN532_COMMAND_INDESELECT (0x44)
#define PN532_COMMAND_INRELEASE (0x52)
#define PN532_COMMAND_INSELECT (0x54)
#define PN532_COMMAND_INAUTOPOLL (0x60)
#define PN532_COMMAND_TGINITASTARGET (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES (0x92)
#define PN532_COMMAND_TGGETDATA (0x86)
#define PN532_COMMAND_TGSETDATA (0x8E)
#define PN532_COMMAND_TGSETMETADATA (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS (0x8A)

#define PN532_RESPONSE_INDATAEXCHANGE (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET (0x4B)

#define PN532_WAKEUP (0x55)

#define PN532_SPI_STATREAD (0x02)
#define PN532_SPI_DATAWRITE (0x01)
#define PN532_SPI_DATAREAD (0x03)
#define PN532_SPI_READY (0x01)

/*
#define PN532_I2C_ADDRESS (0x48 >> 1)
#define PN532_I2C_READBIT (0x01)
#define PN532_I2C_BUSY (0x00)
#define PN532_I2C_READY (0x01)
#define PN532_I2C_READYTIMEOUT (20)
*/
#define PN532_MIFARE_ISO14443A (0x00)

#define PN532_ACK_WAIT_TIME           (10)  // ms, timeout of waiting for ACK

#define STATUS_READ     2
#define DATA_WRITE      1
#define DATA_READ       3

uint8_t replybuff[20];//массив для сбора байтов готовности
uint8_t rl;

#define PN532_PACKBUFFSIZ 64
uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];
//uint16_t pn532_readbuffer[PN532_PACKBUFFSIZ];
uint8_t pn532_readbuffer[PN532_PACKBUFFSIZ];

// Mifare Commands
#define MIFARE_CMD_AUTH_A (0x60)
#define MIFARE_CMD_AUTH_B (0x61)
#define MIFARE_CMD_READ (0x30)
#define MIFARE_CMD_WRITE (0xA0)
#define MIFARE_CMD_TRANSFER (0xB0)
#define MIFARE_CMD_DECREMENT (0xC0)
#define MIFARE_CMD_INCREMENT (0xC1)
#define MIFARE_CMD_STORE (0xC2)
#define MIFARE_ULTRALIGHT_CMD_WRITE (0xA2)

//uint16_t rdbuff_16[PN532_PACKBUFFSIZ];

#define MIFAREDEBUG

//uint8_t ackbuff[6];

//private:
  int8_t _irq = -1, _reset = -1;
  int8_t _uid[7];      // ISO14443A uid
  int8_t _uidLen;      // uid len
  int8_t _key[6];      // Mifare Classic key
  int8_t _inListedTag; // Tg number of inlisted tag.

extern void Delay(uint32_t nTime);

void PN532_Delay(uint32_t cntdelay);
void PN532_SPIInit(void);
bool PN532_WakeUp(void);
bool PN532_SAMConfig(void);
bool PN532_setPassiveActivationRetries(uint8_t maxRetries);
bool PN532_readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout);
bool PN532_readDetectedPassiveTargetID(uint8_t *uid, uint8_t *uidLength);
bool PN532_sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout);
bool PN532_diagnoseCommunicationLineTest(void);
uint32_t PN532_getGeneralStatus(void);
uint32_t PN532_getFirmwareVersion(void);
void PN532_writecommand(uint8_t *cmd, uint8_t cmdlen);
void PN532_write( uint8_t *buffer, size_t len);
void PN532_readdata(uint8_t *rd_buff, uint8_t len_rd_buff,uint16_t sendvalue);
bool PN532_isready();
bool PN532_waitready(uint16_t timeout);
bool PN532_readack();
uint8_t PN532_getDataTarget(uint8_t *cmd, uint8_t *cmdlen);
uint8_t PN532_setDataTarget(uint8_t *cmd, uint8_t cmdlen);
uint8_t PN532_AsTarget();

uint8_t PN532_mifareclassic_AuthenticateBlock(uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData);
uint8_t PN532_mifareclassic_ReadDataBlock(uint8_t blockNumber, uint8_t *data);
uint8_t PN532_mifareclassic_WriteDataBlock(uint8_t blockNumber, uint8_t *data);

uint8_t PN532_Read_Data(uint8_t *uid_card, uint8_t uid_card_len, uint8_t blockNumber, uint8_t *data);

#endif /* NFCPN532_H_ */
