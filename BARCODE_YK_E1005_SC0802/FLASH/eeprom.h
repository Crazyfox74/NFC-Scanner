/*
 * eeprom.h
 *
 *  Created on: 22 мар. 2023 г.
 *      Author: crazy
 */

#ifndef EEPROM_H_
#define EEPROM_H_
/*
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
//#include "stm32f1xx_hal.h"


#define PAGE_DATA_OFFSET                                                8
#define PAGE_DATA_SIZE                                                  8

#define PARAM_1                                                         0x12121212
#define PARAM_2                                                         0x34343434
#define VAR_NUM                                                         2

#define PAGE_0_ADDRESS                                                  0x0801F800
#define PAGE_1_ADDRESS                                                  0x0801FC00
#define PAGE_SIZE                                                       1024


typedef enum {
  PAGE_CLEARED = 0xFFFFFFFF,
  PAGE_ACTIVE = 0x00000000,
  PAGE_RECEIVING_DATA = 0x55555555,
} PageState;

typedef enum {
  PAGE_0 = 0,
  PAGE_1 = 1,
  PAGES_NUM = 2,
} PageIdx;

typedef enum {
  EEPROM_OK = 0,
  EEPROM_ERROR = 1,
} EepromResult;



/* Functions -----------------------------------------------------------------*/




#endif /* EEPROM_H_ */
