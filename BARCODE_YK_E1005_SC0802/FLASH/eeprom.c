/*
 * eeprom.c
 *
 *  Created on: 22 мар. 2023 г.
 *      Author: crazy
 */

/*
#include "eeprom.h"

static uint32_t pageAddress[PAGES_NUM] = {PAGE_0_ADDRESS, PAGE_1_ADDRESS};
static uint32_t varIdList[VAR_NUM] = {PARAM_1, PARAM_2};

uint32_t FLASH_Read(uint32_t address);
PageState EEPROM_ReadPageState(PageIdx idx);
EepromResult EEPROM_SetPageState(PageIdx idx, PageState state);
EepromResult EEPROM_ClearPage(PageIdx idx);
EepromResult EEPROM_Format();
EepromResult EEPROM_GetActivePageIdx(PageIdx *idx);
EepromResult EEPROM_Init();

EepromResult EEPROM_WriteData(uint32_t address, uint32_t varId, uint32_t varValue)
{
  EepromResult res = EEPROM_OK;
 // HAL_StatusTypeDef flashRes = HAL_OK;
  FLASH_Status flashRes = FLASH_COMPLETE;
  uint64_t fullData = ((uint64_t)varValue << 32) | (uint64_t)varId;


 // HAL_FLASH_Unlock();

  FLASH_Unlock();
  flashRes = FLASH_ProgramWord(address, fullData);	//переделать под 32бит
  FLASH_Lock();

  if(flashRes != FLASH_COMPLETE)
  {
	res = EEPROM_ERROR;
  }

  /*flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, fullData);
  HAL_FLASH_Lock();



  if (flashRes != HAL_OK)
  {
    res = EEPROM_ERROR;
  }
*/
 // return res;
//}


