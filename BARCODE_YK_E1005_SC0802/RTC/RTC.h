/*
 * RTC.h
 *
 *  Created on: 22 мар. 2023 г.
 *      Author: crazy
 */

#ifndef RTC_H_
#define RTC_H_

#include "stm32f10x.h"
#include "stdio.h"



void RTC_INIT(void);
uint32_t RTC_GET_COUNTER(void);
void RTC_SET_COUNTER(uint32_t count);


#endif /* RTC_H_ */
