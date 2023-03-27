/*
 * RTC.c
 *
 *  Created on: 22 мар. 2023 г.
 *      Author: crazy
 */

#include "RTC.h"



void RTC_INIT(void)
{
	if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN){		// проверка работы часов, если не включены, то инициализировать
		RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;	// включение тактирования PWR и Backup
		PWR->CR |= PWR_CR_DBP;		// разрешение доступа к Backup области
		RCC->BDCR |= RCC_BDCR_BDRST;	// сброс Backup области
		RCC->BDCR &= ~RCC_BDCR_BDRST;
		RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;	// выбор LSE источника (кварц 32768) и подача тактирования
		RCC->BDCR |= RCC_BDCR_LSEON;
		while((RCC->BDCR&RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}	// ожидание включения
		BKP->RTCCR |= 3;	// калибровка RTC
		while(!(RTC->CRL&RTC_CRL_RTOFF));	// проверка на конец изменений регистров RTC
		RTC->CRL |= RTC_CRL_CNF;		// разрешение записи в регистры RTC
		RTC->PRLL = 0x7FFF;		// настройка делителя на 32768
		RTC->CRL &= ~RTC_CRL_CNF;	// запрет записи в регистры RTC
		while(!(RTC->CRL&RTC_CRL_RTOFF));	// ожидание конца записи
		RTC->CRL &= (uint16_t)~RTC_CRL_CNF;		//синхронизация RTC
		while((RTC->CRL&RTC_CRL_RSF) != RTC_CRL_RSF){}	// ожидание синхронизации
		PWR->CR &= ~(PWR_CR_DBP);	// запрет доступа к Backup области

	}
}


uint32_t RTC_GET_COUNTER(void)
{
	return (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);	// получение значения счетчика
}

void RTC_SET_COUNTER(uint32_t count)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;	// включение тактирования PWR и Backup
	PWR->CR |= PWR_CR_DBP;	// разрешение доступа к Backup области
	while(!(RTC->CRL & RTC_CRL_RTOFF)); // проверка конца изменения регистров RTC
	RTC->CRL |= RTC_CRL_CNF;	// разрешение записи в регистры RTC
	RTC->CNTH = count >> 16;	// запись нового значения счетного регистра
	RTC->CNTL = count;
	RTC->CRL &= ~RTC_CRL_CNF;		// запрет записи в регистры RTC
	while(!(RTC->CRL & RTC_CRL_RTOFF));	// ожидание конца записи
	PWR->CR &= ~PWR_CR_DBP;		// запрет доступа к Backup области

}


