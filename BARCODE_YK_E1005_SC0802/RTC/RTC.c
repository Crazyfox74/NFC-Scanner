/*
 * RTC.c
 *
 *  Created on: 22 ���. 2023 �.
 *      Author: crazy
 */

#include "RTC.h"



void RTC_INIT(void)
{
	if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN){		// �������� ������ �����, ���� �� ��������, �� ����������������
		RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;	// ��������� ������������ PWR � Backup
		PWR->CR |= PWR_CR_DBP;		// ���������� ������� � Backup �������
		RCC->BDCR |= RCC_BDCR_BDRST;	// ����� Backup �������
		RCC->BDCR &= ~RCC_BDCR_BDRST;
		RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;	// ����� LSE ��������� (����� 32768) � ������ ������������
		RCC->BDCR |= RCC_BDCR_LSEON;
		while((RCC->BDCR&RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}	// �������� ���������
		BKP->RTCCR |= 3;	// ���������� RTC
		while(!(RTC->CRL&RTC_CRL_RTOFF));	// �������� �� ����� ��������� ��������� RTC
		RTC->CRL |= RTC_CRL_CNF;		// ���������� ������ � �������� RTC
		RTC->PRLL = 0x7FFF;		// ��������� �������� �� 32768
		RTC->CRL &= ~RTC_CRL_CNF;	// ������ ������ � �������� RTC
		while(!(RTC->CRL&RTC_CRL_RTOFF));	// �������� ����� ������
		RTC->CRL &= (uint16_t)~RTC_CRL_CNF;		//������������� RTC
		while((RTC->CRL&RTC_CRL_RSF) != RTC_CRL_RSF){}	// �������� �������������
		PWR->CR &= ~(PWR_CR_DBP);	// ������ ������� � Backup �������

	}
}


uint32_t RTC_GET_COUNTER(void)
{
	return (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);	// ��������� �������� ��������
}

void RTC_SET_COUNTER(uint32_t count)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;	// ��������� ������������ PWR � Backup
	PWR->CR |= PWR_CR_DBP;	// ���������� ������� � Backup �������
	while(!(RTC->CRL & RTC_CRL_RTOFF)); // �������� ����� ��������� ��������� RTC
	RTC->CRL |= RTC_CRL_CNF;	// ���������� ������ � �������� RTC
	RTC->CNTH = count >> 16;	// ������ ������ �������� �������� ��������
	RTC->CNTL = count;
	RTC->CRL &= ~RTC_CRL_CNF;		// ������ ������ � �������� RTC
	while(!(RTC->CRL & RTC_CRL_RTOFF));	// �������� ����� ������
	PWR->CR &= ~PWR_CR_DBP;		// ������ ������� � Backup �������

}


