/**************************************************************************
 * ������ ���������� ������������� ��� ������ � RTC, � ������ � Unix Time *
 * ��������� ���������� �� �������� � ���������(����, �����, ���) 		  *
 * � ����� (����, ������, �������), � �������							  *
 **************************************************************************/
#include "stm32f10x.h"
#include "stdio.h"

#define SEC_A_DAY 86400

typedef struct
	{
	int year;
	char mon;
	char mday;
	char hour;
	char min;
	char sec;
	char wday;
	} rtc_cal;

void timer_to_cal (unsigned long timer, rtc_cal * rtc_time);
unsigned long cal_to_timer (rtc_cal * rtc_time);
