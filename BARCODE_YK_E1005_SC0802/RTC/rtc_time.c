#include "rtc_time.h"

void timer_to_cal (unsigned long timer, rtc_cal * rtc_time)
{
	unsigned long a;
	char b;
	char c;
	char d;
	unsigned long time;

	time = timer%SEC_A_DAY;
	a = ((timer+43200)/(86400>>1)) + (2440587<<1) + 1;
	a>>=1;
	rtc_time->wday = a%7;
	a+=32044;
	b=(4*a+3)/146097;
	a=a-(146097*b)/4;
	c=(4*a+3)/1461;
	a=a-(1461*c)/4;
	d=(5*a+2)/153;
	rtc_time->mday=a-(153*d+2)/5+1;
	rtc_time->mon=d+3-12*(d/10);
	rtc_time->year=100*b+c-4800+(d/10);
	rtc_time->hour=time/3600;
	rtc_time->min=(time%3600)/60;
	rtc_time->sec=(time%3600)%60;
}

unsigned long cal_to_timer (rtc_cal * rtc_time)
{
	char a;
	int y;
	char m;
	unsigned long Uday;
	unsigned long time;

	a=((14-rtc_time->mon)/12);
	y=rtc_time->year+4800-a;
	m=rtc_time->mon+(12*a)-3;
	Uday=(rtc_time->mday+((153*m+2)/5)+365*y+(y/4)-(y/100)+(y/400)-32045)-2440588;
	time=Uday*86400;
	time+=rtc_time->sec+rtc_time->min*60+rtc_time->hour*3600;
	return time;
}



