/*
 * fifo.c
 *

 */

#include "stm32f10x_conf.h"

char g_aInputQueue [ QUEUE_LEN ];
static int g_iBegin = 0;
static int g_iEnd = 0;

extern bool b_getversion;
extern uint8_t cntbyteversion;
extern uint8_t ScanerVersion [];


ErrorStatus FIFO_GetNextData ( char * pResult ) {
	ErrorStatus res = ERROR;

	if ( g_iBegin != g_iEnd ) {
		*pResult = g_aInputQueue [ g_iBegin++ ];
		g_iBegin %= QUEUE_LEN;
		res = SUCCESS;
	}

	return res;
}

void USART_NUM_IRQHandler ( void ) {
	if (USART_GetFlagStatus ( USART_NUM, USART_FLAG_RXNE ) == SET) {
		if(b_getversion){

			USART_ClearITPendingBit ( USART_NUM, USART_IT_RXNE );
			ScanerVersion [ cntbyteversion++ ] = USART_ReceiveData ( USART_NUM );
			if (cntbyteversion>71){
				b_getversion=DISABLE;
				cntbyteversion=0;
			}

		}
		else{

		USART_ClearITPendingBit ( USART_NUM, USART_IT_RXNE );
		g_aInputQueue [ g_iEnd++ ] = USART_ReceiveData ( USART_NUM );
		g_iEnd %= QUEUE_LEN;
		}
	}
}
