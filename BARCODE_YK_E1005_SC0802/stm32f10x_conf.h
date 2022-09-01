/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_conf.h  
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

#include <stdint.h>
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//#include "../../SPL/src/stm32f10x_tim.c"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "misc.h"

#define QUEUE_LEN		64
#define BUFFER_LEN		128

#define USB_STATE_LEN	26
//------------------------------------------------------------------------------
// USART1 Scaner
#define USART_RCC_PERIPH_CLOCK_CMD	RCC_APB2PeriphClockCmd
#define USART_RCC_APBPORT		RCC_APB2Periph_USART1
#define USART_RCC_PORT			RCC_APB2Periph_GPIOA
#define USART_GPIO_PORT			GPIOA
#define USART_TX_PIN			GPIO_Pin_9
#define USART_RX_PIN			GPIO_Pin_10
//#define USART_GPIO_TX_PINSOURCE	GPIO_PinSource9
//#define USART_GPIO_RX_PINSOURCE	GPIO_PinSource10
//#define USART_GPIOAF			GPIO_AF_USART1
#define USART_IRQ				USART1_IRQn
#define USART_NUM				USART1
//------------------------------------------------------------------------------
// USART2 Computer
#define USART2_RCC_PERIPH_CLOCK_CMD	 RCC_APB1PeriphClockCmd
#define USART2_RCC_APBPORT			 RCC_APB1Periph_USART2
#define USART2_RCC_PORT				 RCC_APB1Periph_USART2//RCC_APB2Periph_GPIOA
#define USART2_GPIO_PORT			 GPIOA
#define USART2_TX_PIN				 GPIO_Pin_2
#define USART2_RX_PIN				 GPIO_Pin_3
#define USART2_IRQ					 USART2_IRQn
#define USART2_NUM					 USART2
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// SPI2 NFC
#define SPI_RCC_PERIPH_CLOCK_CMD	RCC_APB1PeriphClockCmd
#define SPI_RCC_PORT_CLOCK_CMD	    RCC_APB2PeriphClockCmd
#define SPI_RCC_AFIO_CLOCK_CMD      RCC_APB2PeriphClockCmd
#define RCC_AFIO 				    RCC_APB2Periph_AFIO

#define SPI_RCC_APBPORT		        RCC_APB1Periph_SPI2
#define SPI_RCC_PORT			    RCC_APB2Periph_GPIOB //RCC_APB1Periph_SPI2//RCC_APB2Periph_GPIOB
#define SPI_GPIO_PORT			    GPIOB
#define SPI_NSS_PIN				    GPIO_Pin_12
#define SPI_SCK_PIN				    GPIO_Pin_13
#define SPI_MISO_PIN			    GPIO_Pin_14
#define SPI_MOSI_PIN			    GPIO_Pin_15
//-----------------------------------------------------------------------
/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */

#define USE_DEFAULT_TIMEOUT_CALLBACK

/* Defines for newlib_stubs.c means USART number will be used for IO output/input */
#define STDOUT_USART 6
#define STDERR_USART 6
#define STDIN_USART 6
/* ------------------------------------------------------------------------------ */

 //Uncomment for enabling debug port COM<number> where:
 /*	COM1  -  USART6
  * COM2  -	 USART3
  * COM3  -  USART1
  */
#define DEBUG_ENABLE	1

/* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external 
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */

/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */

// #define USE_FULL_ASSERT    1

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed.
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */





#endif /* __STM32F4xx_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
