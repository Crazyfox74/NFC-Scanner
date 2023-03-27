/* PB0									Button
 * PB6, PB7								I2C LCD
 * PA9, PA10							USART1 BarCodeScan
 * PB12, PB13, PB14, PB15 SPI NFC
 * PB11-reset, PA8-IRQ	SPI NFC
 *
 *PA11-USBDM PA12-USBDP					USB CDC
 *
*/
#include <errno.h>


#include "stdio.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "nfcpn532.h"
#include "nfcpn532.c"
#include "stm32f10x_flash.h"
#include "stm32f10x_flash.c"


//#include "fifo.c"

#include <string.h>



#include "hw_config.h"
#include "RTC/rtc_time.c"

#include "RTC/RTC.h"
#include "RTC/RTC.c"
//#include "RTC/unix_time.h"


//******************************************************************************

#define ST_DEV_ALL_ERR 0x00
#define ST_DEV_NFC_NOERR 0x01
#define ST_DEV_BAR_NOERR 0x10
#define ST_DEV_ALL_NOERR  0x11

#define TIMEOUT_MAX				1000

#define LCD_I2C_ADDRESS			0x4E

#define NVIC_GROUP		NVIC_PriorityGroup_0

#define LCD_I2C_SCL_RCC_PERIPH_CLOCK_CMD RCC_APB2PeriphClockCmd
#define LCD_I2C_SCL_RCC_PORT			RCC_APB2Periph_GPIOB
#define LCD_I2C_SCL_GPIO_PORT			GPIOB
#define LCD_I2C_SCL_PIN					GPIO_Pin_6

#define LCD_I2C_SDA_RCC_PERIPH_CLOCK_CMD RCC_APB2PeriphClockCmd
#define LCD_I2C_SDA_RCC_PORT			RCC_APB2Periph_GPIOB
#define LCD_I2C_SDA_GPIO_PORT			GPIOB
#define LCD_I2C_SDA_PIN					GPIO_Pin_7

#define LCD_I2C_RCC_PERIPH_CLOCK_CMD	RCC_APB1PeriphClockCmd
#define LCD_I2C_RCC_APBPORT				RCC_APB1Periph_I2C1
#define LCD_I2C_NUM						I2C1

#define LCD_RS							0x04
#define LCD_RW							0x01
#define LCD_E							0x02

#define LCD_STR_LEN						8

#define  LCD_CURSOR_1STR				0x80
#define  LCD_CURSOR_2STR				0xC0

//*************************************************************
#define BTN_RCC_PERIPH_CLOCK_CMD		RCC_APB2PeriphClockCmd
#define BTN_RCC_GPIO_PORT				RCC_APB2Periph_GPIOB
#define BTN_GPIO_PORT					GPIOB
#define BTN_PIN							GPIO_Pin_0

#define BTN_EXTI_PORTSOURCE				GPIO_PortSourceGPIOB
#define BTN_EXTI_PINSOURCE				GPIO_PinSource0
#define BTN_EXTI_LINE					EXTI_Line0
#define BTN_IRQ							EXTI0_IRQn

#define BTN_TIMEOUT						1000//500
//*********************************************************
#define LED_RCC_PERIPH_CLOCK_CMD		RCC_APB2PeriphClockCmd
#define LED_RCC_GPIO_PORT				RCC_APB2Periph_GPIOA
#define LED_GPIO_PORT					GPIOA
#define LED_PIN							GPIO_Pin_12

#define LED_EXTI_PORTSOURCE				GPIO_PortSourceGPIOA
#define LED_EXTI_PINSOURCE				GPIO_PinSource12
#define LED_EXTI_LINE					EXTI_Line12
#define LED_IRQ							EXTI0_IRQn

#define LED_FLASH1						50000
#define LED_FLASH2						100000
//**********************************************************
#define SUM_MAX							10000000
#define SUM_STR_LEN						(LCD_STR_LEN - 1)

#define RUN_STR_DELAY_TST				200
#define RUN_STR_DELAY					800
#define STR_CMD_LENGTH					12

#define CNT_LOOP_PN532	50000

#define TIMER1_PRESCALER 7200
//	500=50мс
/// 1000=100мс
//	10000=1мс

#define	TIMER1_RELOAD_PERIOD 500

#define BTN_PRESSED 1
#define BTN_UNPRESSED 2



/********/

#define PA7_RCC_PERIPH_CLOCK_CMD		RCC_APB2PeriphClockCmd
#define PA7_RCC_GPIO_PORT				RCC_APB2Periph_GPIOA
#define PA7_GPIO_PORT					GPIOA
#define PA7_PIN							GPIO_Pin_7



 /**************/

#define FIRMWARE_PAGE_OFFSET 	0xC800

#define flash_addr1 0x0800C800

//A77c

// адрес начала С800

//0800 |

typedef enum {
	UNPRESSED = 0,
	PRESSED,
} ButtonState_t;

/* Display initialization commands */
static uint8_t g_arInitDisplayCommands[] = {
		0b0011, 0b0011, 0b0011,
		0b0010, 0b0010, 0b1000,
		//4 bits interface, confirm 4 bits interface
		//N == 1 (2 active lines), F == 0 (matrix 5 x 7 dots)
		0b0000, 0b1000,
		//D == 0, C == 0, B == 0
		//Display OFF
		0b0000, 0b0110,
		//I/D == 1, S == 0
		//shift cursor to right
		0b0000, 0b1100,
		//D == 1, C == 0, B == 0
		//Display ON
};

__IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

ButtonState_t buttonState = UNPRESSED;
char cmd_Version [STR_CMD_LENGTH]      = {0x0A, 0x04, 0x31, 0x00, 0x00, 0x59, 0x52, 0x56, 0x4b, 0xFF, 0xFD, 0x76};
char cmd_TriggerScann [STR_CMD_LENGTH] = {0x0A, 0x04, 0x31, 0x00, 0x00, 0x59, 0x4c, 0x54, 0x4b, 0xFF, 0xFD, 0x7E};
char g_sBuffer [ BUFFER_LEN + 1 ];
int g_iBufferLength = 0;

static char g_sBarCode [ USB_STATE_LEN + 1 ];
static int g_iSum = 0;
static char g_sSum[SUM_STR_LEN + 2];

static char g_sNfcCode [ USB_STATE_LEN + 1 ];

uint8_t pn532_cmd[PN532_PACKBUFFSIZ];
uint8_t pn532_cmdlen;

uint8_t btnpress;
uint8_t pn532_rddata[PN532_PACKBUFFSIZ];
uint8_t pn532_rdlen;


bool b_getversion=DISABLE;
 uint8_t cntbyteversion;
 uint8_t ScanerVersion [71]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool b_firstrddata=DISABLE;

//****bluetooth uart
char date_sBuffer [ BUFFER_LEN + 1];
int date_iBufferLength = 0;


//******

//USBLIB_WByte _LineState;


void SCAN_SumToString ( int sum, char * sumStr );
void SCAN_SendCMD (char* strcmd,int sz_cmd);
uint8_t I2C_SingleRandomWrite ( I2C_TypeDef* I2Cx, uint8_t Device, uint8_t Data );

void LCD_SetCursor ( uint8_t index );
void LCD_WriteData ( uint8_t data );
void LCD_WriteString ( char * str );
//Отображение кода сканера штрих-кодов бегущей строкой в первой строчке LCD
void LCD_RunStringBARCode ( char * str, uint32_t delay, uint8_t start );
//Отображение кода сканера NFC бегущей строкой во второй строчке LCD
void LCD_RunStringNFCCode ( char * str, uint32_t delay, uint8_t start );
void LCD_DisplaySum ( void );
//Сборка сборка бегущей строки с версией ПО NFC модудя для LCD
void LCD_PN532_CreateRunInfoString();
//Сборка сборка бегущей строки с версией ПО сканера штрих-кодов модудя для LCD
void LCD_BAR_CreateRunInfoString();

void LCD_RunStringDevInfo ( char * str1line,char * str2line, uint32_t delay);
void LCD_Init(void);

void BTN_Init(void);
void LED_Init(void);

void Usart2Init(void);



void Timer1Init(void);
void EnableTimer1Interrupt();
void DisableTimer1Interrupt();

void ValueInterrupToStr();

void Usart2_SendData (char* strcmd, int sz_cmd);

void SCAN_UsartInit(void);
void SCAN_SumToString ( int sum, char * sumStr );

uint32_t GetTickCount();
void Delay ( uint32_t nTime );

ErrorStatus FIFO_GetNextData ( char * pResult );

void Send2PC_FirmWareVesion (uint32_t version);
void PN532_FirmWareVersion2String (uint32_t version);
bool fourbytes2strhex (uint8_t *indatarray, uint8_t *outstrhex);

void PN532_WriteTagtoHex();

void PN532_WriteCARD();

uint8_t PN532_Write_Data(uint8_t *uid_card, uint8_t uid_card_len, uint8_t blockNumber, uint8_t *data);

bool BytesHex2Str(uint8_t *indatarray, uint8_t size_array, uint8_t *outstrhex);

void SD_SPI_GPIO_Init(void);

void SD_SPI_AF_Init(void);

void PA7_Init(void);

void FLASH_Init(void);
uint32_t FLASH_Read(uint32_t address);
void FLASH_Write(uint32_t Value);

ErrorStatus Date_GetNextDate(char *dResult );

//******************************************************************************

uint8_t nfc_command;
uint8_t nfc_cntcomand;
bool b_wakeup;
bool b_samconfig;
bool b_rertyes;
bool b_discannfc;
bool success;
//uint8_t uidt[] = { 0, 0, 0x0F, 0x1F, 0, 0, 0, 0};	// Buffer to store the returned UID
//uint8_t uidt_2[] = { 0, 0, 0, 0, 0, 0, 0, 0};	// Buffer to store the returned UID
bool b_hextostr;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };	// Buffer to store the returned UID
uint8_t uidhex2pc4b[20] = { '0', 'x', '0', '0', ' ','0', 'x', '0', '0', ' ','0', 'x', '0', '0', ' ','0', 'x', '0', '0', 0x00 };	// Buffer to store the returned UID
//---------------------------------------------------------------------------------------------
uint8_t uidnfcwrdara[] = { '0', '0', '0', '0', '0', '0', '0', '0'};	// массив для записи номера метки nfc
uint8_t u8_cntchar;
uint8_t u8_cntindex;

//--------------------------------------------------
uint8_t outwrdata[]={0,0,0,0};
//--------------------------------------------------------------------------------------------
uint8_t uidLength;				// Length of the UID (4 or 7 bytes depending on ISO14443A card type)

uint32_t decuid;
uint32_t cntlooppn532=0;

uint8_t cntbutpress=0;

bool b_1secflg;
uint16_t cnttimer30sec=0;
uint16_t cnttimer1sec=0;
uint16_t cnttimer1=0;
uint8_t previousState=0;
uint8_t u8_btnstate=0;
uint16_t timer1data=0;

uint16_t testtimer1=0;
bool timer1flag;

uint8_t u8_wrnfcstatus=0;
bool b_wrstr1;//флаг выдачи первой строки в режиме записи

uint8_t pn532_rd_stat=0;

uint8_t pn532_wr_stat;

NVIC_InitTypeDef        NVIC_InitTmr1Structure;
EXTI_InitTypeDef		EXTI_InitExtiStruct;
NVIC_InitTypeDef		NVIC_InitExtiStruct;


static char s_lcd_hello[]    = " Hello! ";
static char s_lcd_scanner_rdy[]  = "Scanner ";
static char s_lcd_scanner[]  = "Scanner NFC & Barcode";
static char s_lcd_test[]     = "Test... ";
static char s_lcd_err[]      = "error...";
static char s_lcd_ok[]       = "  OK!   ";
static char s_lcd_nfc[]      = "  NFC   ";
static char s_lcd_barcode[]  = "Barcode ";
static char s_lcd_ready[]    = " ready  ";
static char s_lcd_nfc_err[]  = "NFC module:  test error...";
static char s_lcd_bar_err[]  = "Barcode reader module: test error...";
static char s_lcd_clrstr[]   = "                   " ;
static char s_lcd_wrnfc[]      = "Write NFC";

static char s_lcd_barcode_read[]  = "Barcode: ";
static char s_lcd_nfc_read[]      = "NFC: ";

static char s_pc_hello[]    = "Hello!\r\n";
static char s_pc_scanner[]  = "Scanner NFC & Barcode...\r\n";
static char s_pc_scanner_ready[]  = "Scanner NFC & Barcode ready...\r\n";

static char s_pc_test[]     = "Test device wait...\r\n";
static char s_pc_barcode[]     = "Barcode device read code: ";
static char s_pc_err[]      = "test error...";

//static char s_point[1]    = ".";
static char s_newline[2]  = {"\r\n"};
static char s_pn_samconfig_ok[]   = "NFC configuration set\r\n";
static char s_pn_samconfig_err[]  = "NFC configuration error\r\n";

static char s_pn_notfoundcard[]   = "NFC card not found\r\n";
static char s_pn_foundcard[]      = "NFC card found!\r\n";
static char s_pn_uidlength[]      = "UID Length (bytes): ";
static char s_pn_uidvalue[]       = "UID Value (hex): ";
static char s_pn_uidlengthdata[]  = "000";
static char s_pn_uiddecdata[]     = "0000000000";
static char s_pn_uiddecvalue[]    = "UID Value (dec): ";

//***********************************************************global NFC var

static char s_space[1]       = " ";
static char s_pn_point[1]    = ".";
static char s_pn_newline[2]  = {0x0D,0x0A};

static char s_pn_nchip[2]     = "  ";
static char s_pn_fw1nchip[3]  = "   ";
static char s_pn_fw2nchip[3]  = "   ";
static char s_pn_suchip[2]    = "  ";

static char s_pn_nfchip[]   = "NFC module: test error...\r\n";
static char s_pn_fchip[]    = "NFC chip PN5";
static char s_pn_wfchip[]   = "Firmware ver. ";
static char s_pn_suppchip[] = "Supports ";

uint8_t pn_chip;
uint8_t pn_fw1chip;
uint8_t pn_fw2chip;
uint8_t pn_suppchip;

char s_pn_info[128] = "";
char s_bar_info[128] = "";

char s_bar_ver_info[42]="";
char s_bar_site_info[42]="";

uint8_t statusDevice;

static char b_bounce_value[]="00000";

uint8_t rdDataBlock[16];

static char s_cal_data[21]="00.00.0000-00:00:00\r\n";
static char s_scan_d_c[42]="";

static char s_pc_PN532_info1[]="NFC read operation correct";
static char s_pc_PN532_info2[]="NFC write operation correct";
static char s_pc_PN532_err1[]="Authentication failed for sector";
static char s_pc_PN532_err2[]="Unable to write to sector";
static char s_pc_PN532_err3[]="Unable to write trailer block of sector";
static char s_pc_PN532_err4[]="Unable to read data block";

uint8_t DataHex2pc[47] = { '0', '0',' ', '0', '0',' ', '0','0',' ', '0', '0',' ', '0', '0',' ','0', '0',' ', '0', '0',' ', '0','0',' ', '0', '0',' ', '0', '0',' ', '0', '0',' ', '0', '0',' ', '0','0',' ', '0', '0',' ', '0', '0',' ','0', '0' };
static char s_pn_data_block0[]    = "Data from block 1: ";



static char sd_error_con[]="Can't connect to SD\r\n";
static char sd_error_wr[]="Can't write to SD\r\n";
static char sd_error_rd[]="Can't read from SD\r\n";

uint32_t flash_test;


uint32_t timer = 138450400+14400;
uint32_t tim;
static char s_cal_sec[2]="00";
static char s_cal_min[2]="00";
static char s_cal_hour[2]="00";



//***********************************************************
//Сборка сборка бегущей строки с версией ПО NFC модуля для LCD
void LCD_PN532_CreateRunInfoString(){

	strcpy( s_pn_info, s_pn_fchip);	//копирую первую заготовку в пустую строку
	strcat(s_pn_info, s_pn_nchip);	//копирую в заготовку пробельную строку
	strcat(s_pn_info,s_space );
	strcat(s_pn_info,s_pn_wfchip );
	strcat(s_pn_info, s_pn_fw1nchip);
	strcat(s_pn_info,s_pn_fw2nchip );
	strcat(s_pn_info,s_space );
	strcat(s_pn_info,s_pn_suppchip);
	strcat(s_pn_info,s_pn_suchip);

}
//***********************************************************



//***********************************************************
//Сборка сборка бегущей строки с версией ПО сканера штрих-кодов модуля для LCD
void LCD_BAR_CreateRunInfoString(){

//	while (ScanerVersion != NULL) {
//		if (strstr(ScanerVersion, "\r\n") != NULL) {
			/* Выделение лексемы,
			 *  ограниченной справа одним из символов множества "\r\n"
	//		 */
//			strtok(ScanerVersion, "\r\n");
//
	//		char *s_bar_info = ScanerVersion;
//			while (strpbrk(s_bar_info, "\n\xFF") != NULL) {
//				s_bar_info++;
//			}
//		}
//	}

	char *ptr_bar_ver_info=strtok(ScanerVersion,"\r\n");
	strcpy(s_bar_info,ptr_bar_ver_info);
	strcat(s_bar_info,s_space);
	ptr_bar_ver_info=strtok(NULL,"\r\n");
	strcat(s_bar_info,ptr_bar_ver_info);

//	char *s_bar_info=strcat(s_bar_ver_info,s_bar_site_info);
//	strcpy(s_bar_info,s_bar_ver_info);


}
//***********************************************************

uint32_t UIDResponse(uint8_t *uid,  uint8_t uidLength);

uint32_t UIDResponse(uint8_t *uid,  uint8_t uidLength){
	uint32_t result;
	result=0;

	if(uidLength==4){

		result = uid[0];
		result <<= 8;
		result |= uid[1];
		result <<= 8;
		result |= uid[2];
		result <<= 8;
		result |= uid[3];
	}
return result;

}
//**********************************************************************
void PA7_Init(void){
	GPIO_InitTypeDef		GPIO_InitStruct;
	PA7_RCC_PERIPH_CLOCK_CMD ( PA7_RCC_GPIO_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = PA7_PIN;
	GPIO_Init ( PA7_GPIO_PORT, &GPIO_InitStruct );
//#define LED_GPIO_PORT					GPIOA
//#define LED_PIN							GPIO_Pin_12


	GPIO_WriteBit(PA7_GPIO_PORT, PA7_PIN, 0);
}


//********************

void SD_SPI_GPIO_Init(void)
{
    /*Configure SPI GPIO************************************************************************/
    /*You should configure MISO, MOSI and SCK pins here!*/
    RCC->APB2ENR     |=  RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
   // GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);
 //   GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
  //  GPIOA->CRL |= (GPIO_CRL_MODE5 | GPIO_CRL_MODE7);

    GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5);	//sck
    GPIOA->CRL |= (GPIO_CRL_CNF6_0);	//miso
    GPIOA->CRL |= (GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7);	//mosi

    /*
    GPIOC->MODER    &=  ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOC->MODER    |=  (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
    GPIOC->OTYPER   &=  ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12);
    GPIOC->OSPEEDR  |=  (GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);
    GPIOC->PUPDR    &=  ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    GPIOC->PUPDR    |=  (GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
*/

    /*********************************************************************************************/
}


void SD_SPI_AF_Init(void){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);

	GPIOB->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_CNF4 | GPIO_CRL_CNF5);
	GPIOB->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_MODE4 | GPIO_CRL_MODE5);

	GPIOB->CRL |= (GPIO_CRL_CNF3_1 | GPIO_CRL_MODE3);
	GPIOB->CRL |= (GPIO_CRL_CNF4_0);
	GPIOB->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5);



}

//**************************FLASH********************
void FLASH_Init(void)
{
	FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	FLASH_SetLatency(FLASH_Latency_1);

}
uint32_t FLASH_Read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

void FLASH_Write(uint32_t Value)
{
	uint16_t i;
	uint32_t pageAdr;

	pageAdr = NVIC_VectTab_FLASH | FIRMWARE_PAGE_OFFSET;                    // Адрес страницы памяти

	FLASH_Unlock();
	FLASH_ErasePage(pageAdr);

	FLASH_ProgramWord((uint32_t)(pageAdr), (uint32_t)Value);
	FLASH_Lock();


}



//******************************************************************************

int main(void)
{
	g_sSum[SUM_STR_LEN + 1] = '\0';

	rtc_cal rtc_time;
	RTC_INIT();

	RCC_PLLCmd(DISABLE);
	RCC_HSEConfig(RCC_HSE_ON);
	RCC_ClockSecuritySystemCmd(ENABLE);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	uint32_t tmpreg = 0;
	tmpreg=RCC->CFGR;
	RCC->CFGR=tmpreg & 0xFFFFF0FF;
	//RCC->CFGR=tmpreg|0x00000400;//HCLK / 2 - значение по умолчанию? (36МГц)
	RCC->CFGR=tmpreg|0x00000500;//HCLK / 4

	RCC->CFGR|=RCC_CFGR_USBPRE;	//предделитель для usb

	//RCC->CFGR=tmpreg | 0x00000600;//HCLK / 8

    RCC_PLLCmd(ENABLE);

	/* SysTick end of count event each 1 ms */
	RCC_GetClocksFreq ( &RCC_Clocks );
	SysTick_Config ( RCC_Clocks.HCLK_Frequency / 1000 );

	btnpress=0;



	//RTC

	RTC_SET_COUNTER(timer);



//	FLASH_Init();

	LCD_Init();
	BTN_Init();
	u8_btnstate=GPIO_ReadInputDataBit ( BTN_GPIO_PORT, BTN_PIN );
	previousState=u8_btnstate;
    LED_Init();

    Usart2Init();
	SCAN_UsartInit();
	PN532_SPIInit();


	//BTN_UNPRESSED

	nfc_cntcomand=0;

	statusDevice=ST_DEV_ALL_ERR;

	uint32_t buttonTick = GetTickCount();
	uint32_t ledTick1 = GetTickCount();
	uint32_t ledTick2 = ledTick1;

	uint32_t versiondata;
	uint32_t st;

//------------------------------------------------------------------------------
//Передача приветствия в USART
	Usart2_SendData(s_pc_hello,strlen(s_pc_hello));
	Usart2_SendData(s_pc_scanner,strlen(s_pc_scanner));
	Usart2_SendData(s_pc_test,strlen(s_pc_test));
//Передача приветствия на экран
	LCD_RunStringDevInfo ( s_lcd_hello,s_lcd_scanner,RUN_STR_DELAY_TST);
//********************************
//Инициализация и тест NFC PN532
	b_wakeup=PN532_WakeUp();
	b_samconfig=PN532_SAMConfig();
/*
	if(b_samconfig){
		 Usart2_SendData(s_pn_samconfig_ok,strlen(s_pn_samconfig_ok));
	}
	else{
		 Usart2_SendData(s_pn_samconfig_err,strlen(s_pn_samconfig_err));

	}
*/
	versiondata = PN532_getFirmwareVersion();//Получениe версии ПО PN532

	if(versiondata){
		statusDevice=statusDevice|ST_DEV_NFC_NOERR;//установка флага успешного теста NFC
		PN532_FirmWareVersion2String (versiondata);
		b_rertyes=PN532_setPassiveActivationRetries(0xFF);
		b_discannfc=0;
	}

//********************************
	//Delay(1000);
//********************************
//Tест Scanner barcode E1005&E1006
 b_getversion=ENABLE;
 cntbyteversion=0;

 SCAN_SendCMD(cmd_Version,STR_CMD_LENGTH);//Запрос версии  ПО сканера штрих-кодов

uint32_t tick1 = 0;
uint32_t tick2 = 0;

	tick1=GetTickCount();
	tick2=tick1;
	while(b_getversion || (tick2 - tick1 > 1000)){
		tick2=GetTickCount();
	}

	if(!b_getversion){//Получена версия сканера штрих-кодов
		statusDevice=statusDevice|ST_DEV_BAR_NOERR;//установка флага успешного теста сканера штрих-кодов
	}

switch(statusDevice){
	case ST_DEV_ALL_ERR://Оба устройства не ответили

		Usart2_SendData(s_lcd_bar_err,strlen(s_pc_barcode));
		Usart2_SendData(s_newline,strlen(s_newline));

		Usart2_SendData(s_lcd_nfc_err,sizeof(s_pn_nfchip));
		Usart2_SendData(s_newline,strlen(s_newline));

		LCD_RunStringDevInfo ( s_lcd_bar_err,s_lcd_nfc_err,RUN_STR_DELAY_TST);

	break;

	case ST_DEV_NFC_NOERR://Ответил только NFC

		Usart2_SendData(s_lcd_bar_err,strlen(s_pc_barcode));
		Usart2_SendData(s_newline,strlen(s_newline));

	    Usart2_SendData(s_pn_fchip,strlen(s_pn_fchip));
	    Usart2_SendData(s_pn_nchip,sizeof(s_pn_nchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

		Usart2_SendData(s_pn_wfchip,strlen(s_pn_wfchip));
		Usart2_SendData(s_pn_fw1nchip,sizeof(s_pn_fw1nchip));
		Usart2_SendData(s_pn_point,strlen(s_pn_point));
		Usart2_SendData(s_pn_fw2nchip,sizeof(s_pn_fw2nchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

		Usart2_SendData(s_pn_suppchip,sizeof(s_pn_suppchip));
		Usart2_SendData(s_pn_suchip,sizeof(s_pn_suchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));


		LCD_PN532_CreateRunInfoString();
		LCD_RunStringDevInfo ( s_lcd_bar_err,s_pn_info,RUN_STR_DELAY_TST);

	break;

	case ST_DEV_BAR_NOERR://Ответил только сканер штрих-кодов

		Usart2_SendData(s_pc_barcode,strlen(s_pc_barcode));
		Usart2_SendData(ScanerVersion,strlen(ScanerVersion));
		Usart2_SendData(s_newline,strlen(s_newline));

		Usart2_SendData(s_lcd_nfc_err,sizeof(s_pn_nfchip));
		Usart2_SendData(s_newline,strlen(s_newline));

		LCD_BAR_CreateRunInfoString();
		LCD_RunStringDevInfo ( s_bar_info,s_lcd_nfc_err,RUN_STR_DELAY_TST);

//		Usart2_SendData(s_bar_info,strlen(s_bar_info));
//		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

	break;

	case ST_DEV_ALL_NOERR://Оба устройства ответили

		Usart2_SendData(s_pc_barcode,strlen(s_pc_barcode));
		Usart2_SendData(ScanerVersion,strlen(ScanerVersion));
		Usart2_SendData(s_newline,strlen(s_newline));

	    Usart2_SendData(s_pn_fchip,strlen(s_pn_fchip));
	    Usart2_SendData(s_pn_nchip,sizeof(s_pn_nchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

		Usart2_SendData(s_pn_wfchip,strlen(s_pn_wfchip));
		Usart2_SendData(s_pn_fw1nchip,sizeof(s_pn_fw1nchip));
		Usart2_SendData(s_pn_point,strlen(s_pn_point));
		Usart2_SendData(s_pn_fw2nchip,sizeof(s_pn_fw2nchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

		Usart2_SendData(s_pn_suppchip,sizeof(s_pn_suppchip));
		Usart2_SendData(s_pn_suchip,sizeof(s_pn_suchip));
		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));


		LCD_BAR_CreateRunInfoString();
		LCD_PN532_CreateRunInfoString();
		LCD_RunStringDevInfo ( s_bar_info,s_pn_info,RUN_STR_DELAY_TST);

//		Usart2_SendData(s_bar_info,strlen(s_bar_info));
//		Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

		LCD_SetCursor ( LCD_CURSOR_1STR );
		LCD_WriteString (s_lcd_scanner_rdy );

		LCD_SetCursor ( LCD_CURSOR_2STR );
		LCD_WriteString (s_lcd_ready );

		Usart2_SendData(s_pc_scanner_ready,sizeof(s_pc_scanner_ready));


	break;
}

/*
	FLASH_Write(0xFFFFFFFF);
	flash_test = FLASH_Read(flash_addr1);
	Usart2_SendData((char*)flash_test,sizeof(flash_test));
	Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));



/*
Usart2_SendData(s_lcd_scanner,strlen(s_lcd_scanner));
Usart2_SendData(s_lcd_ready,strlen(s_lcd_ready));
Usart2_SendData(s_newline,strlen(s_newline));
*/

	//LCD_SetCursor ( LCD_CURSOR_2STR );
	//LCD_WriteString (s_lcd_ready );

//------------------------------------------------------------------------------
Timer1Init();
EnableTimer1Interrupt();


	while (1) {
		char cSymbol;

		char date_Symbol;





	//	GPIO_WriteBit(PA7_GPIO_PORT, PA7_PIN, 0);

		if(timer1flag==1){
			ValueInterrupToStr();
			Usart2_SendData(b_bounce_value,sizeof(b_bounce_value));
			Usart2_SendData(s_newline,strlen(s_newline));
			timer1flag=0;
		}

	//	GPIO_WriteBit(PA7_GPIO_PORT, PA7_PIN, 1);

/*****************************************************************************/
		/*
		 * Считывание данных от сканера из циклического буфера FIFO
		 *  в линейный буфер g_sBuffer[]
		 */
		//GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 1);
//если не режим записи метки NFC
if(u8_wrnfcstatus==0){

		while (FIFO_GetNextData ( &cSymbol ) == SUCCESS) {
			g_sBuffer [ g_iBufferLength++ ] = cSymbol;
			g_sBuffer [ g_iBufferLength ] = '\0';
			if ( g_iBufferLength >= BUFFER_LEN ) {
				memmove ( g_sBuffer, g_sBuffer + 1, BUFFER_LEN );
				g_iBufferLength--;
			}
		}
/*	прием данных с телефона
		while (Date_GetNextDate(&date_Symbol) == SUCCESS){
			date_sBuffer [ date_iBufferLength++ ] = date_Symbol;
			date_sBuffer [ date_iBufferLength] = '\0';
			if( date_iBufferLength >= BUFFER_LEN){
				memmove(date_sBuffer, date_sBuffer + 1, BUFFER_LEN);
				date_iBufferLength--;
			}

		}
*/
/*	декодирование данных с телефона
		if(strstr(date_sBuffer, "\r\n" != NULL)){
			strtok(date_sBuffer, "\r\n");
			char *dBuffer = date_sBuffer;

		}
*/

		//GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 0);

		/*
		 * Декодирование данных сканера
		 */
		if (strstr(g_sBuffer, "\r\n") != NULL) {
			/* Выделение лексемы,
			 *  ограниченной справа одним из символов множества "\r\n"
			 */
			strtok ( g_sBuffer, "\r\n" );

			char * pBuffer = g_sBuffer;
			while (strpbrk ( pBuffer, "\n\xFF" ) != NULL) {
				pBuffer++;
			}

			memset(g_sBarCode,0,strlen(g_sBarCode));
			  strlcpy ( g_sBarCode, (const char *)s_lcd_barcode_read, USB_STATE_LEN );
			  strcat(g_sBarCode,pBuffer );
			//strlcpy ( g_sBarCode, (const char *)pBuffer, USB_STATE_LEN );
/*
			Usart2_SendData(s_pc_barcode,strlen(s_pc_barcode));
			Usart2_SendData(s_newline,strlen(s_newline));
			Usart2_SendData(g_sBarCode,strlen(g_sBarCode));
			Usart2_SendData(s_newline,strlen(s_newline));
*/
			//LCD_SetCursor ( LCD_CURSOR_1STR );
			//LCD_WriteString ( s_lcd_barcode );

			//if(b_firstrddata==DISABLE){
				LCD_SetCursor ( LCD_CURSOR_1STR );
				LCD_WriteString ( s_lcd_clrstr );
				LCD_SetCursor ( LCD_CURSOR_2STR );
				LCD_WriteString ( s_lcd_clrstr );
				//b_firstrddata=ENABLE;

			//}

			LCD_RunStringBARCode ( g_sBarCode, RUN_STR_DELAY, 0 );

			int strLength = strlen(g_sBuffer) + 1;
			g_iBufferLength -= strLength;
			memmove ( g_sBuffer, g_sBuffer + strLength,

					BUFFER_LEN + 1 - strLength );

			timer = RTC_GET_COUNTER();
			timer_to_cal(timer, &rtc_time);
			tim = cal_to_timer(&rtc_time);


/*
			s_cal_sec[0] = 0x30 + (unix_time.sec/10);
			s_cal_sec[1] = 0x30 + (unix_time.sec%10);
			s_cal_min[0] = 0x30 + (unix_time.min/10);
			s_cal_min[1] = 0x30 + (unix_time.min%10);
			s_cal_hour[0] = 0x30 + (unix_time.hour/10);
			s_cal_hour[1] = 0x30 + (unix_time.hour%10);
*/

			s_cal_data[0] = 0x30 + (rtc_time.mday/10);
			s_cal_data[1] = 0x30 + (rtc_time.mday%10);
			s_cal_data[3] = 0x30 + (rtc_time.mon/10);
			s_cal_data[4] = 0x30 + (rtc_time.mon%10);
			s_cal_data[6] = 0x30 + (rtc_time.year/1000%10);
			s_cal_data[7] = 0x30 + (rtc_time.year/100%10);
			s_cal_data[8] = 0x30 + (rtc_time.year/10%10);
			s_cal_data[9] = 0x30 + (rtc_time.year%10);
			s_cal_data[11] = 0x30 + (rtc_time.hour/10);
			s_cal_data[12] = 0x30 + (rtc_time.hour%10);
			s_cal_data[14] = 0x30 + (rtc_time.min/10);
			s_cal_data[15] = 0x30 + (rtc_time.min%10);
			s_cal_data[17] = 0x30 + (rtc_time.sec/10);
			s_cal_data[18] = 0x30 + (rtc_time.sec%10);


			strncat(s_scan_d_c, s_cal_data, 19);
			strncat(s_scan_d_c, s_space, strlen(s_space));
			strncat(s_scan_d_c, g_sBarCode, strlen(g_sBarCode));
			strncat(s_scan_d_c, s_newline, strlen(s_newline));



		//	Usart2_SendData(s_cal_data,strlen(s_cal_data));
			Usart2_SendData(s_scan_d_c,strlen(s_scan_d_c));
			Usart2_SendData(s_newline,strlen(s_newline));

			memset(s_scan_d_c, 0, strlen(s_scan_d_c));
		}



		//*****************************************************************************
		//Если тест NFC пройден успешно,то сканируем ID
				if(statusDevice==ST_DEV_NFC_NOERR || statusDevice==ST_DEV_ALL_NOERR ){
					if(cntlooppn532==CNT_LOOP_PN532){

						success=PN532_readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength,1500);

						if(success && !b_discannfc){

							b_discannfc=1;

							s_pn_uidlengthdata[0] = 0x30+(uidLength/100);
							s_pn_uidlengthdata[1] = 0x30+(uidLength/10%10);
							s_pn_uidlengthdata[2] = 0x30+(uidLength%10);

							Usart2_SendData(s_pn_foundcard,strlen(s_pn_foundcard));

							Usart2_SendData(s_pn_uidlength,strlen(s_pn_uidlength));
							Usart2_SendData(s_pn_uidlengthdata,strlen(s_pn_uidlengthdata));
							Usart2_SendData(s_newline,strlen(s_newline));

							b_hextostr = fourbytes2strhex (&uid[0], &uidhex2pc4b[0]);

							Usart2_SendData(s_pn_uidvalue,strlen(s_pn_uidvalue));
							Usart2_SendData(uidhex2pc4b,strlen(uidhex2pc4b));
							Usart2_SendData(s_newline,strlen(s_newline));

							decuid = UIDResponse(&uid[0], uidLength);

							s_pn_uiddecdata[0] = 0x30+(decuid/1000000000);
							s_pn_uiddecdata[1] = 0x30+(decuid/100000000%10);
							s_pn_uiddecdata[2] = 0x30+(decuid/10000000%10);
							s_pn_uiddecdata[3] = 0x30+(decuid/1000000%10);
							s_pn_uiddecdata[4] = 0x30+(decuid/100000%10);
							s_pn_uiddecdata[5] = 0x30+(decuid/10000%10);
							s_pn_uiddecdata[6] = 0x30+(decuid/1000%10);
							s_pn_uiddecdata[7] = 0x30+(decuid/100%10);
							s_pn_uiddecdata[8] = 0x30+(decuid/10%10);
							s_pn_uiddecdata[9] = 0x30+(decuid%10);

							Usart2_SendData(s_pn_uiddecvalue,strlen(s_pn_uiddecvalue));
							Usart2_SendData(s_pn_uiddecdata,strlen(s_pn_uiddecdata));
							Usart2_SendData(s_newline,strlen(s_newline));

							//memset(g_sBarCode,0,strlen(g_sBarCode));
							//strlcpy ( g_sBarCode, (const char *)s_pn_uiddecdata,strlen(s_pn_uiddecdata)+1 );


							//if(b_firstrddata==DISABLE){
								LCD_SetCursor ( LCD_CURSOR_1STR );
								LCD_WriteString ( s_lcd_clrstr );
								LCD_SetCursor ( LCD_CURSOR_2STR );
								LCD_WriteString ( s_lcd_clrstr );
								//b_firstrddata=ENABLE;

							//}

							memset(g_sNfcCode,0,strlen(g_sNfcCode));

							  strlcpy ( g_sNfcCode, (const char *)s_lcd_nfc_read, USB_STATE_LEN );
							  strcat(g_sNfcCode, s_pn_uiddecdata );

							//strlcpy ( g_sNfcCode, (const char *)s_pn_uiddecdata,strlen(s_pn_uiddecdata)+1 );

							//LCD_SetCursor ( LCD_CURSOR_1STR );
							//LCD_WriteString ( s_lcd_nfc );
							LCD_RunStringNFCCode ( g_sNfcCode, RUN_STR_DELAY, 0 );


						//	PN532_WriteID();
							pn532_rd_stat=PN532_Read_Data(uid,uidLength,1,  rdDataBlock);

							switch(pn532_rd_stat){
							case 0:
								Usart2_SendData(s_pc_PN532_info1,strlen(s_pc_PN532_info1));
								Usart2_SendData(s_newline,strlen(s_newline));
								break;
							case 1:
								Usart2_SendData(s_pc_PN532_err1,strlen(s_pc_PN532_err1));
								Usart2_SendData(s_newline,strlen(s_newline));
								break;
							case 2:
								Usart2_SendData(s_pc_PN532_err4,strlen(s_pc_PN532_err4));
								Usart2_SendData(s_newline,strlen(s_newline));
								break;
							}


							/*
							if(pn532_rd_stat==1){
								Usart2_SendData(s_pc_PN532_err1,strlen(s_pc_PN532_err1));
								Usart2_SendData(s_newline,strlen(s_newline));
							}
						*/

							Usart2_SendData(s_pn_data_block0,strlen(s_pn_data_block0));


							//if(pn532_rd_stat==0){
							BytesHex2Str(rdDataBlock, sizeof(rdDataBlock), DataHex2pc);
								Usart2_SendData(DataHex2pc,sizeof(DataHex2pc));
								Usart2_SendData(s_newline,strlen(s_newline));

							//}

						}
						if(!success)
						{

							//Usart2_SendData(s_pn_notfoundcard,sizeof(s_pn_notfoundcard));
							b_discannfc=0;

						}

						cntlooppn532=0;
					}//конец if(cntlooppn532==CNT_LOOP_PN532)

					cntlooppn532++;

					LCD_RunStringNFCCode ( g_sNfcCode, RUN_STR_DELAY, 1 );

				}//конец if(statusDevice==ST_DEV_NFC_NOERR || statusDevice==ST_DEV_ALL_NOERR )
		//******************************************************************************




}//конец если не режим записи метки NFC

else{ //если режим записи метки NFC


	if(!b_1secflg){
		LCD_SetCursor ( LCD_CURSOR_2STR );
		LCD_WriteString (uidnfcwrdara );

	}
	else{
		LCD_SetCursor ( LCD_CURSOR_2STR+u8_cntindex );
		LCD_WriteString (s_space);

	}
}


if(u8_wrnfcstatus!=0 && b_wrstr1==TRUE){
	   memset(g_sBarCode,0,strlen(g_sBarCode));
	  strlcpy ( g_sBarCode, (const char *)s_lcd_wrnfc, USB_STATE_LEN );
	  LCD_RunStringBARCode ( g_sBarCode, RUN_STR_DELAY, 0 );
	  b_wrstr1=FALSE;
}

LCD_RunStringBARCode ( g_sBarCode, RUN_STR_DELAY, 1 );

if(u8_wrnfcstatus==9){
	PN532_WriteTagtoHex();

	//b_hextostr = fourbytes2strhex (&outwrdata[0], &uidhex2pc4b[0]);

	Usart2_SendData(s_pn_uidvalue,strlen(s_pn_uidvalue));
	Usart2_SendData(outwrdata,sizeof(outwrdata));
	Usart2_SendData(s_newline,strlen(s_newline));

	pn532_wr_stat=PN532_Write_Data(uid,uidLength, 1, outwrdata);

	switch(pn532_wr_stat){
	case 0:
		Usart2_SendData(s_pc_PN532_info2,strlen(s_pc_PN532_info2));
		Usart2_SendData(s_newline,strlen(s_newline));
		break;
	case 1:
		Usart2_SendData(s_pc_PN532_err1,strlen(s_pc_PN532_err1));
		Usart2_SendData(s_newline,strlen(s_newline));
		break;
	case 2:
		Usart2_SendData(s_pc_PN532_err4,strlen(s_pc_PN532_err4));
		Usart2_SendData(s_newline,strlen(s_newline));
		break;
	case 3:
		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
		Usart2_SendData(s_newline,strlen(s_newline));
		break;
	case 4:
		Usart2_SendData(s_pc_PN532_err3,strlen(s_pc_PN532_err3));
		Usart2_SendData(s_newline,strlen(s_newline));
		break;


	}

	//PN532_WriteID();

	u8_wrnfcstatus=0;

	   memset(g_sBarCode,0,strlen(g_sBarCode));
	  strlcpy ( g_sBarCode, (const char *)s_lcd_scanner_rdy, USB_STATE_LEN );
	  LCD_RunStringBARCode ( g_sBarCode, RUN_STR_DELAY, 0 );


	//LCD_SetCursor ( LCD_CURSOR_1STR );
//	LCD_WriteString (s_lcd_scanner_rdy );

	LCD_SetCursor ( LCD_CURSOR_2STR );
	LCD_WriteString (s_lcd_ready );
}


		/*
		 * Обработка состояния кнопки
		 */

//		if (GPIO_ReadInputDataBit ( BTN_GPIO_PORT, BTN_PIN ) == Bit_RESET) {
//
//			if (btnpress==0/*buttonState == UNPRESSED*/) {
//				btnpress=1;
//				cntbutpress++;
////				buttonState = PRESSED;
//				//Если тест сканера штрих-кодов пройден успешно,то сканируем ID
//				if(statusDevice==ST_DEV_BAR_NOERR || statusDevice==ST_DEV_ALL_NOERR ){
//				 SCAN_SendCMD(cmd_TriggerScann,STR_CMD_LENGTH);
//				}
//				uint8_t pn532_dataResult;
//				uint8_t pn532_cmd[] = {0x8E, 0x6F, 0x23, 0x84, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0xA5, 0x11, 0xBF, 0x0C, 0x0E, 0x61, 0x0C, 0x4F, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10, 0x87, 0x01, 0x01, 0x90, 0x00};
//				uint8_t pn532getdata;
////тест записи и чтения nfc
//				switch(cntbutpress){
//				case 1:
//
//					pn532_dataResult = PN532_setDataTarget(pn532_cmd, sizeof(pn532_cmd));
//
//				break;
//
//				case 2:
//					//pn532getdata= PN532_getDataTarget(pn532_rddata, pn532_rdlen);
//					//Usart2_SendData(pn532_rddata, pn532_rdlen);
//					cntbutpress=0;
//					break;
//				}
//
//
////				USART_SendData(USART2_NUM, 0x39);
//				buttonTick = GetTickCount();
//
//
//
//			}
//
//
//		} else {//если кнопка не нажата (1)
//
//
////			if ((buttonState == PRESSED) && ((GetTickCount() - buttonTick) > 1000)) {
////				buttonState = UNPRESSED;USART_SendData(USART2_NUM, 0x33);
////			}
//
//			if ((btnpress == 1) && ((GetTickCount() - buttonTick) > 1000)) {
//				btnpress = 0;//USART_SendData(USART2_NUM, 0x33);
//			}
//
//
//		}









	}//конец while(1)


}
//******************************************************************
void ValueInterrupToStr(){
	b_bounce_value[0] = 0x30+(testtimer1/10000%10);
	b_bounce_value[1] = 0x30+(testtimer1/1000%10);
	b_bounce_value[2] = 0x30+(testtimer1/100%10);
	b_bounce_value[3] = 0x30+(testtimer1/10%10);
	b_bounce_value[4] = 0x30+(testtimer1%10);

}
//******************************************************************
void SCAN_SendCMD (char* strcmd,int sz_cmd){

	for(int ik=0;ik<sz_cmd;ik++){

		 while (READ_BIT(USART_NUM->SR, USART_SR_TXE) != (USART_SR_TXE)) {}

		USART_SendData(USART_NUM, strcmd[ik]);
	}


}

//******************************************************************
void SCAN_SumToString ( int sum, char * sumStr ) {
	int div = SUM_MAX / 10;
	uint8_t i = 0;

	sum = (sum >= SUM_MAX) ? (SUM_MAX - 1) : sum;
	sum = (sum < 0) ? 0 : sum;

	while (div) {
		sumStr[i++] = sum / div + '0';
		sum %= div;
		div /= 10;
	}
}
//******************************************************************
void LCD_CtrlLinesConfig(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
	I2C_InitTypeDef			I2C_InitStruct;

	/*
	 * I2C pins configure
	 */
	LCD_I2C_SCL_RCC_PERIPH_CLOCK_CMD ( LCD_I2C_SCL_RCC_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = LCD_I2C_SCL_PIN;
	GPIO_Init ( LCD_I2C_SCL_GPIO_PORT, &GPIO_InitStruct );

	LCD_I2C_SDA_RCC_PERIPH_CLOCK_CMD ( LCD_I2C_SDA_RCC_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin = LCD_I2C_SDA_PIN;
	GPIO_Init ( LCD_I2C_SDA_GPIO_PORT, &GPIO_InitStruct );

	LCD_I2C_RCC_PERIPH_CLOCK_CMD ( LCD_I2C_RCC_APBPORT, ENABLE );

	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 50000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 1;
	I2C_Init ( LCD_I2C_NUM, &I2C_InitStruct );

	I2C_Cmd ( LCD_I2C_NUM, ENABLE );
}
//******************************************************************
/**
  * @brief  Writes a byte at a specific LCD register
  * @param  Device: device address
  * @param  Addr: register address
  * @param  Data: data to be written to the specific register
  * @retval 0x00 if write operation is OK
  *         0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t I2C_SingleRandomWrite ( I2C_TypeDef* I2Cx, uint8_t Device, uint8_t Data ) {
	uint32_t timeout = TIMEOUT_MAX;

	/* Generate the Start Condition */
	I2C_GenerateSTART ( I2Cx, ENABLE );

	/* Test on I2Cx EV5 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_MODE_SELECT )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send TS selected device slave Address for write */
	I2C_Send7bitAddress ( I2Cx, Device, I2C_Direction_Transmitter );

	/* Test on I2Cx EV6 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send Data */
	I2C_SendData ( I2Cx, Data );

	/* Test on I2Cx EV8 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP ( I2Cx, ENABLE );

	/* If operation is OK, return 0 */
	return 0;
}
//******************************************************************
void LCD_SetCursor ( uint8_t index ) {
	uint8_t index_hi = index & 0xF0;
	uint8_t index_lo = (index & 0x0F) << 4;

	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_hi & ~LCD_E) & ~LCD_RS & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_hi | LCD_E) & ~LCD_RS & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_hi & ~LCD_E) & ~LCD_RS & ~LCD_RW );

	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_lo & ~LCD_E) & ~LCD_RS & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_lo | LCD_E) & ~LCD_RS & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			(index_lo & ~LCD_E) & ~LCD_RS & ~LCD_RW );
}
//******************************************************************
void LCD_WriteData ( uint8_t data ) {
	uint8_t data_hi = data & 0xF0;
	uint8_t data_lo = (data & 0x0F) << 4;

	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_hi & ~LCD_E) | LCD_RS) & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_hi | LCD_E) | LCD_RS) & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_hi & ~LCD_E) | LCD_RS) & ~LCD_RW );

	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_lo & ~LCD_E) | LCD_RS) & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_lo | LCD_E)  | LCD_RS) & ~LCD_RW );
	I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
			((data_lo & ~LCD_E) | LCD_RS) & ~LCD_RW );
}
//******************************************************************
void LCD_WriteString ( char * str ) {
	uint8_t i;

	for (i = 0; i < strlen(str); i++) {
		LCD_WriteData ( str[i] );
	}

}
//******************************************************************
//******************************************************************
void LCD_RunStringDevInfo ( char * str1line,char * str2line, uint32_t delay) {
	static uint8_t strFrameIndex = 0;
	static uint32_t tick = 0;
	static char runStr1[100 + 2 * LCD_STR_LEN + 1];
	static char runStr2[100 + 2 * LCD_STR_LEN + 1];
	static uint8_t runStr1Len, runStr2Len;
	char str1Frame[LCD_STR_LEN + 1];
	char str2Frame[LCD_STR_LEN + 1];
	uint8_t i;
	uint8_t cnt;

		tick = GetTickCount();
		strFrameIndex = 0;
		cnt=0;

		/*
		 * Копирование исходной строки в буфер
		 *  с учётом добавления с двух сторон пробелов
		 */
		*runStr1 = '\0';
		*runStr2 = '\0';

		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr1, " " );
			strcat ( runStr2, " " );
		}
		strcat ( runStr1, str1line );
		strcat ( runStr2, str2line );

		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr1, " " );
			strcat ( runStr2, " " );
		}

		runStr1Len = strlen ( runStr1 );
		runStr2Len = strlen ( runStr2 );

	while(cnt<(runStr1Len-1)||cnt<(runStr2Len-1))
		if (GetTickCount() - tick > delay) {
			tick = GetTickCount();
			strFrameIndex++;

			if (cnt<(runStr1Len-1)) {
			for (i = 0; i < LCD_STR_LEN; i++)
				str1Frame[i] = runStr1[(strFrameIndex + i) % runStr1Len];

			LCD_SetCursor ( LCD_CURSOR_1STR );//вывод в первую строку строку
			LCD_WriteString ( str1Frame );
			}

			if (cnt<(runStr2Len-1)){
			for (i = 0; i < LCD_STR_LEN; i++)
				str2Frame[i] = runStr2[(strFrameIndex + i) % runStr2Len];

			LCD_SetCursor ( LCD_CURSOR_2STR );//вывод во вторую строку
			LCD_WriteString ( str2Frame );
			}

			cnt++;
		}

}
//******************************************************************
//******************************************************************
//Отображение кода сканера штрих-кодов бегущей строкой в первой строчке LCD
void LCD_RunStringBARCode ( char * str, uint32_t delay, uint8_t start ) {
	static uint8_t strFrameIndex = 0;
	static uint32_t tick = 0;
	static char runStr[USB_STATE_LEN + 2 * LCD_STR_LEN + 1];
	static uint8_t runStrLen;
	char strFrame[LCD_STR_LEN + 1];
	uint8_t i;

	if (start == 0) {
		tick = GetTickCount();
		strFrameIndex = 0;

		/*
		 * Копирование исходной строки в буфер
		 *  с учётом добавления с двух сторон пробелов
		 */
		*runStr = '\0';

		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr, " " );
		}
		strcat ( runStr, str );
		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr, " " );
		}
		runStrLen = strlen ( runStr );
	}

	if (strlen(str)) {
		if (GetTickCount() - tick > delay) {
			tick = GetTickCount();
			strFrameIndex++;
			for (i = 0; i < LCD_STR_LEN; i++) {
				strFrame[i] = runStr[(strFrameIndex + i) % runStrLen];
			}
			LCD_SetCursor ( LCD_CURSOR_1STR );//вывод в первую строку
			LCD_WriteString ( strFrame );
		}
	}
}
//******************************************************************
//******************************************************************
//Отображение кода сканера NFC бегущей строкой во второй строчке LCD
void LCD_RunStringNFCCode ( char * str, uint32_t delay, uint8_t start ) {
	static uint8_t strFrameIndex = 0;
	static uint32_t tick = 0;
	static char runStr[USB_STATE_LEN + 2 * LCD_STR_LEN + 1];
	static uint8_t runStrLen;
	char strFrame[LCD_STR_LEN + 1];
	uint8_t i;

	if (start == 0) {
		tick = GetTickCount();
		strFrameIndex = 0;

		/*
		 * Копирование исходной строки в буфер
		 *  с учётом добавления с двух сторон пробелов
		 */
		*runStr = '\0';

		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr, " " );
		}
		strcat ( runStr, str );
		for (i = 0; i < (LCD_STR_LEN - 1); i++) {
			strcat ( runStr, " " );
		}
		runStrLen = strlen ( runStr );
	}

	if (strlen(str)) {
		if (GetTickCount() - tick > delay) {
			tick = GetTickCount();
			strFrameIndex++;
			for (i = 0; i < LCD_STR_LEN; i++) {
				strFrame[i] = runStr[(strFrameIndex + i) % runStrLen];
			}
			LCD_SetCursor ( LCD_CURSOR_2STR );//вывод во вторую строку
			LCD_WriteString ( strFrame );
		}
	}
}
//******************************************************************

void LCD_Init(void) {
	uint8_t i;

	LCD_CtrlLinesConfig();

	for (i = 0; i < sizeof(g_arInitDisplayCommands) / sizeof(uint8_t); i++) {
		I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
				((g_arInitDisplayCommands[i] << 4) & ~LCD_E) & ~LCD_RS & ~LCD_RW );
		Delay(10);
		I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
				((g_arInitDisplayCommands[i] << 4) | LCD_E) & ~LCD_RS & ~LCD_RW );
		Delay(10);
		I2C_SingleRandomWrite ( LCD_I2C_NUM, LCD_I2C_ADDRESS,
				((g_arInitDisplayCommands[i] << 4) & ~LCD_E) & ~LCD_RS & ~LCD_RW );
		Delay(10);
	}
	/*
	 * Очистка дисплея
	 */
	LCD_SetCursor ( LCD_CURSOR_1STR );
	LCD_WriteString ( "                   " );
	LCD_SetCursor ( LCD_CURSOR_2STR );
	LCD_WriteString ( "                   " );
	//LCD_WriteString ( "SUM:               " );
	/*
	 * Отображение на дисплее текущей суммы
	 */
	//LCD_DisplaySum();
}
//******************************************************************
void LCD_DisplaySum ( void ) {
	uint8_t i = SUM_STR_LEN - 3;

	SCAN_SumToString ( g_iSum, g_sSum );
	memmove ( g_sSum + SUM_STR_LEN - 1, g_sSum + SUM_STR_LEN - 2, 2 );
	g_sSum[SUM_STR_LEN - 2] = '.';

	do {
		if (*g_sSum == '0') {
			memmove ( g_sSum, g_sSum + 1, SUM_STR_LEN + 1 );
		} else break;
	} while (--i);

	//LCD_SetCursor ( 0xC0 );
	//LCD_WriteString ("*");

	LCD_SetCursor ( 0xC8 - strlen(g_sSum) );
	LCD_WriteString ( g_sSum );
}


/**
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
  assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
  assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));
  assert_param(IS_FUNCTIONAL_STATE(EXTI_InitStruct->EXTI_LineCmd));

  tmp = (uint32_t)EXTI_BASE;

  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;

    tmp += EXTI_InitStruct->EXTI_Mode;

    *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;

    /* Select the trigger for the selected external interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    {
      /* Rising Falling edge */
      EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
      EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      tmp = (uint32_t)EXTI_BASE;
      tmp += EXTI_InitStruct->EXTI_Trigger;

      *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    tmp += EXTI_InitStruct->EXTI_Mode;

    /* Disable the selected external lines */
    *(__IO uint32_t *) tmp &= ~EXTI_InitStruct->EXTI_Line;
  }
}

/**
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  EXTI_Line: specifies the EXTI line flag to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The new state of EXTI_Line (SET or RESET).
  */
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));

  if ((EXTI->PR & EXTI_Line) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the EXTI's line pending bits.
  * @param  EXTI_Line: specifies the EXTI lines to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
void EXTI_ClearITPendingBit(uint32_t EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));

  EXTI->PR = EXTI_Line;
}


//******************************************************************
void BTN_Init(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
//	EXTI_InitTypeDef		EXTI_InitStruct;
//	NVIC_InitTypeDef		NVIC_InitStruct;

	/*
	 * EXTI pin configure
	 */
	BTN_RCC_PERIPH_CLOCK_CMD ( BTN_RCC_GPIO_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = BTN_PIN;
	GPIO_Init ( BTN_GPIO_PORT, &GPIO_InitStruct );

//	/* Enable AFIO clock */				//комментирование блока инициализации для кнопки с прерыванием
//	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_AFIO, ENABLE );
////
////	/* Connect EXTI0 Line to PB.0 pin */
//	GPIO_EXTILineConfig ( BTN_EXTI_PORTSOURCE, BTN_EXTI_PINSOURCE );
////
//	EXTI_InitExtiStruct.EXTI_Line = BTN_EXTI_LINE;
//	EXTI_InitExtiStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitExtiStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitExtiStruct.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;
//	EXTI_Init ( &EXTI_InitExtiStruct );
//
//	NVIC_PriorityGroupConfig ( NVIC_GROUP );
//	NVIC_InitExtiStruct.NVIC_IRQChannel = BTN_IRQ;
//	NVIC_InitExtiStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitExtiStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitExtiStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init ( &NVIC_InitExtiStruct );
}
//******************************************************************
void BTN_IRQHandler ( void ) {
	if (EXTI_GetFlagStatus ( BTN_EXTI_LINE ) == SET) {
		if(u8_btnstate==BTN_UNPRESSED){

		  NVIC_InitExtiStruct.NVIC_IRQChannelCmd = DISABLE;
		  EXTI_InitExtiStruct.EXTI_Trigger = EXTI_Trigger_Rising;
		  NVIC_Init ( &NVIC_InitExtiStruct );
		  cnttimer1=0;
		  u8_btnstate=BTN_PRESSED;
		  EnableTimer1Interrupt();
		  GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 1);
		}
		else{
			//DisableTimer1Interrupt
			  NVIC_InitExtiStruct.NVIC_IRQChannelCmd = DISABLE;
			  EXTI_InitExtiStruct.EXTI_Trigger = EXTI_Trigger_Falling;
			  NVIC_Init ( &NVIC_InitExtiStruct );
			  timer1data=cnttimer1;
			  u8_btnstate=BTN_UNPRESSED;
			  GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 0);


		}
//		if(b_ledbtnstate==FALSE){
//
//		GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 1);
//		b_ledbtnstate=TRUE;
//		}
//		else{
//			GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 0);
//			b_ledbtnstate=FALSE;
//		}
		EXTI_ClearITPendingBit ( BTN_EXTI_LINE );


	}
}
//******************************************************************
//USART2 для передачи на компьютер
void Usart2Init(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
	USART_InitTypeDef		USART2_InitStruct;
	NVIC_InitTypeDef		NVIC_InitStruct;

	/* USART2 initialization: PA2 - USART2_TX, PA3 - USART2_RX */

	USART2_RCC_PERIPH_CLOCK_CMD ( USART2_RCC_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = USART2_TX_PIN;
	GPIO_Init ( USART2_GPIO_PORT, &GPIO_InitStruct );

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = USART2_RX_PIN;
	GPIO_Init ( USART2_GPIO_PORT, &GPIO_InitStruct );

	USART_RCC_PERIPH_CLOCK_CMD ( USART2_RCC_APBPORT, ENABLE );

	USART2_InitStruct.USART_BaudRate = 9600;//для работы с SPI2 только эта скорость подходит
	USART2_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART2_InitStruct.USART_Parity = USART_Parity_No;
	USART2_InitStruct.USART_StopBits = USART_StopBits_1;
	USART2_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init ( USART2_NUM, &USART2_InitStruct );

	USART_Cmd ( USART2_NUM, ENABLE );

	NVIC_PriorityGroupConfig ( NVIC_GROUP );
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQ;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init ( &NVIC_InitStruct );

	USART_ITConfig ( USART2_NUM, USART_IT_RXNE, ENABLE );

}
//******************************************************************


//******************************************************************
void Usart2_SendData (char* strcmd, int sz_cmd){
//Передача на копьютер до символа окончания строки (0x00).
	for(int ik=0;ik<sz_cmd;ik++){
	    //if(strcmd[ik]!=0x00){
 		  while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE) == RESET){}
		  USART_SendData(USART2_NUM, strcmd[ik]);
	    //}
	}

}

//******************************************************************
//******************************************************************
void SCAN_UsartInit(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
	USART_InitTypeDef		USART_InitStruct;
	NVIC_InitTypeDef		NVIC_InitStruct;

	/* USART1 initialization: PA9 - USART1_TX, PA10 - USART1_RX */

	USART_RCC_PERIPH_CLOCK_CMD ( USART_RCC_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = USART_TX_PIN;
	GPIO_Init ( USART_GPIO_PORT, &GPIO_InitStruct );

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = USART_RX_PIN;
	GPIO_Init ( USART_GPIO_PORT, &GPIO_InitStruct );

	USART_RCC_PERIPH_CLOCK_CMD ( USART_RCC_APBPORT, ENABLE );

	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init ( USART_NUM, &USART_InitStruct );

	USART_Cmd ( USART_NUM, ENABLE );

	NVIC_PriorityGroupConfig ( NVIC_GROUP );
	NVIC_InitStruct.NVIC_IRQChannel = USART_IRQ;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init ( &NVIC_InitStruct );

	USART_ITConfig ( USART_NUM, USART_IT_RXNE, ENABLE );
}
//******************************************************************
void LED_Init(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
//	EXTI_InitTypeDef		EXTI_InitStruct;
//	NVIC_InitTypeDef		NVIC_InitStruct;
/*
 /*
 /* конфигурация вывода PB13 на активный выход */
//GPIO_InitStruct.Pin = GPIO_PIN_13; // номер вывода
//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // режим выход
//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; // средняя скорость выхода
//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	/*
	 * EXTI pin configure
	 */
	LED_RCC_PERIPH_CLOCK_CMD ( LED_RCC_GPIO_PORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_PIN;
	GPIO_Init ( LED_GPIO_PORT, &GPIO_InitStruct );
//#define LED_GPIO_PORT					GPIOA
//#define LED_PIN							GPIO_Pin_12


	GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 0);
//	/* Enable AFIO clock */
//	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_AFIO, ENABLE );
//
//	/* Connect EXTI0 Line to PB.0 pin */
//	GPIO_EXTILineConfig ( BTN_EXTI_PORTSOURCE, BTN_EXTI_PINSOURCE );
//
//	EXTI_InitStruct.EXTI_Line = BTN_EXTI_LINE;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_Init ( &EXTI_InitStruct );
//
//	NVIC_PriorityGroupConfig ( NVIC_GROUP );
//	NVIC_InitStruct.NVIC_IRQChannel = BTN_IRQ;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init ( &NVIC_InitStruct );
}



/**
  * @brief  Enables or disables the specified TIM peripheral.
  * @param  TIMx: where x can be 1 to 17 to select the TIMx peripheral.
  * @param  NewState: new state of the TIMx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the TIM Counter */
    TIMx->CR1 |= TIM_CR1_CEN;
  }
  else
  {
    /* Disable the TIM Counter */
    TIMx->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
  }
}

/**
  * @brief  Initializes the TIMx Time Base Unit peripheral according to
  *         the specified parameters in the TIM_TimeBaseInitStruct.
  * @param  TIMx: where x can be 1 to 17 to select the TIM peripheral.
  * @param  TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef
  *         structure that contains the configuration information for the
  *         specified TIM peripheral.
  * @retval None
  */
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  uint16_t tmpcr1 = 0;

  tmpcr1 = TIMx->CR1;

  if((TIMx == TIM1) || (TIMx == TIM8)|| (TIMx == TIM2) || (TIMx == TIM3)||
     (TIMx == TIM4) || (TIMx == TIM5))
  {
    /* Select the Counter Mode */
    tmpcr1 &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR | TIM_CR1_CMS)));
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_CounterMode;
  }

  if((TIMx != TIM6) && (TIMx != TIM7))
  {
    /* Set the clock division */
    tmpcr1 &= (uint16_t)(~((uint16_t)TIM_CR1_CKD));
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_ClockDivision;
  }

  TIMx->CR1 = tmpcr1;

  /* Set the Autoreload value */
  TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period ;

  /* Set the Prescaler value */
  TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;

  if ((TIMx == TIM1) || (TIMx == TIM8)|| (TIMx == TIM15)|| (TIMx == TIM16) || (TIMx == TIM17))
  {
    /* Set the Repetition Counter value */
    TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
  }

  /* Generate an update event to reload the Prescaler and the Repetition counter
     values immediately */
  TIMx->EGR = TIM_PSCReloadMode_Immediate;
}
/**
  * @brief  Enables or disables the specified TIM interrupts.
  * @param  TIMx: where x can be 1 to 17 to select the TIMx peripheral.
  * @param  TIM_IT: specifies the TIM interrupts sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg TIM_IT_Update: TIM update Interrupt source
  *     @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *     @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *     @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *     @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *     @arg TIM_IT_COM: TIM Commutation Interrupt source
  *     @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *     @arg TIM_IT_Break: TIM Break Interrupt source
  * @note
  *   - TIM6 and TIM7 can only generate an update interrupt.
  *   - TIM9, TIM12 and TIM15 can have only TIM_IT_Update, TIM_IT_CC1,
  *      TIM_IT_CC2 or TIM_IT_Trigger.
  *   - TIM10, TIM11, TIM13, TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.
  *   - TIM_IT_Break is used only with TIM1, TIM8 and TIM15.
  *   - TIM_IT_COM is used only with TIM1, TIM8, TIM15, TIM16 and TIM17.
  * @param  NewState: new state of the TIM interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    TIMx->DIER |= TIM_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    TIMx->DIER &= (uint16_t)~TIM_IT;
  }
}

/**
  * @brief  Checks whether the TIM interrupt has occurred or not.
  * @param  TIMx: where x can be 1 to 17 to select the TIM peripheral.
  * @param  TIM_IT: specifies the TIM interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg TIM_IT_Update: TIM update Interrupt source
  *     @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *     @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *     @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *     @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *     @arg TIM_IT_COM: TIM Commutation Interrupt source
  *     @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *     @arg TIM_IT_Break: TIM Break Interrupt source
  * @note
  *   - TIM6 and TIM7 can generate only an update interrupt.
  *   - TIM9, TIM12 and TIM15 can have only TIM_IT_Update, TIM_IT_CC1,
  *      TIM_IT_CC2 or TIM_IT_Trigger.
  *   - TIM10, TIM11, TIM13, TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.
  *   - TIM_IT_Break is used only with TIM1, TIM8 and TIM15.
  *   - TIM_IT_COM is used only with TIM1, TIM8, TIM15, TIM16 and TIM17.
  * @retval The new state of the TIM_IT(SET or RESET).
  */
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  ITStatus bitstatus = RESET;
  uint16_t itstatus = 0x0, itenable = 0x0;

  itstatus = TIMx->SR & TIM_IT;

  itenable = TIMx->DIER & TIM_IT;
  if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}
/**
  * @brief  Clears the TIMx's interrupt pending bits.
  * @param  TIMx: where x can be 1 to 17 to select the TIM peripheral.
  * @param  TIM_IT: specifies the pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg TIM_IT_Update: TIM1 update Interrupt source
  *     @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *     @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *     @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *     @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *     @arg TIM_IT_COM: TIM Commutation Interrupt source
  *     @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *     @arg TIM_IT_Break: TIM Break Interrupt source
  * @note
  *   - TIM6 and TIM7 can generate only an update interrupt.
  *   - TIM9, TIM12 and TIM15 can have only TIM_IT_Update, TIM_IT_CC1,
  *      TIM_IT_CC2 or TIM_IT_Trigger.
  *   - TIM10, TIM11, TIM13, TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.
  *   - TIM_IT_Break is used only with TIM1, TIM8 and TIM15.
  *   - TIM_IT_COM is used only with TIM1, TIM8, TIM15, TIM16 and TIM17.
  * @retval None
  */
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  /* Clear the IT pending Bit */
  TIMx->SR = (uint16_t)~TIM_IT;
}

//******************************************************************************
void Timer1Init(void)	{		//	(функция инициализации таймера)

//GPIO_InitTypeDef  		GPIO_InitStruct;
TIM_TimeBaseInitTypeDef		TIMER1_InitStruct;


//	LED_RCC_PERIPH_CLOCK_CMD ( LED_RCC_GPIO_PORT, ENABLE );

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
/*
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_PIN;
	GPIO_Init ( LED_GPIO_PORT, &GPIO_InitStruct );

*/
	TIMER1_InitStruct.TIM_Prescaler=TIMER1_PRESCALER-1;	//предделитель 720-1
	TIMER1_InitStruct.TIM_CounterMode=TIM_CounterMode_Up;	//режим прямого счета
	TIMER1_InitStruct.TIM_Period=TIMER1_RELOAD_PERIOD;		//множитель периодов?
	TIMER1_InitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		//делитель самого таймера(можно не ставить)
	TIMER1_InitStruct.TIM_RepetitionCounter=0x0000;		//счетчик повторов?
	TIM_TimeBaseInit(TIM1,&TIMER1_InitStruct);


	//timer1data=1;
    NVIC_InitTmr1Structure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitTmr1Structure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitTmr1Structure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitTmr1Structure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitTmr1Structure);

	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM1,DISABLE);


	//NVIC_EnableIRQ(TIM1_UP_IRQn);


}
//******************************************************************


void EnableTimer1Interrupt()
{
    //NVIC_InitTmr1Structure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitTmr1Structure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1,ENABLE);

    NVIC_EnableIRQ(TIM1_UP_IRQn);

}

void DisableTimer1Interrupt()
{

    //NVIC_InitTmr1Structure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitTmr1Structure);
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM1,DISABLE);

    NVIC_EnableIRQ(TIM1_UP_IRQn);

}

//********************************************************************
void TIM1_IRQHandler()
{
/*

 //---------------------------------------------------------------------------------------------
uint8_t uidnfcwrdara[] = { '0', '0', '0', '0', '0', '0', '0', '0'};	// массив для записи номера метки nfc
uint8_t u8_cntchar;
uint8_t u8_cntindex;
//--------------------------------------------------------------------------------------------

 */

	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){


		if (GPIO_ReadInputDataBit ( BTN_GPIO_PORT, BTN_PIN ) == Bit_RESET) {//если кнопка нажата

			   if (previousState==1){
				   u8_btnstate=0;
				   cnttimer1=0;

				}
			   else{
				   cnttimer1++;
				   if (cnttimer1==60 && u8_wrnfcstatus==0) {

					   u8_wrnfcstatus=1; b_wrstr1=TRUE;//флаг выдачи первой строки в режиме записи
					   u8_cntchar=0; u8_cntindex=0;
					   b_1secflg=FALSE;
					   cnttimer1sec=0;


				   }
			   }

		}
		else{//если кнопка не нажата

			 if(previousState==0){
				 u8_btnstate=1;
				 cnttimer1=0;
				 cnttimer30sec=0;
				 switch (u8_wrnfcstatus){
				 case 1:
				 case 2:
				 case 3:
				 case 4:
				 case 5:
				 case 6:
				 case 7:
				 case 8:
					    u8_cntchar++;
					    if (u8_cntchar>15) u8_cntchar=0;
					    if (u8_cntchar<10){
					       uidnfcwrdara[u8_cntindex]= 48 + u8_cntchar;
					    }
					    else{
					    	uidnfcwrdara[u8_cntindex]= 55 + u8_cntchar;
					    }
					 break;


				 }

				}

			   else{
				   cnttimer30sec++;
				   if(cnttimer30sec==600){
					   u8_wrnfcstatus=0;
					   cnttimer30sec=0;
				   }


				   cnttimer1sec++;
				   if(cnttimer1sec==10){
					   cnttimer1sec=0;
					   b_1secflg=~b_1secflg;
				   }

				   cnttimer1++;
				   if (cnttimer1==100 ) {
					   switch (u8_wrnfcstatus){
					   case 1:
						   u8_wrnfcstatus=2;
						   u8_cntchar=0; u8_cntindex=1;
						   break;
					   case 2:
						   u8_wrnfcstatus=3;
						   u8_cntchar=0; u8_cntindex=2;
						   break;
					   case 3:
						   u8_wrnfcstatus=4;
						   u8_cntchar=0; u8_cntindex=3;
						   break;
					   case 4:
						   u8_wrnfcstatus=5;
						   u8_cntchar=0; u8_cntindex=4;
						   break;
					   case 5:
						   u8_wrnfcstatus=6;
						   u8_cntchar=0; u8_cntindex=5;
						   break;
					   case 6:
						   u8_wrnfcstatus=7;
						   u8_cntchar=0; u8_cntindex=6;
						   break;
					   case 7:
						   u8_wrnfcstatus=8;
						   u8_cntchar=0; u8_cntindex=7;
						   break;
					   case 8:
					   		u8_wrnfcstatus=9;

					   		break;
					   }

				   }
			   }

		}

		previousState=u8_btnstate;

//		if(u8_btnstate==BTN_PRESSED){
//
//			cnttimer1++;
//		}
//		if(u8_btnstate==BTN_UNPRESSED){
//
//		}

		////////////////////////////////////////////////////////
	//	cnttimer1++;
//
//		if(u8_btnstate==BTN_PRESSED){
//		 if(cnttimer1==10){
//
//				NVIC_InitExtiStruct.NVIC_IRQChannelCmd = ENABLE;
//				NVIC_Init ( &NVIC_InitExtiStruct );
//
//		 }
//		}
//
//		if(u8_btnstate==BTN_UNPRESSED)
//		{
//			if(cnttimer1==timer1data+10){
//
//				NVIC_InitExtiStruct.NVIC_IRQChannelCmd = ENABLE;
//				NVIC_Init ( &NVIC_InitExtiStruct );
//				DisableTimer1Interrupt();
//				previousState=1;
//				timer1data=0;
//			}
//
//		}
//
////	GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 1);
//
//	TIM1->ARR=TIMER1_RELOAD_PERIOD;
//
//	if(cnttimer1==timer1data){
//		cnttimer1=0;
//
//    // Если на выходе был 0
//    if (previousState == 0)
//    {
//        // Выставляем единицу на выходе
//        previousState = 1;
//    //    GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 1);
//        timer1data=3;
//        // Период 50 тиков таймера, то есть 0.5 мс
//        //TIM1->ARR = 50;
//    }
//    else
//    {
//        // Выставляем ноль на выходе
//        previousState = 0;
//  //      GPIO_WriteBit(LED_GPIO_PORT, LED_PIN, 0);
//        timer1data=1;
//        // А период теперь будет 250 тиков – 2.5 мс
//       // TIM1->ARR = 250;
//    }
//
//	}
//



    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

	}
}
//***************************************************


//******************************************************************************
// Инициализация SPI для работы с PN532 NFC *************************************
void PN532_SPIInit(void){
	GPIO_InitTypeDef  		GPIO_InitStruct;
	SPI_InitTypeDef			SPI_PN532_InitStruct;

	SPI_RCC_PORT_CLOCK_CMD ( SPI_RCC_PORT, ENABLE);
	SPI_RCC_PERIPH_CLOCK_CMD ( SPI_RCC_APBPORT, ENABLE );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = SPI_MOSI_PIN | SPI_SCK_PIN;
	GPIO_Init ( SPI_GPIO_PORT, &GPIO_InitStruct );

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = SPI_NSS_PIN;
	GPIO_Init ( SPI_GPIO_PORT, &GPIO_InitStruct );


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_AF_PP;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = SPI_MISO_PIN;
	GPIO_Init ( SPI_GPIO_PORT, &GPIO_InitStruct );



//------------------------------------------------------------------------------
	//SPI_StructInit( &SPI_PN532_InitStruct);

	/* Initialize the SPI_Direction member */
	SPI_PN532_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
    /* initialize the SPI_Mode member */
	SPI_PN532_InitStruct.SPI_Mode = SPI_Mode_Master;
	/* initialize the SPI_DataSize member */
	SPI_PN532_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	/* Initialize the SPI_CPOL member */
	SPI_PN532_InitStruct.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_High;SPI_CPOL_Low;
	/* Initialize the SPI_CPHA member */
	SPI_PN532_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	/* Initialize the SPI_NSS member */
	SPI_PN532_InitStruct.SPI_NSS = SPI_NSS_Hard;//SPI_NSS_Hard;//SPI_NSS_Soft;//SPI_NSS_Hard;
	/* Initialize the SPI_BaudRatePrescaler member */
	SPI_PN532_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//SPI_BaudRatePrescaler_256;//SPI_BaudRatePrescaler_4;
	/* Initialize the SPI_FirstBit member */
	SPI_PN532_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB;//SPI_FirstBit_MSB;//SPI_FirstBit_LSB;
	/* Initialize the SPI_CRCPolynomial member */
	SPI_PN532_InitStruct.SPI_CRCPolynomial = 7;

	SPI_Init (SPI2,  &SPI_PN532_InitStruct);
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE|SPI_I2S_IT_TXE, ENABLE);

	SPI_CalculateCRC ( SPI2, DISABLE);
	GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,1);

	SPI_SSOutputCmd ( SPI2, ENABLE);


/*
	RCC_PLLCmd(DISABLE);
	uint32_t tmpreg = 0;
	tmpreg=RCC->CFGR;
	RCC->CFGR=tmpreg|0x00000500;
	RCC_PLLCmd(ENABLE);
*/

	SPI_Cmd(SPI2, ENABLE);

}
//******************************************************************************
void Send2PC_FirmWareVesion(uint32_t version){

 if (! version) {
	Usart2_SendData(s_pn_nfchip,sizeof(s_pn_nfchip));
  }
 else{
	 pn_chip=version>>24;
	 pn_fw1chip=version>>16;
	 pn_fw2chip=version>>8;
	 pn_suppchip=version&0xFF;

	 s_pn_nchip[0] = 0x30 + (pn_chip >> 4);
	 s_pn_nchip[1] = 0x30 + (pn_chip & 0x0F);

	 s_pn_fw1nchip[0] = 0x30+(pn_fw1chip/100);
	 s_pn_fw1nchip[1] = 0x30+(pn_fw1chip/10%10);
	 s_pn_fw1nchip[2] = 0x30+(pn_fw1chip%10);

	 s_pn_fw2nchip[0] = 0x30+(pn_fw2chip/100);
	 s_pn_fw2nchip[1] = 0x30+(pn_fw2chip/10%10);
	 s_pn_fw2nchip[2] = 0x30+(pn_fw2chip%10);

	 s_pn_suchip[0] = 0x30 + (pn_suppchip >> 4);
	 s_pn_suchip[1] = 0x30 + (pn_suppchip & 0x0F);


	 Usart2_SendData(s_pn_fchip,strlen(s_pn_fchip));
	 Usart2_SendData(s_pn_nchip,sizeof(s_pn_nchip));
	 Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));


	 Usart2_SendData(s_pn_wfchip,strlen(s_pn_wfchip));
	 Usart2_SendData(s_pn_fw1nchip,sizeof(s_pn_fw1nchip));
	 Usart2_SendData(s_pn_point,strlen(s_pn_point));
	 Usart2_SendData(s_pn_fw2nchip,sizeof(s_pn_fw2nchip));
	 Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

	 Usart2_SendData(s_pn_suppchip,sizeof(s_pn_suppchip));
	 Usart2_SendData(s_pn_suchip,sizeof(s_pn_suchip));
	 Usart2_SendData(s_pn_newline,sizeof(s_pn_newline));

 }
}
//*******************************************************
void PN532_FirmWareVersion2String(uint32_t version){

		 pn_chip=version>>24;
		 pn_fw1chip=version>>16;
		 pn_fw2chip=version>>8;
		 pn_suppchip=version&0xFF;

		 s_pn_nchip[0] = 0x30 + (pn_chip >> 4);
		 s_pn_nchip[1] = 0x30 + (pn_chip & 0x0F);

		 s_pn_fw1nchip[0] = 0x30+(pn_fw1chip/100);
		 s_pn_fw1nchip[1] = 0x30+(pn_fw1chip/10%10);
		 s_pn_fw1nchip[2] = 0x30+(pn_fw1chip%10);

		 s_pn_fw2nchip[0] = 0x30+(pn_fw2chip/100);
		 s_pn_fw2nchip[1] = 0x30+(pn_fw2chip/10%10);
		 s_pn_fw2nchip[2] = 0x30+(pn_fw2chip%10);

		 s_pn_suchip[0] = 0x30 + (pn_suppchip >> 4);
		 s_pn_suchip[1] = 0x30 + (pn_suppchip & 0x0F);

}

//*******************************************************




bool fourbytes2strhex(uint8_t *indatarray, uint8_t *outstrhex){

	uint8_t i=0;
	uint8_t b=0;
	uint8_t th=0;
	uint8_t tl=0;

		for ( b=0; b<4; b++){


			th = indatarray[b] >> 4;
			if ((th>=0) && (th <= 9))   { th = th + 0x30; }
			else{
			     if ((th>=0x0A) && (th<=0x0F) ) { th = th + 0x37;}
			}

			tl = indatarray[b]& 0x0F;

			if ((tl>=0) && (tl <= 9) )   { tl = tl + 0x30; }
			else{
				if ((tl>=0x0A) && (tl<=0x0F) ) { tl = tl + 0x37;}

			}

			outstrhex[i] = '0';	i++;
			outstrhex[i] = 'x';	i++;
			outstrhex[i] = th;  i++;
			outstrhex[i] = tl;	i++;
			//outstrhex[i] = ' '; i++;
			if(i<18){ outstrhex[i] = ' '; i++; }


		}
		return 1;
	//}
	//else{
	//	return 0;
	//}
}


//*********************************************************
void PN532_WriteTagtoHex(){

	/*

	 //---------------------------------------------------------------------------------------------
	uint8_t uidnfcwrdara[] = { '0', '0', '0', '0', '0', '0', '0', '0'};	// массив для записи номера метки nfc
	uint8_t u8_cntchar;
	uint8_t u8_cntindex;
	//--------------------------------------------------------------------------------------------

	 */

	uint8_t k=0;
	uint8_t tx=0;
	uint8_t x=0;

	for ( x=0; x<4 ; x++){


				//tx = uidnfcwrdara[k];

				if ((uidnfcwrdara[k]>='0') && (uidnfcwrdara[k] <= '9'))   { tx = uidnfcwrdara[k] - 0x30; }
				else{
				     if ((uidnfcwrdara[k]>='A') && (uidnfcwrdara[k]<='F') ) { tx = uidnfcwrdara[k] - 0x37;}
				}

				outwrdata[x]=tx<<4;
				k++;

				if ((uidnfcwrdara[k]>='0') && (uidnfcwrdara[k] <= '9'))   { tx = uidnfcwrdara[k] - 0x30; }
				else{
				     if ((uidnfcwrdara[k]>='A') && (uidnfcwrdara[k]<='F') ) { tx = uidnfcwrdara[k] - 0x37;}
				}
				outwrdata[x]|=tx;
				k++;

			}


}

//-------------------------------------------------------------------------
bool BytesHex2Str(uint8_t *indatarray, uint8_t size_array, uint8_t *outstrhex){

	uint8_t i=0;
	uint8_t b=0;
	uint8_t th=0;
	uint8_t tl=0;

		for ( b=0; b<size_array; b++){


			th = indatarray[b] >> 4;
			if ((th>=0) && (th <= 9))   { th = th + 0x30; }
			else{
			     if ((th>=0x0A) && (th<=0x0F) ) { th = th + 0x37;}
			}

			tl = indatarray[b]& 0x0F;

			if ((tl>=0) && (tl <= 9) )   { tl = tl + 0x30; }
			else{
				if ((tl>=0x0A) && (tl<=0x0F) ) { tl = tl + 0x37;}

			}

		//	outstrhex[i] = '0';	i++;
		//	outstrhex[i] = 'x';	i++;
			outstrhex[i] = th;  i++;
			outstrhex[i] = tl;	i++;
			//outstrhex[i] = ' '; i++;

			 outstrhex[i] = ' '; i++;




		}
		return 1;
	//}
	//else{
	//	return 0;
	//}
}

//------------------------------------------------------------------------------
void PN532_WriteCARD(){

#define IRQ                     (2)
#define RESET                   (3)     // Not connected by default on the NFC Shield

#define NR_SHORTSECTOR          (32)    // Number of short sectors on Mifare 1K/4K
#define NR_LONGSECTOR           (8)     // Number of long sectors on Mifare 4K
#define NR_BLOCK_OF_SHORTSECTOR (4)     // Number of blocks in a short sector
#define NR_BLOCK_OF_LONGSECTOR  (16)    // Number of blocks in a long sector

// Determine the sector trailer block based on sector number
#define BLOCK_NUMBER_OF_SECTOR_TRAILER(sector) (((sector)<NR_SHORTSECTOR)? \
  ((sector)*NR_BLOCK_OF_SHORTSECTOR + NR_BLOCK_OF_SHORTSECTOR-1):\
  (NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR + NR_BLOCK_OF_LONGSECTOR-1))

// Determine the sector's first block based on the sector number
#define BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sector) (((sector)<NR_SHORTSECTOR)? \
  ((sector)*NR_BLOCK_OF_SHORTSECTOR):\
  (NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR))




	uint8_t success;	// Flag to check if there was an error with the PN532
	uint8_t KEY_DEFAULT_KEYAB[6]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// The default Mifare Classic key

	bool authenticated = FALSE;
	uint8_t blockBuffer[16];                  // Buffer to store block contents
	uint8_t blankAccessBits[3] = { 0xff, 0x07, 0x80 };
	  uint8_t idx = 0;
	  uint8_t numOfSector = 16;                 // Assume Mifare Classic 1K for now (16 4-block sectors)


static char s_pc_PN532_err1[]="Authentication failed for sector";
static char s_pc_PN532_err2[]="Unable to write to sector";
static char s_pc_PN532_err3[]="Unable to write trailer block of sector";

	  for (idx = 0; idx < numOfSector; idx++)
	      {
	        // Step 1: Authenticate the current sector using key B 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
	        success = PN532_mifareclassic_AuthenticateBlock (uid, uidLength, BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), 1, (uint8_t *)KEY_DEFAULT_KEYAB);
	        if (!success)
	        {

	    		Usart2_SendData(s_pc_PN532_err1,strlen(s_pc_PN532_err1));
	    		Usart2_SendData(s_newline,strlen(s_newline));
	    //      Serial.print("Authentication failed for sector "); Serial.println(numOfSector);
	          return;
	        }

	        // Step 2: Write to the other blocks
	        if (idx == 16)
	        {
	          memset(blockBuffer, 0, sizeof(blockBuffer));
	          if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer)))
	          {

		    		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
		    		Usart2_SendData(s_newline,strlen(s_newline));
	     //       Serial.print("Unable to write to sector "); Serial.println(numOfSector);
	            return;
	          }
	        }
	        if ((idx == 0) || (idx == 16))
	        {
	          memset(blockBuffer, 0, sizeof(blockBuffer));
	          if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer)))
	          {

		    		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
		    		Usart2_SendData(s_newline,strlen(s_newline));
	     //       Serial.print("Unable to write to sector "); Serial.println(numOfSector);
	            return;
	          }
	        }
	        else
	        {
	          memset(blockBuffer, 0, sizeof(blockBuffer));
	          if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer)))
	          {

		    		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
		    		Usart2_SendData(s_newline,strlen(s_newline));
	     //       Serial.print("Unable to write to sector "); Serial.println(numOfSector);
	            return;
	          }
	          if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer)))
	          {
		    		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
		    		Usart2_SendData(s_newline,strlen(s_newline));
	     //       Serial.print("Unable to write to sector "); Serial.println(numOfSector);
	            return;
	          }
	        }
	        memset(blockBuffer, 0, sizeof(blockBuffer));
	        if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 1, blockBuffer)))
	        {
	    		Usart2_SendData(s_pc_PN532_err2,strlen(s_pc_PN532_err2));
	    		Usart2_SendData(s_newline,strlen(s_newline));
	   //       Serial.print("Unable to write to sector "); Serial.println(numOfSector);
	          return;
	        }

	        // Step 3: Reset both keys to 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
	        memcpy(blockBuffer, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
	        memcpy(blockBuffer + 6, blankAccessBits, sizeof(blankAccessBits));
	        blockBuffer[9] = 0x69;
	        memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));

	        // Step 4: Write the trailer block
	        if (!(PN532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)), blockBuffer)))
	        {

	    		Usart2_SendData(s_pc_PN532_err3,strlen(s_pc_PN532_err3));
	    		Usart2_SendData(s_newline,strlen(s_newline));
	  //        Serial.print("Unable to write trailer block of sector "); Serial.println(numOfSector);
	          return;
	        }
	      }


	//success=PN532_mifareclassic_AuthenticateBlock(uid, uidLen, 0, 0, keya);
	//success = PN532_mifareclassic_AuthenticateBlock (uid, uidLength, 4, 0, keya);

}

uint8_t PN532_Write_Data(uint8_t *uid_card, uint8_t uid_card_len, uint8_t blockNumber, uint8_t *data){
//	uint8_t success=0;	// Flag to check if there was an error with the PN532
	uint8_t KEY_DEFAULT_KEYAB[6]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// The default Mifare Classic key

	uint8_t res_excep;

	bool authenticated = FALSE;
	uint8_t blockBuffer[16];                  // Buffer to store block contents
	uint8_t blankAccessBits[3] = { 0xff, 0x07, 0x80 };
	//uint8_t idx = 0;
	uint8_t numOfSector = 16;                 // Assume Mifare Classic 1K for now (16 4-block sectors)
    uint8_t i=0;

    uint8_t RdWrDataBlock[16];


	        if (!(PN532_mifareclassic_AuthenticateBlock (uid_card, uid_card_len, 0, 1, (uint8_t *)KEY_DEFAULT_KEYAB)))
	        {
	          return res_excep=1;	//выход из-за ошибки аутентификации блока
	        }

	        res_excep=PN532_mifareclassic_ReadDataBlock(1, RdWrDataBlock);
	        if (!(PN532_mifareclassic_ReadDataBlock(1, RdWrDataBlock)))
	        {
	          return res_excep=2;	//выход из-за ошибки чтения блока
	        }
	        else{

	    		  for(i=0;i<4;i++){
	    			  RdWrDataBlock[i]=data[i];
	    		}



	    		  if(!(PN532_mifareclassic_WriteDataBlock(1,RdWrDataBlock))){
	    			  return res_excep=3;	//выход из-за ошибки запииси блока
	    		  }

	        // Step 3: Reset both keys to 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
	        memcpy(blockBuffer, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
	        memcpy(blockBuffer + 6, blankAccessBits, sizeof(blankAccessBits));
	        blockBuffer[9] = 0x69;
	        memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));

	         //Step 4: Write the trailer block
	        if (!(PN532_mifareclassic_WriteDataBlock(3, blockBuffer)))
	        {
	          return res_excep=4;	//выход из-за ошибки записи ключа блока
	        }else return res_excep=0;

	        }

}

