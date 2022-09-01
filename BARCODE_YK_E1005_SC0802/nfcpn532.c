/*
 * nfcpn532.c
 *
 *  Created on: 14 апр. 2022 г.
 *      Author: User
 */
#include "nfcpn532.h"



void PN532_Delay(uint32_t cntdelay){
	while(cntdelay--){};
}
//******************************************************************************
bool PN532_WakeUp(void){

/*
    uint8_t packet[21];
    uint8_t *p = packet;
*/
	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,0);
	 Delay(10);
	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,1);

/*
    for(int y=0;y<sizeof(packet);y++){
    	packet[y]=0x55;
    }
    packet[0]=0x01;

    ///--------------------------------------------------------------------------
    		for (size_t i = 0; i < sizeof(packet); i++) {

    				while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
    				USART_SendData(USART2_NUM,packet[i]);

    	     }

    		while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
    		USART_SendData(USART2_NUM,0x0D);
    		while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
    ///-----------------------------------------------------------------------------


	 PN532_write( packet,21);
*/
	  return TRUE;

}
/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation byte of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532_setPassiveActivationRetries(uint8_t maxRetries) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;

  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 5,1000))
  { return 0x0;} // no ACK

  return 1;
}


/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field and reads its ID.

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532_readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid,
                                         uint8_t *uidLength, uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1; // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;

  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 3, timeout)) {
    return 0x0; // no cards read
  }
  //Нужна задержка после выполнения подтверждения пакета перед чтением данных!!!!!
 //Задержка внутри PN532_sendCommandCheckAck(pn532_packetbuffer, 3, timeout)
 //Для чтения ID карты задежка не менее 10 мс обязательна!
     Delay(20);
/*
  // wait for a card to enter the field (only possible with I2C)
  if (!waitready(timeout)) {
    return 0x0;   }
*/
  return PN532_readDetectedPassiveTargetID(uid, uidLength);
}

/**************************************************************************/
/**************************************************************************/
/*!
    Reads the ID of the passive target the reader has deteceted.

    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532_readDetectedPassiveTargetID(uint8_t *uid, uint8_t *uidLength) {
  // read data packet
  PN532_readdata(pn532_readbuffer, 19, 0x00);

 //Отладка ----------------------------------------------------------------------
  #ifdef DEBUG_PN532
  //Вывод на компьютер прочитанных байтов


  for (uint8_t i = 0; i < 19; i++) {
	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
	    }

#endif
 //------------------------------------------------------------------------------
  // check some basic stuff

  /* ISO14443A card response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */
/*
#ifdef MIFAREDEBUG
  PN532DEBUGPRINT.print(F("Found "));
  PN532DEBUGPRINT.print(pn532_packetbuffer[7], DEC);
  PN532DEBUGPRINT.println(F(" tags"));
#endif
*/
  if (pn532_readbuffer[7] != 1)
    return FALSE;

  uint16_t sens_res = pn532_readbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_readbuffer[10];
 /*
#ifdef MIFAREDEBUG
  PN532DEBUGPRINT.print(F("ATQA: 0x"));
  PN532DEBUGPRINT.println(sens_res, HEX);
  PN532DEBUGPRINT.print(F("SAK: 0x"));
  PN532DEBUGPRINT.println(pn532_packetbuffer[11], HEX);
#endif
*/
  /* Card appears to be Mifare Classic */
  *uidLength = pn532_readbuffer[12];
/*
#ifdef MIFAREDEBUG
  PN532DEBUGPRINT.print(F("UID:"));
#endif
*/
  for (uint8_t i = 0; i < pn532_readbuffer[12]; i++) {
    uid[i] = pn532_readbuffer[13 + i];
/*
#ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print(uid[i], HEX);
#endif
*/
  }
/*
#ifdef MIFAREDEBUG
  PN532DEBUGPRINT.println();
#endif
*/
  return TRUE;
}

//******************************************************************************
bool PN532_SAMConfig(void){

/*
        /// <summary>
        /// Set SAM configuration
        /// </summary>
        /// <param name="mode">Mode to use SAM</param>
        /// <param name="isIRQ">Use IRQ pin</param>
        /// <returns>Result from SAM configuration</returns>
        public byte[] SAMConfiguration(SamMode mode, bool isIRQ)
        {
            byte irq = isIRQ ? (byte)0x01 : (byte)0x00;
            byte[] cmd = { PN532_SAM_CONFIGURATION, (byte)mode, 0x00, irq };

            return this.ExecuteCmd(cmd);
        }
 *
 */
	 pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
	 pn532_packetbuffer[1] = 0x01;//0x01;
	 pn532_packetbuffer[2] = 0x14;
	 pn532_packetbuffer[3] = 0x00;//не использоать прерывания

	  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 4,500)) {
	    return 0;
	  }

//Нужна задержка после выполнения подтверждения пакета перед чтением данных!!!!!
//Задержка внутри 	PN532_sendCommandCheckAck
//     PN532_Delay(100);
// read data packet
	  PN532_readdata(pn532_readbuffer, 9, 0xFF);
/*
	    for (uint8_t i = 0; i < 8; i++) {
	  	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	  	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
	  	    }
*/
	    //int offset = 6;
	    //pn532_packetbuffer[offset] == 0x15

return (pn532_readbuffer[6] == 0x15);
}


/******************************************************************************/
 uint32_t PN532_getGeneralStatus(void) {
 uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETGENERALSTATUS;

  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 1,500)) {
    return 0;
  }

//Нужна задержка после выполнения подтверждения готовности!!!!!
//Задержка внутри 	PN532_sendCommandCheckAck
//     PN532_Delay(100);

   PN532_readdata(pn532_readbuffer, 13, 0xFF);
/*
   for (uint8_t i = 0; i < 12; i++) {
	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
	    }
*/
 /*
   //сравнение полученных первых шести байт с массивом правильных данных
   //если сдержимое не одинаковое, от выход с ошибкой;
	if (0 != memcmp((char *)pn532_readbuffer, (char *)pn532response_firmwarevers, 6))
	   { return 0; }

  int offset = 7;

  response = pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];
*/

  return response;
}

/******************************************************************************/
/******************************************************************************/
 bool PN532_diagnoseCommunicationLineTest(void){
	 const uint8_t checkdata [4]={'T','e','s','t'};

	 const uint8_t fdata [2]={0xD5,0x01};
	 uint8_t readdata [4]={' ',' ',' ',' '};
	 uint8_t index;

 	 pn532_packetbuffer[0] = PN532_COMMAND_DIAGNOSE;//команда
 	 pn532_packetbuffer[1] = 0x00;//номер теста
 	 pn532_packetbuffer[2] = 0x04;//колличество байт
 	 pn532_packetbuffer[3] = 'T';//байты....
 	 pn532_packetbuffer[4] = 'e';
 	 pn532_packetbuffer[5] = 's';
 	 pn532_packetbuffer[6] = 't';

 	  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 7,500)) {
 	    return 0;
 	  }

 //Нужна задержка после выполнения подтверждения пакета перед чтением данных!!!!!
 //Задержка внутри 	PN532_sendCommandCheckAck
 //     PN532_Delay(100);
 // read data packet
 	  PN532_readdata(pn532_readbuffer, 16, 0xFF);

 	  for(index=0;index<16;index++){
 		  if (pn532_readbuffer[index]==fdata[0] && pn532_readbuffer[index+1]==fdata[1]){

 			 for (uint8_t i = 0; i < 4; i++){
 				readdata[i]=pn532_readbuffer[index+4+i];
 			 }

   			  if(0 != memcmp(checkdata,readdata,sizeof(readdata))){
 				  return FALSE;
 			  }
 			  else
 			  {return TRUE;}
 			return FALSE;
 		  }


 	  }
 }
 /******************************************************************************/

 /******************************************************************************/
/*!
    Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/******************************************************************************/
 uint32_t PN532_getFirmwareVersion(void) {
 uint32_t response;
 const uint8_t pn532response_firmwarevers[] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5};

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 1,500)) {
    return 0;
  }

//Нужна задержка после выполнения подтверждения готовности!!!!!
//Задержка внутри 	PN532_sendCommandCheckAck
//     PN532_Delay(100);

   PN532_readdata(pn532_readbuffer, 13, 0xFF);
/*
   for (uint8_t i = 0; i < 12; i++) {
	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
	    }
*/
   //сравнение полученных первых шести байт с массивом правильных данных
   //если сдержимое не одинаковое, от выход с ошибкой;
	if (0 != memcmp((char *)pn532_readbuffer, (char *)pn532response_firmwarevers, 6))
	   { return 0; }

  int offset = 7;

  response = pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];
  response <<= 8;
  response |= pn532_readbuffer[offset++];

  return response;
}

/**************************************************************************/
 /**************************************************************************/
 /*!
     @brief  Sends a command and waits a specified period for the ACK

     @param  cmd       Pointer to the command buffer
     @param  cmdlen    The size of the command in bytes
     @param  timeout   timeout before giving up

     @returns  1 if everything is OK, 0 if timeout occured before an
               ACK was recieved
 */
 /**************************************************************************/

 bool PN532_sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {

	 PN532_writecommand(cmd, cmdlen);
/*
//Отладка ----------------------------------------------------------------------
#ifdef DEBUG_PN532

	 for (uint8_t r=0; r<sizeof(replybuff);r++) {
	     replybuff[r]=0; }
     rl=0;

#endif
//------------------------------------------------------------------------------
*/
//Ожидание и чтение байта готовности (0x01)
   if (!PN532_waitready(timeout)) {
	  return FALSE; }

//Нужна задержка после выполнения подтверждения готовности!!!!!
   PN532_Delay(100);

   //Delay(1);
/*
//Отладка ----------------------------------------------------------------------
  #ifdef DEBUG_PN532
//Вывод на компьютер байтов принятых на команду чтение готовности
   for(uint8_t r=0; r<sizeof(replybuff);r++){

   	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
   	if(r==sizeof(replybuff)-1){replybuff[r]=0xAA;}
   	USART_SendData(USART2_NUM, replybuff[r]);

  	}
  #endif
//------------------------------------------------------------------------------
*/
// чтение подтверждения
  if (!PN532_readack()) {  return FALSE;  }

   // For SPI only wait for the chip to be ready again.
   // This is unnecessary with I2C.
  PN532_Delay(100);

//Delay(2);
 //  if (!PN532_waitready(timeout)) {  return FALSE;}
//Delay(1);

   return TRUE; // ack'd command

 }

 /*****************************************************************************/
 /**************************************************************************/
 /*!
     @brief  Writes a command to the PN532, automatically inserting the
             preamble and required frame details (checksum, len, etc.)

     @param  cmd       Pointer to the command buffer
     @param  cmdlen    Command length in bytes
 */
 /**************************************************************************/
 void PN532_writecommand(uint8_t *cmd, uint8_t cmdlen) {

     uint8_t checksum;
     uint8_t packet[8 + cmdlen];
     uint8_t *p = packet;
     cmdlen++;

     p[0] = PN532_SPI_DATAWRITE;
     p++;

     p[0] = PN532_PREAMBLE;
     p++;
     p[0] = PN532_STARTCODE1;
     p++;
     p[0] = PN532_STARTCODE2;
     p++;
     checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;

     p[0] = cmdlen;
     p++;
     p[0] = ~cmdlen + 1;
     p++;

     p[0] = PN532_HOSTTOPN532;
     p++;
     checksum += PN532_HOSTTOPN532;

     for (uint8_t i = 0; i < cmdlen - 1; i++) {
       p[0] = cmd[i];
       p++;
       checksum += cmd[i];
     }

     p[0] = ~checksum;
     p++;
     p[0] = PN532_POSTAMBLE;
     p++;
/*
    if(test==3){
    	packet[0]=0x00;packet[1]=0x00;packet[2]=0xFF;packet[3]=0x04;packet[4]=0xFC;
    	packet[5]=0xD4;packet[6]=0x4A;packet[7]=0x02;packet[8]=0x00;packet[9]=0xE0;packet[10]=0x00;
    	cmdlen=3;

    // 00 00 ff 04 fc d4 4a 02 00 e0 00
    }
*/
//Отладка ----------------------------------------------------------------------
#ifdef DEBUG_PN532
//Вывод на компьютер байтов собранных для отправления

		for (size_t i = 0; i < 8 + cmdlen; i++) {

				while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
				USART_SendData(USART2_NUM,packet[i]);

	     }

//		while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
//		USART_SendData(USART2_NUM,0x0D);
//		while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
#endif
//-----------------------------------------------------------------------------

     PN532_write( packet, 8 + cmdlen);

 }
/******************************************************************************/
 void PN532_write( uint8_t *buffer, size_t len) {
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET) {};  // ждём пока данные уйдут

	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,0);

	  //PN532_Delay(100);
	  Delay(2);
	for (size_t i = 0; i < len; i++) {

		SPI_I2S_SendData(SPI2,buffer[i]);

	    PN532_Delay(400);

	    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET){};  // ждём пока данные уйдут

     }

//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

	GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,1);


 }
 /*****************************************************************************/
 /*****************************************************************************/
 /*!
     @brief  Reads len_rd_buff bytes of data from the PN532 via SPI or I2C.

     @param  rd_buff      Pointer to the buffer where data will be written
     @param  len_rd_buff         Number of bytes to be read
 */
 /**************************************************************************/
 void PN532_readdata(uint8_t *rd_buff, uint8_t len_rd_buff,uint16_t sendvalue) {
      //uint8_t cmd = PN532_SPI_DATAREAD;
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,0);

    // PN532_Delay(100);
     Delay(2);
	   while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут
	   SPI_I2S_SendData(SPI2, PN532_SPI_DATAREAD);
	   while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут
	   while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появятся
	   uint8_t rdrd =SPI_I2S_ReceiveData(SPI2);

	   // чтение
	     for (uint8_t i = 0; i < len_rd_buff; i++) {
	         PN532_Delay(400);

	    	 SPI_I2S_SendData(SPI2, sendvalue);

	         PN532_Delay(400);

			 while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут

	    	 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появтся

	    	 rd_buff[i] =SPI_I2S_ReceiveData(SPI2);
	     }


//	 	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу


	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN, 1);


//Отладка ----------------------------------------------------------------------
#ifdef DEBUG_PN532
//Вывод на компьютер прочитанных байтов

     for (uint8_t i = 0; i < len_rd_buff; i++) {
	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	    	USART_SendData(USART2_NUM,rd_buff[i]);
	    }
#endif
//------------------------------------------------------------------------------

}

 /*****************************************************************************/
 /**************************************************************************/
 /*!
     @brief  Tries to read the SPI or I2C ACK signal
 */
 /**************************************************************************/
 bool PN532_readack() {

	 const uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
	 uint8_t ackbuff[6];
	 //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

//NSS устанавливаем в 0
	 GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,0);

	 //PN532_Delay(100);
	 Delay(2);
//Перед началом передачи проверяем свободен ли передающий буфер для записи новых данных
//Пока флаг SPI_I2S_FLAG_TXE ==0 (не установлен) передача новых данных невозможна
	  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут

	  SPI_I2S_SendData(SPI2, PN532_SPI_DATAREAD);
      while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут

//Читаем данные, чтобы очистить приемный буфер иначе там данные от байта готовности!!????(непонятно!)
	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появятся
	   uint8_t rdrd =SPI_I2S_ReceiveData(SPI2);

//Читаем 6 байт пакета подтверждения
	  for (uint8_t i = 0; i < 6; i++) {

		  PN532_Delay(100);

	      while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут
	      SPI_I2S_SendData(SPI2, 0x00);

	      PN532_Delay(100);

	      while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут
    	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появятся
	      ackbuff[i] =SPI_I2S_ReceiveData(SPI2);

	     }

//		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

	    GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,1);



//Отладка ----------------------------------------------------------------------
 #ifdef DEBUG_PN532
//Вывод на компьютер байтов принятых на комаду подтверждение (ack)
//для отладки выводим буфер подтверждения

	    for (uint8_t i = 0; i < 6; i++) {
	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	    	USART_SendData(USART2_NUM,ackbuff[i]);

	    }
#endif
//------------------------------------------------------------------------------

//сравнение полученных  шести байт с массивом правильных данных
//если сдержимое не одинаковое, от выход с ошибкой;
//проверка подтверждения (ack)
   return (0 == memcmp((char *)ackbuff, (char *)pn532ack, sizeof(pn532ack)));

}

 /**************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Return true if the PN532 is ready with a response.
 */
 /**************************************************************************/
 bool PN532_isready() {

     uint8_t reply;
// 	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

//Перед началом передачи проверяем свободен ли передающий буфер для записи новых данных
//Пока флаг SPI_I2S_FLAG_TXE ==0 (не установлен) передача новых данных невозможна
      while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET) {};  // ждём пока данные уйдут

//Перед началом передачи проверяем пустой ли принимающий буфер
//Читаем данные, чтобы очистить принимающий регистр (не понятно зачем)
	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появятся
	  //uint8_t rdrd =SPI_I2S_ReceiveData(SPI2);
	  replybuff[rl]=SPI_I2S_ReceiveData(SPI2);
	  rl++;
//NSS устанавливаем в 0
	  GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,0);

	    //PN532_Delay(100);
	    Delay(2);
	  SPI_I2S_SendData(SPI2, PN532_SPI_STATREAD);//посылка команды чтения байта состояния

//Перед началом передачи проверяем свободен ли передающий буфер для записи новых данных
//Пока флаг SPI_I2S_FLAG_TXE ==0 (не установлен) передача новых данных невозможна
	  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут

//Перед началом передачи проверяем пустой ли принимающий буфер
//Читаем данные, чтобы очистить примнающий регистр (не понятно зачем)
//Получается, что нужно прочитать 2 раза приемный буфер, что бы потом получить корретные данные
//при посылке байта и чтении данных после его передачи (странно!!! установленно опытным путем)
	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появятся
	  //uint8_t rdrd =SPI_I2S_ReceiveData(SPI2);
	  replybuff[rl]=SPI_I2S_ReceiveData(SPI2);
	  rl++;

	     PN532_Delay(400);

      while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET) {};  // ждём пока данные уйдут

	  SPI_I2S_SendData(SPI2, 0XFF);//посылка "пустышки", чтобы забрать байт состояния

	     PN532_Delay(400);

   	  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET) {};  // ждём пока данные уйдут

	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {};  // ждём пока данные появятся

	  reply = SPI_I2S_ReceiveData(SPI2);

//		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);  // ждём пока SPI не закончит работу

//NSS устанавливаем в 1
	  GPIO_WriteBit(SPI_GPIO_PORT, SPI_NSS_PIN,1);

	   replybuff[rl]=reply;
       rl++;

       //USART_SendData(USART2_NUM,reply);

// Проверка ответа с байтом готовности (0x01);
     return reply == PN532_SPI_READY;


 }

 /**************************************************************************/
 /*!
     @brief  Waits until the PN532 is ready.

     @param  timeout   Timeout before giving up
 */
 /**************************************************************************/
 bool PN532_waitready(uint16_t timeout) {
   uint16_t timer = 0;

   Delay(5);
   timer += 5;
   //USART_SendData(USART2_NUM,0xFF);

   while (!PN532_isready()) {
     if (timeout != 0) {
       timer += 5;
       if (timer > timeout) {
         return FALSE;
       }
     }
     Delay(5);
   }
   return TRUE;
 }

 /**************************************************************************/

 /**************************************************************************/
 /*!
     @brief  set data in PN532 in the emulation mode
     @param  cmd    = data
     @param  cmdlen = data length
 */
 /**************************************************************************/
 uint8_t PN532_setDataTarget(uint8_t *cmd, uint8_t cmdlen) {
   uint8_t length;
   // cmd1[0] = 0x8E; Must!

   if (!PN532_sendCommandCheckAck(cmd, cmdlen,500))
     return FALSE;

   //PN532_readdata(uint8_t *rd_buff, uint8_t len_rd_buff,uint16_t sendvalue);

   PN532_readdata(pn532_readbuffer,8,0xFF);
   length = pn532_readbuffer[3] - 3;
   for (int i = 0; i < length; ++i) {
        cmd[i] = pn532_readbuffer[8 + i];
      }
      // cmdl = 0
      cmdlen = length;
 /*
   // read data packet
   readdata(pn532_packetbuffer, 8);
   length = pn532_packetbuffer[3] - 3;
   for (int i = 0; i < length; ++i) {
     cmd[i] = pn532_packetbuffer[8 + i];
   }
   // cmdl = 0
   cmdlen = length;
*/
   int offset = 6;
   return (pn532_readbuffer[offset] == 0x15);
 }

 /**************************************************************************/

 /**************************************************************************/
 /*!
     @brief  retrieve response from the emulation mode
     @param  cmd    = data
     @param  cmdlen = data length
 */
 /**************************************************************************/
 uint8_t PN532_getDataTarget(uint8_t *cmd, uint8_t *cmdlen) {
   uint8_t length;
   pn532_packetbuffer[0] = PN532_COMMAND_TGGETDATA;
   if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 1,500)) {
     //PN532DEBUGPRINT.println(F("Error en ack"));
     return FALSE;
   }

   // read data packet
   PN532_readdata(pn532_readbuffer, 64,0xFF);
   length = pn532_readbuffer[3] - 3;

   // if (length > *responseLength) {// Bug, should avoid it in the reading
   // target data
   //  length = *responseLength; // silent truncation...
   //}

   for (int i = 0; i < length; ++i) {
     cmd[i] = pn532_readbuffer[8 + i];
   }
   *cmdlen = length;
   return TRUE;
 }

 /**************************************************************************/
 /**************************************************************************/
 /*!
     @brief  set the PN532 as iso14443a Target behaving as a SmartCard
     @param  None
     #author Salvador Mendoza(salmg.net) new functions:
     -AsTarget
     -getDataTarget
     -setDataTarget
 */
 /**************************************************************************/
 uint8_t PN532_AsTarget() {
   pn532_packetbuffer[0] = 0x8C;
   uint8_t target[] = {
       0x8C,             // INIT AS TARGET
       0x00,             // MODE -> BITFIELD
       0x08, 0x00,       // SENS_RES - MIFARE PARAMS
       0xdc, 0x44, 0x20, // NFCID1T
       0x60,             // SEL_RES
       0x01, 0xfe, // NFCID2T MUST START WITH 01fe - FELICA PARAMS - POL_RES
       0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xc0,
       0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, // PAD
       0xff, 0xff,                               // SYSTEM CODE
       0xaa, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44,
       0x33, 0x22, 0x11, 0x01, 0x00, // NFCID3t MAX 47 BYTES ATR_RES
       0x0d, 0x52, 0x46, 0x49, 0x44, 0x49, 0x4f,
       0x74, 0x20, 0x50, 0x4e, 0x35, 0x33, 0x32 // HISTORICAL BYTES
   };
   if (!PN532_sendCommandCheckAck(target, sizeof(target),100))
     return FALSE;

   // read data packet
   PN532_readdata(pn532_packetbuffer, 8,0xFF);

   int offset = 6;
   return (pn532_packetbuffer[offset] == 0x15);
 }
 /**************************************************************************/

 /**************************************************************************/
 /*!
     Tries to authenticate a block of memory on a MIFARE card using the
     INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
     for more information on sending MIFARE and other commands.
     @param  uid           Pointer to a byte array containing the card UID
     @param  uidLen        The length (in bytes) of the card's UID (Should
                           be 4 for MIFARE Classic)
     @param  blockNumber   The block number to authenticate.  (0..63 for
                           1KB cards, and 0..255 for 4KB cards).
     @param  keyNumber     Which key type to use during authentication
                           (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
     @param  keyData       Pointer to a byte array containing the 6 byte
                           key value
     @returns 1 if everything executed properly, 0 for an error
 */
 /**************************************************************************/
 uint8_t PN532_mifareclassic_AuthenticateBlock(uint8_t *uid,
                                                         uint8_t uidLen,
                                                         uint32_t blockNumber,
                                                         uint8_t keyNumber,
                                                         uint8_t *keyData) {
   // uint8_t len;
   uint8_t i;

   // Hang on to the key and uid data
   memcpy(_key, keyData, 6);
   memcpy(_uid, uid, uidLen);
   _uidLen = uidLen;
/*
 #ifdef MIFAREDEBUG
   PN532DEBUGPRINT.print(F("Trying to authenticate card "));
   Adafruit_PN532::PrintHex(_uid, _uidLen);
   PN532DEBUGPRINT.print(F("Using authentication KEY "));
   PN532DEBUGPRINT.print(keyNumber ? 'B' : 'A');
   PN532DEBUGPRINT.print(F(": "));
   Adafruit_PN532::PrintHex(_key, 6);
 #endif
*/
   // Prepare the authentication command //
   pn532_packetbuffer[0] =
       PN532_COMMAND_INDATAEXCHANGE; /* Data Exchange Header */
   pn532_packetbuffer[1] = 1;        /* Max card numbers */
   pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
   //pn532_packetbuffer[2] = 0x61;
   pn532_packetbuffer[3] =
       blockNumber; /* Block Number (1K = 0..63, 4K = 0..255 */
   memcpy(pn532_packetbuffer + 4, _key, 6);
   for (i = 0; i < _uidLen; i++) {
     pn532_packetbuffer[10 + i] = _uid[i]; /* 4 byte card ID */
   }
/*
   for (uint8_t i = 0; i < 14; i++) {
  	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
  	    	USART_SendData(USART2_NUM,pn532_packetbuffer[i]);
  	    }
*/

   if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 10 + _uidLen,100))
     return 0;
   Delay(10);
   // Read the response packet
   PN532_readdata(pn532_readbuffer, 12,0xFF);
/*
   for (uint8_t i = 0; i < 12; i++) {
 	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
 	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
 	    }
*/
   // check if the response is valid and we are authenticated???
   // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
   // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00
   // is not good
   if (pn532_readbuffer[7] != 0x00) {
 #ifdef PN532DEBUG
	   for (uint8_t i = 0; i < 12; i++) {
	 	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
	 	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
	 	    }
 #endif
     return 0;
   }

   return 1;
 }

 /**************************************************************************/
 /*!
     Tries to read an entire 16-byte data block at the specified block
     address.
     @param  blockNumber   The block number to authenticate.  (0..63 for
                           1KB cards, and 0..255 for 4KB cards).
     @param  data          Pointer to the byte array that will hold the
                           retrieved data (if any)
     @returns 1 if everything executed properly, 0 for an error
 */
 /**************************************************************************/
 uint8_t PN532_mifareclassic_ReadDataBlock(uint8_t blockNumber,
                                                     uint8_t *data) {
 #ifdef MIFAREDEBUG
 //  PN532DEBUGPRINT.print(F("Trying to read 16 bytes from block "));
   //PN532DEBUGPRINT.println(blockNumber);
 #endif

   /* Prepare the command */
   pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
   pn532_packetbuffer[1] = 1;               /* Card number */
   pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Read command = 0x30 */
   pn532_packetbuffer[3] = blockNumber; /* Block Number (0..63 for 1K, 0..255 for 4K) */

   /* Send the command */
   if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 4,100)) {
 #ifdef MIFAREDEBUG
   //  PN532DEBUGPRINT.println(F("Failed to receive ACK for read command"));
 #endif
     return 0;
   }
Delay(10);
   /* Read the response packet */
   PN532_readdata(pn532_readbuffer, 26,0xFF);

   /* If byte 8 isn't 0x00 we probably have an error */
   if (pn532_readbuffer[7] != 0x00) {
 #ifdef MIFAREDEBUG
  //   PN532DEBUGPRINT.println(F("Unexpected response"));
    // Adafruit_PN532::PrintHexChar(pn532_packetbuffer, 26);
 #endif
     return 0;
   }

   /* Copy the 16 data bytes to the output buffer        */
   /* Block content starts at byte 9 of a valid response */
   memcpy(data, pn532_readbuffer + 8, 16);

 /* Display data for debug if requested */
 #ifdef MIFAREDEBUG
  // PN532DEBUGPRINT.print(F("Block "));
  // PN532DEBUGPRINT.println(blockNumber);
  // Adafruit_PN532::PrintHexChar(data, 16);
 #endif

   return 1;
 }

 /**************************************************************************/
 /*!
     Tries to write an entire 16-byte data block at the specified block
     address.
     @param  blockNumber   The block number to authenticate.  (0..63 for
                           1KB cards, and 0..255 for 4KB cards).
     @param  data          The byte array that contains the data to write.
     @returns 1 if everything executed properly, 0 for an error
 */
 /**************************************************************************/
 uint8_t PN532_mifareclassic_WriteDataBlock(uint8_t blockNumber,
                                                      uint8_t *data) {
 #ifdef MIFAREDEBUG
  // PN532DEBUGPRINT.print(F("Trying to write 16 bytes to block "));
  // PN532DEBUGPRINT.println(blockNumber);
 #endif

   /* Prepare the first command */
   pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
   pn532_packetbuffer[1] = 1;                /* Card number */
   pn532_packetbuffer[2] = MIFARE_CMD_WRITE; /* Mifare Write command = 0xA0 */
   pn532_packetbuffer[3] =
       blockNumber; /* Block Number (0..63 for 1K, 0..255 for 4K) */
   memcpy(pn532_packetbuffer + 4, data, 16); /* Data Payload */

   /* Send the command */
   if (!PN532_sendCommandCheckAck(pn532_packetbuffer, 20,100)) {
 #ifdef MIFAREDEBUG
 //    PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
 #endif
     return 0;
   }
   Delay(10);
   Delay(10);
   /* Read the response packet */
   PN532_readdata(pn532_readbuffer, 26,0xFF);
/*
   for (uint8_t i = 0; i < 26; i++) {
 	    	while(USART_GetFlagStatus(USART2_NUM, USART_FLAG_TXE)==RESET){};
 	    	USART_SendData(USART2_NUM,pn532_readbuffer[i]);
 	    }
*/
   return 1;
 }

 /**************************************************************************/
 uint8_t PN532_Read_Data(uint8_t *uid_card, uint8_t uid_card_len, uint8_t blockNumber, uint8_t *data)
 {
 	uint8_t success=0;	// Flag to check if there was an error with the PN532
 	uint8_t KEY_DEFAULT_KEYAB[6]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// The default Mifare Classic key

	uint8_t res_excep;

 	bool authenticated = FALSE;
 	uint8_t blockBuffer[16];                  // Buffer to store block contents
 	uint8_t blankAccessBits[3] = { 0xff, 0x07, 0x80 };


 static char s_pc_PN532_err1[]="Authentication failed for sector";
 static char s_pc_PN532_err4[]="Unable to read data block";


 //success = PN532_mifareclassic_AuthenticateBlock (uid_card, uid_card_len, 0, 1, (uint8_t *)KEY_DEFAULT_KEYAB);
 	        if (!(PN532_mifareclassic_AuthenticateBlock (uid_card, uid_card_len, 0, 1, (uint8_t *)KEY_DEFAULT_KEYAB)))
 	        {
 	          return res_excep=1;
 	        }

 	       if(!(res_excep=PN532_mifareclassic_ReadDataBlock(blockNumber, data)))
 	       {
 	          return res_excep=2;
 	       }else return res_excep=0;
 }

