
#include "pn532.h"
#include "string.h"

extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;

#define PN532_PACKBUFFSIZ 64
uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];
uint8_t arr[PN532_PACKBUFFSIZ];
uint8_t  pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
uint8_t  pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};
uint8_t abtHalt[4] = { 0x50, 0x00, 0x57, 0xCD };
uint8_t abtUnlock1 = 0x40;
uint8_t abtUnlock2 = 0x43;
uint8_t buf[16],nbuf;
 
static uint8_t _inListedTag;  // Tg number of inlisted tag.


#define spi_write( M ) arr[im++] = M


HAL_StatusTypeDef  writecommand(SPI_HandleTypeDef *hspi, uint8_t* cmd, uint8_t cmdlen, uint16_t timeout){

    // SPI command write.
    HAL_StatusTypeDef tmp;
    uint8_t checksum;
    uint8_t  im = 0;

    cmdlen++;

    spi_write(PN532_SPI_DATAWRITE);

    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    spi_write(PN532_PREAMBLE);
    spi_write(PN532_PREAMBLE);
    spi_write(PN532_STARTCODE2);

    spi_write(cmdlen);
    spi_write(~cmdlen + 1);

    spi_write(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

    for (uint8_t i=0; i<cmdlen-1; i++) {
      spi_write(cmd[i]);
      checksum += cmd[i];

    }

    spi_write(~checksum);
    spi_write(PN532_POSTAMBLE);
    
    CS_LOW(hspi);
//    HAL_Delay(1);
    tmp = HAL_SPI_Transmit(hspi, arr, im, timeout);
    CS_HIGH(hspi);
    return tmp;
}


void readdata(SPI_HandleTypeDef *hspi, uint8_t* buff, uint8_t n, uint16_t timeout) {

    // SPI write.
    uint8_t b = PN532_SPI_DATAREAD;
    CS_LOW(hspi);
//    HAL_Delay(1);
    HAL_SPI_Transmit(hspi, &b, 1, timeout);
    HAL_SPI_Receive(hspi, buff, n, timeout);
    CS_HIGH(hspi);
}


uint8_t readack(SPI_HandleTypeDef *hspi) {
  uint8_t ackbuff[6];

  readdata(hspi, ackbuff, 6,1000);

  return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}


uint8_t isready(SPI_HandleTypeDef *hspi) {

    // SPI read status and check if ready.
    uint8_t b = PN532_SPI_STATREAD;
    uint8_t x ;
    CS_LOW(hspi);
//    HAL_Delay(1);
    HAL_SPI_Transmit(hspi, &b, 1, 1000);
    // read byte
    HAL_SPI_Receive(hspi, &x, 1, 1000);
    CS_HIGH(hspi);

    // Check if status is ready.
    return x == PN532_SPI_READY;
}


uint8_t waitready(SPI_HandleTypeDef *hspi, uint16_t timeout) {
  uint16_t timer = 0;
  while(!isready( hspi )) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        return false;
      }
    }
    HAL_Delay(10);
  }
  return true;
}


uint8_t sendCommandCheckAck(SPI_HandleTypeDef *hspi, uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {


  // write the command
  
  if ( 0 != writecommand(hspi, cmd, cmdlen , timeout) ) {
    return 0;                                 // Error
  }
  
    if (!readack( hspi )) {
    return 0;
  }

  return 1; // OK
}


uint8_t readRegisters(SPI_HandleTypeDef *hspi, uint16_t *Adr,uint8_t *rd, uint8_t n){
         
        uint8_t i;
        
	pn532_packetbuffer[0] = PN532_COMMAND_READREGISTER;
        for(i = 0; i < n; i ++)
        {

	  pn532_packetbuffer[i*2 + 1] = *((uint8_t *)Adr + 2*i + 1);
	  pn532_packetbuffer[i*2 + 2] = *((uint8_t *)Adr + 2*i);
        }
	// Send the  command
	if (! sendCommandCheckAck(hspi, pn532_packetbuffer, (2 * n +1), 1000))
	  return 0x0;
        
        if (!waitready(hspi, 1000)) {
          return 0x0;
        }
        
	readdata(hspi, pn532_packetbuffer, (7 + n), 1000);
	int offset =  6;
        for(i = 0; i < n; i ++){
          rd[i] =  pn532_packetbuffer[offset + 1 +i ];
        }
	return  1;
}


uint8_t  writeRegisters(SPI_HandleTypeDef *hspi, uint16_t *ADR,uint8_t *d, uint8_t n){

	uint8_t j = 0;
	// Fill command buffer

	pn532_packetbuffer[j++] = PN532_COMMAND_WRITEREGISTER;
	for( uint8_t i = 0; i < n; i++){
		pn532_packetbuffer[j++] = *((uint8_t *)&(ADR[i]) + 1);
		pn532_packetbuffer[j++] = *((uint8_t *)&ADR[i]);
		pn532_packetbuffer[j++] = d[i];
	}


	// Send the  command
	if (! sendCommandCheckAck(hspi, pn532_packetbuffer, j, 1000))
	  return 0x0;

	readdata(hspi, pn532_packetbuffer, 6, 1000);
	int offset = 6;
	return  (pn532_packetbuffer[offset+1] == (PN532_COMMAND_WRITEREGISTER + 1));

}


uint8_t  writeRegister(SPI_HandleTypeDef *hspi, uint16_t ADR,uint8_t d){

	// Fill command buffer
	pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
	pn532_packetbuffer[1] = *((uint8_t *)&ADR + 1);
	pn532_packetbuffer[2] = *((uint8_t *)&ADR);
	pn532_packetbuffer[3] = d;

	// Send the  command
	if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 4, 1000))
	  return 0x0;

	readdata(hspi, pn532_packetbuffer, 6, 1000);
	int offset = 6;
	return  (pn532_packetbuffer[offset+1] == (PN532_COMMAND_WRITEREGISTER + 1));

}


uint32_t getFirmwareVersion( SPI_HandleTypeDef *hspi, uint16_t timeout) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 1, timeout)) {
    return 0;
  }
  
  if (!waitready(hspi, timeout)) {
     return 0x0;
  }

  // read data packet
  readdata(hspi, pn532_packetbuffer, 12, timeout);

  // check some basic stuff
  if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {

    return 0;
  }

  int offset =  6 ;  
  response = pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];

  return response;
}


uint8_t SAM_VirtualCard(SPI_HandleTypeDef *hspi ) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x02; // VirtualCard mode;
  pn532_packetbuffer[2] = 0xFF; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 4,1000))
    return 0;

  // read data packet
  readdata( hspi, pn532_packetbuffer, 8,1000);

  int offset = 6;
  return  (pn532_packetbuffer[offset] == 0x15);
}


uint8_t SAM_Config(SPI_HandleTypeDef *hspi ) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // VirtualCard mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 4,1000))
    return 0;

  // read data packet
  readdata( hspi, pn532_packetbuffer, 8,1000);

  int offset = 6;
  return  (pn532_packetbuffer[offset] == 0x15);
}


uint8_t  readPassiveTargetID( SPI_HandleTypeDef *hspi, uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;

  if (!sendCommandCheckAck( hspi, pn532_packetbuffer, 3, timeout))
  {
    return 0x0;  // no cards read
  }

  // wait for a card to enter the field (only possible with I2C)


    if (!waitready(hspi, timeout)) {

      return 0x0;
    }


  // read data packet
  readdata( hspi, pn532_packetbuffer, 20, timeout);
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


  if (pn532_packetbuffer[7] != 1)
    return 0;

  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];

  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];

  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++)
  {
    uid[i] = pn532_packetbuffer[13+i];

  }

  return 1;
}


uint8_t		inListPassiveTarget( SPI_HandleTypeDef *hspi ) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;

  if (!sendCommandCheckAck(hspi, pn532_packetbuffer,3,1000)) {
    return false;
  }

  if (!waitready(hspi, 30000)) {
    return false;
  }

  readdata(hspi, pn532_packetbuffer,sizeof(pn532_packetbuffer), 1000);

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      return false;    //"Length check invalid"
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INLISTPASSIVETARGET) {
      if (pn532_packetbuffer[7] != 1) {
        return false;  //"Unhandled number of targets inlisted"
      }

      _inListedTag = pn532_packetbuffer[8];
      return true;
    } else {
      return false;   // "Unexpected response to inlist passive host"
    }
  }
  else {
    return false;  //"Preamble missing"
  }
}


uint8_t		inDataExchange( SPI_HandleTypeDef *hspi, uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength ) {
  if (sendLength > PN532_PACKBUFFSIZ-2) {
    return false;
  }
  uint8_t i;

  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;    //_inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }

  if (!sendCommandCheckAck(hspi, pn532_packetbuffer,sendLength+2,1000)) {
    return false;
  }

  if (!waitready(hspi, 1000)) {
    return false;
  }

  readdata(hspi, pn532_packetbuffer,sizeof(pn532_packetbuffer), 1000);

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      return false; //"Length check invalid"
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INDATAEXCHANGE) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        return false; // "Status code indicates an error"
      }

      length -= 3;

      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }

      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }
      *responseLength = length;

      return true;
    }
    else {
      return false;    // "Don't know how to handle this command: "
    }
  }
  else {
    return false;
  }
  
}


uint8_t		InCommunicateThru( SPI_HandleTypeDef *hspi, uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength ) {
  if (sendLength > PN532_PACKBUFFSIZ-2) {
    return false;
  }
  uint8_t i;

  pn532_packetbuffer[0] = PN532_COMMAND_INCOMMUNICATETHRU; // PN532_COMMAND_INDATAEXCHANGE;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+1] = send[i];
  }

  if (!sendCommandCheckAck(hspi, pn532_packetbuffer,sendLength+1,1000)) {
    return false;
  }

  if (!waitready(hspi, 1000)) {
    return false;
  }

  readdata(hspi, pn532_packetbuffer,sizeof(pn532_packetbuffer), 1000);

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];

    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      return false; //"Length check invalid"
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_COMMAND_INCOMMUNICATETHRU + 1) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        return false; // "Status code indicates an error"
      }

      length -= 3;
      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }else{
      	*responseLength = length;
      }

      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }

      return true;
    }
    else {
      return false;    // "Don't know how to handle this command: "
    }
  }
  else {
    return false;
  }

}


uint8_t mifareclassic_AuthenticateBlock (SPI_HandleTypeDef *hspi, uint8_t * uid, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData)
{

	uint8_t i;

  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, keyData, 6);
  for (i = 0; i < 4; i++)
  {
    pn532_packetbuffer[10+i] = uid[i];                /* 4 byte card ID */
  }

  if (! sendCommandCheckAck(hspi,pn532_packetbuffer, 14,1000))
    return 0;

  if (!waitready(hspi, 1000)) {
    return false;
  }

  // Read the response packet
  readdata(hspi, pn532_packetbuffer, 12, 1000);

  // check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)
  {
    return 0;
  }

  return 1;
}


uint8_t mifareclassic_ReadDataBlock (SPI_HandleTypeDef *hspi, uint8_t blockNumber, uint8_t * data)
{


  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 4, 1000))
  {
    return 0;
  }

  if (!waitready(hspi, 1000)) {
    return false;
  }

  /* Read the response packet */
  readdata(hspi, pn532_packetbuffer, 26, 1000);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    return 0;
  }

  /* Copy the 16 data bytes to the output buffer        */
  /* Block content starts at byte 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  /* Display data for debug if requested */

  return 1;
}


uint8_t mifareclassic_WriteDataBlock (SPI_HandleTypeDef *hspi, uint8_t blockNumber, uint8_t * data)
{

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, 20, 1000))
  {

    return 0;
  }
  HAL_Delay(10);
  if (!waitready(hspi, 1000)) {
    return false;
  }

  /* Read the response packet */
  readdata(hspi, pn532_packetbuffer, 26, 1000);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    return 0;
  }

  return 1;
}


uint8_t 	calcCRC(SPI_HandleTypeDef *hspi, uint8_t *buf, uint8_t n, uint16_t *crc){
  
  pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
  pn532_packetbuffer[1] = 0x63;
  pn532_packetbuffer[2] = 0x31;
  pn532_packetbuffer[3] = 0x03;
  uint8_t j = 4;
  for(int i = 0; i < n; i++){
    pn532_packetbuffer[j++] = 0x63;
    pn532_packetbuffer[j++] = 0x39;
    pn532_packetbuffer[j++] = buf[i];
  }
  
   if (! sendCommandCheckAck(hspi, pn532_packetbuffer, (3 * n + 4), 1000))
    return 0x0;
        
  if (!waitready(hspi, 1000)) {
    return 0x0;
  }
  
  readdata(hspi, pn532_packetbuffer, 7, 1000);
  int offset = 6;
  if(pn532_packetbuffer[offset] != (PN532_COMMAND_WRITEREGISTER + 1))
    return 0;
  
  pn532_packetbuffer[0] = PN532_COMMAND_READREGISTER;
  pn532_packetbuffer[1] = 0x63;
  pn532_packetbuffer[2] = 0x11;
  pn532_packetbuffer[3] = 0x63;
  pn532_packetbuffer[4] = 0x12;
  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, (2 * 2 + 1), 1000))
    return 0x0;
        
  if (!waitready(hspi, 1000)) {
    return 0x0;
  }
        
  readdata(hspi, pn532_packetbuffer, (7 + 2), 1000);
  offset =  6;
  if(pn532_packetbuffer[offset] != (PN532_COMMAND_READREGISTER + 1))
    return 0;

  *((uint8_t *)crc) =  pn532_packetbuffer[offset + 2];
  *((uint8_t *)crc + 1) =  pn532_packetbuffer[offset + 1];
  
  return 1;
}


uint8_t   	Transceive(SPI_HandleTypeDef *hspi, uint8_t *tr, uint8_t ntr, uint8_t *rec, uint8_t *nrec){
  
  uint8_t j = 0, i, n;
  
  pn532_packetbuffer[ j++ ] = PN532_COMMAND_WRITEREGISTER;
  pn532_packetbuffer[ j++ ] = 0x63;     // CRC Disable
  pn532_packetbuffer[ j++ ] = 0x02;
  pn532_packetbuffer[ j++ ] = 0x00;
  pn532_packetbuffer[ j++ ] = 0x63;
  pn532_packetbuffer[ j++ ] = 0x03;
  pn532_packetbuffer[ j++ ] = 0x00;
  
  for(i = 0; i < ntr; i++){           // FIFO
    pn532_packetbuffer[j++] = 0x63;
    pn532_packetbuffer[j++] = 0x39;
    pn532_packetbuffer[j++] = tr[i];
  }

  pn532_packetbuffer[ j++ ] = 0x63;     //Transceive
  pn532_packetbuffer[ j++ ] = 0x31;
  pn532_packetbuffer[ j++ ] = 0x0C; 
  pn532_packetbuffer[ j++ ] = 0x63;     //Start
  pn532_packetbuffer[ j++ ] = 0x3D;
  pn532_packetbuffer[ j++ ] = 0x80; 

  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, j , 1000))
    return 0x0;
  if (!waitready(hspi, 1000)) {
    return 0x0;
  }
  
  readdata(hspi, pn532_packetbuffer, 7, 1000);
  int offset = 6;
  if(pn532_packetbuffer[offset] != (PN532_COMMAND_WRITEREGISTER + 1))
    return 0;
  
  // Читаем число байт ответа
  j = 0;
  n =  ((*nrec) > 16)? 16: (*nrec);
  pn532_packetbuffer[ j++ ] = PN532_COMMAND_READREGISTER;
  pn532_packetbuffer[ j++ ] = 0x63;     
  pn532_packetbuffer[ j++ ] = 0x3A;

  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, j, 1000)){
	    return 0x0;
  }

  if (!waitready(hspi, 1000)) {
    return 0x0;
  }

  readdata(hspi, pn532_packetbuffer, (7 + n), 1000);
  offset =  6;
  if(pn532_packetbuffer[offset] != (PN532_COMMAND_READREGISTER + 1))
    return 0;
  *nrec =  pn532_packetbuffer[ offset + 1 ];
  if( *nrec == 0){
	  return 1;
  }

  // Читаем ответ
  j = 0;
  n =  ((*nrec) > 16)? 16: (*nrec);
  pn532_packetbuffer[ j++ ] = PN532_COMMAND_READREGISTER;
  for( i = 0; i < n; i++){
    pn532_packetbuffer[ j++ ] = 0x63;
    pn532_packetbuffer[ j++ ] = 0x39;
  }
  
  if (! sendCommandCheckAck(hspi, pn532_packetbuffer, j, 1000)){
	    return 0x0;
  }
        
  if (!waitready(hspi, 1000)) {
    return 0x0;
  }
        
  readdata(hspi, pn532_packetbuffer, (7 + n), 1000);
  offset =  6;
  if(pn532_packetbuffer[offset] != (PN532_COMMAND_READREGISTER + 1))
    return 0;
  
  j = offset + 1;
  for(i = 0; i < n; i++){
    rec[ i ]= pn532_packetbuffer[ j++ ];
  }
  
  	j = 0;

  	pn532_packetbuffer[j++] = PN532_COMMAND_WRITEREGISTER;

  	pn532_packetbuffer[j++] = 0x63;     // CRC Disable
  	pn532_packetbuffer[j++] = 0x02;
  	pn532_packetbuffer[j++] = 0x80;
  	pn532_packetbuffer[j++] = 0x63;
  	pn532_packetbuffer[j++] = 0x03;
  	pn532_packetbuffer[j++] = 0x80;


  	if (!sendCommandCheckAck(hspi, pn532_packetbuffer, j, 1000))
  		return 0x0;
  	if (!waitready(hspi, 1000)) {
  		return 0x0;
  	}

  	readdata(hspi, pn532_packetbuffer, 7, 1000);
  	if (pn532_packetbuffer[offset] != (PN532_COMMAND_WRITEREGISTER + 1))
  		return 0;

  return 1;  
}


uint8_t sendBits(SPI_HandleTypeDef *hspi, uint8_t send, uint8_t nbits, uint8_t *rec, uint8_t *nrec){
	uint16_t Adr[] = {0x6302, 0x6303, 0x633D};
	uint8_t r1[] = {0x00, 0x00, 0x00}, scs;

	r1[2] = 0x07 & nbits;
	writeRegisters(hspi,Adr,r1,3);

	scs = InCommunicateThru(&hspi1, &send ,1,rec, nrec);

	r1[0] = 0x80;
	r1[1] = 0x80;
	r1[2] = 0x00;
	writeRegisters(hspi,Adr,r1,3);

	return scs;
}


uint8_t sendByte(SPI_HandleTypeDef *hspi, uint8_t send, uint8_t *rec, uint8_t *nrec){
	uint16_t Adr[] = {0x6302, 0x6303};
	uint8_t r1[] = {0x00, 0x00}, scs;

	writeRegisters(hspi,Adr,r1,2);

	scs = InCommunicateThru(&hspi1, &send ,1,rec, nrec);

	r1[0] = 0x80;
	r1[1] = 0x80;
	writeRegisters(hspi,Adr,r1,2);

	return scs;
}


uint8_t unlock(SPI_HandleTypeDef *hspi){
	uint8_t scs;

	InCommunicateThru(&hspi1,abtHalt,2,buf,&nbuf);
	nbuf = 16;
	scs = sendBits(&hspi1,0x40,7, buf, &nbuf);
	if(!scs || (nbuf != 1) || (buf[0] != 0x0A) ){
		return 0;
	}
	nbuf = 16;
	scs = sendByte(&hspi1, 0x43, buf, &nbuf);
	if(!scs || (nbuf != 1) || (buf[0] != 0x0A) ){
		return 0;
	}
	return 1;
}


