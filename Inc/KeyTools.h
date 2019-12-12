

#ifndef INC_KEYTOOLS_H_
#define INC_KEYTOOLS_H_

#include "stm32f1xx_hal.h"

#define CODEC_BUFFER_SIZE           64
#define ISO14443A_BUFFER_PARITY_OFFSET    (CODEC_BUFFER_SIZE/2)

#define BUFFER_SIZE  CODEC_BUFFER_SIZE
#define BUFFER_PAR   (BUFFER_SIZE/2)
#define FRAME_SIZE	40

#define CMD				0xA5
#define NOP				0x00
#define GET_INFO		0x02
#define GET_UID			0x04
#define GET_GRAB		0x06
#define AUTHENT_KEY		0x08
#define READ_BLOCK		0x0A
#define WRITE_BLOCK		0x0C
#define UNLOCK			0x0E
#define EMULATOR		0x10
#define READ_CARD		0x12
#define WRITE_CARD		0x14
#define BREAK			0x16


void keytools(void);
uint8_t keygrab(uint8_t *nkey);
void conv(uint32_t *a, uint8_t *d);
uint8_t emul();

#endif /* INC_KEYTOOLS_H_ */
