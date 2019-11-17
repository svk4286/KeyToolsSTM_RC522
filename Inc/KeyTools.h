/*
 * KeyTools.h
 *
 *  Created on: 8 рту. 2019 у.
 *      Author: Sergey
 */

#ifndef INC_KEYTOOLS_H_
#define INC_KEYTOOLS_H_

#include "stm32f1xx_hal.h"

#define BUFFER_SIZE  64
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


void keytools(void);
uint8_t keygrab(uint8_t *nkey);
void conv(uint32_t *a, uint8_t *d);

#endif /* INC_KEYTOOLS_H_ */
