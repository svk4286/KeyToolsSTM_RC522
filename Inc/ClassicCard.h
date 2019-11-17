/*
 * ClassicCard.h
 *
 *  Created on: 20 èþë. 2019 ã.
 *      Author: Sergey
 */

#ifndef INC_CLASSICCARD_H_
#define INC_CLASSICCARD_H_

#include "stm32f1xx_hal.h"
#include "Configuration.h"


void MemoryReadBlock(void* Buffer, uint16_t Address, uint16_t ByteCount);
void MemoryWriteBlock(const void* Buffer, uint16_t Address, uint16_t ByteCount);
void RandomGetBuffer(uint8_t *Buffer, uint8_t ByteCount);

#endif /* INC_CLASSICCARD_H_ */
