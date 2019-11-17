/*
 * MifareClassic.h
 *
 *  Created on: 13.05.2013
 *      Author: skuser
 */

#ifndef MIFARECLASSIC_H_
#define MIFARECLASSIC_H_

#include "ISO14443-3A.h"
#include "Configuration.h"

#define MIFARE_CLASSIC_UID_SIZE       ISO14443A_UID_SIZE_SINGLE
#define MIFARE_CLASSIC_MINI_MEM_SIZE  320
#define MIFARE_CLASSIC_1K_MEM_SIZE    1024
#define MIFARE_CLASSIC_4K_MEM_SIZE    4096

#define BITS_PER_BYTE 8
#define ISO14443A_APP_NO_RESPONSE       0x0000
#define ISO14443A_APP_CUSTOM_PARITY     0x1000

void MifareClassicAppInitMini4B(void);
void MifareClassicAppInit1K(void);
void MifareClassicAppInit4K(void);
void MifareClassicAppInit1K7B(void);
void MifareClassicAppInit4K7B(void);
void MifareClassicAppReset(void);
void MifareClassicAppTask(void);

uint16_t MifareClassicAppProcess(uint8_t *Buffer, uint16_t BitCount);

void MifareClassicGetUid(ConfigurationUidType Uid);
void MifareClassicSetUid(ConfigurationUidType Uid);


#endif /* MIFARECLASSIC_H_ */
