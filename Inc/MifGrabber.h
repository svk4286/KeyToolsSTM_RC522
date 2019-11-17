
#ifndef INC_MIFCLASS_H_
#define INC_MIFCLASS_H_

#include "stm32f1xx_hal.h"

#define bool	_Bool
#define true	1
#define false	0

#define BITS_PER_BYTE 8

#define MFCLASSIC_1K_ATQA_VALUE     0x0004
#define MFCLASSIC_1K_7B_ATQA_VALUE  0x0044
#define MFCLASSIC_4K_ATQA_VALUE     0x0002
#define MFCLASSIC_4K_7B_ATQA_VALUE  0x0042

#define MFCLASSIC_1K_SAK_VALUE 		0x08
#define MFCLASSIC_4K_SAK_VALUE  	0x18
#define SAK_UID_NOT_FINISHED		0x04

#define MEM_UID_CL1_ADDRESS         0x00
#define MEM_UID_CL1_SIZE            4
#define MEM_UID_BCC1_ADDRESS        0x04
#define MEM_UID_CL2_ADDRESS        	0x03
#define MEM_UID_CL2_SIZE            4
#define MEM_KEY_A_OFFSET            48        /* Bytes */
#define MEM_KEY_B_OFFSET            58        /* Bytes */
#define MEM_KEY_BIGSECTOR_OFFSET	192
#define MEM_KEY_SIZE                6        /* Bytes */
#define MEM_ACC_GPB_SIZE            4        /* Bytes */
#define MEM_SECTOR_ADDR_MASK        0xFC
#define MEM_BIGSECTOR_ADDR_MASK		0xF0
#define MEM_BYTES_PER_BLOCK         16        /* Bytes */
#define MEM_VALUE_SIZE              4       /* Bytes */

/* NXP Originality check */
/* Sector 18/Block 68..71 is used to store signature data for NXP originality check */
#define MEM_EV1_SIGNATURE_BLOCK     68
#define MEM_EV1_SIGNATURE_TRAILOR   ((MEM_EV1_SIGNATURE_BLOCK + 3 ) * MEM_BYTES_PER_BLOCK)

#define ACK_NAK_FRAME_SIZE          4         /* Bits */
#define ACK_VALUE                   0x0A
#define NAK_INVALID_ARG             0x00
#define NAK_CRC_ERROR               0x01
#define NAK_NOT_AUTHED              0x04
#define NAK_EEPROM_ERROR            0x05
#define NAK_OTHER_ERROR             0x06

#define CMD_AUTH_A                  0x60
#define CMD_AUTH_B                  0x61
#define CMD_AUTH_FRAME_SIZE         2         /* Bytes without CRCA */
#define CMD_AUTH_RB_FRAME_SIZE      4        /* Bytes */
#define CMD_AUTH_AB_FRAME_SIZE      8        /* Bytes */
#define CMD_AUTH_BA_FRAME_SIZE      4        /* Bytes */
#define CMD_HALT                    0x50
#define CMD_HALT_FRAME_SIZE         2        /* Bytes without CRCA */
#define CMD_READ                    0x30
#define CMD_READ_FRAME_SIZE         2         /* Bytes without CRCA */
#define CMD_READ_RESPONSE_FRAME_SIZE 16 /* Bytes without CRCA */
#define CMD_WRITE                   0xA0
#define CMD_WRITE_FRAME_SIZE        2         /* Bytes without CRCA */
#define CMD_DECREMENT               0xC0
#define CMD_DECREMENT_FRAME_SIZE    2         /* Bytes without CRCA */
#define CMD_INCREMENT               0xC1
#define CMD_INCREMENT_FRAME_SIZE    2         /* Bytes without CRCA */
#define CMD_RESTORE                 0xC2
#define CMD_RESTORE_FRAME_SIZE      2         /* Bytes without CRCA */
#define CMD_SIG_READ                0xC2
#define CMD_SIG_READ_FRAME_SIZE     1         /* Bytes without CRCA */
#define CMD_TRANSFER                0xB0
#define CMD_TRANSFER_FRAME_SIZE     2         /* Bytes without CRCA */
#define CMD_CHINESE_UNLOCK          0x40
#define CMD_CHINESE_WIPE            0x41
#define CMD_CHINESE_UNLOCK_RW       0x43




/*
Source: NXP: MF1S50YYX Product data sheet

Access conditions for the sector trailer

Access bits     Access condition for                   Remark
            KEYA         Access bits  KEYB
C1 C2 C3        read  write  read  write  read  write
0  0  0         never key A  key A never  key A key A  Key B may be read[1]
0  1  0         never never  key A never  key A never  Key B may be read[1]
1  0  0         never key B  keyA|B never never key B
1  1  0         never never  keyA|B never never never
0  0  1         never key A  key A  key A key A key A  Key B may be read,
                                                       transport configuration[1]
0  1  1         never key B  keyA|B key B never key B
1  0  1         never never  keyA|B key B never never
1  1  1         never never  keyA|B never never never

[1] For this access condition key B is readable and may be used for data
*/
#define ACC_TRAILOR_READ_KEYA   0x01
#define ACC_TRAILOR_WRITE_KEYA  0x02
#define ACC_TRAILOR_READ_ACC    0x04
#define ACC_TRAILOR_WRITE_ACC   0x08
#define ACC_TRAILOR_READ_KEYB   0x10
#define ACC_TRAILOR_WRITE_KEYB  0x20



/*
Access conditions for data blocks
Access bits Access condition for 				Application
C1 C2 C3 	read 	write 	increment 	decrement,
                                                transfer,
                                                restore

0 0 0 		key A|B key A|B key A|B 	key A|B 	transport configuration
0 1 0 		key A|B never 	never 		never 		read/write block
1 0 0 		key A|B key B 	never 		never 		read/write block
1 1 0 		key A|B key B 	key B 		key A|B 	value block
0 0 1 		key A|B never 	never 		key A|B 	value block
0 1 1 		key B 	key B 	never 		never 		read/write block
1 0 1 		key B 	never 	never 		never 		read/write block
1 1 1 		never 	never 	never 		never 		read/write block

*/
#define ACC_BLOCK_READ      0x01
#define ACC_BLOCK_WRITE     0x02
#define ACC_BLOCK_INCREMENT 0x04
#define ACC_BLOCK_DECREMENT 0x08

#define KEY_A 0
#define KEY_B 1


uint16_t MifareClassicGrabber(uint8_t* Buffer, uint16_t BitCount);
void MifareClassicAppInit1K(void);


#endif /* INC_MIFCLASS_H_ */
