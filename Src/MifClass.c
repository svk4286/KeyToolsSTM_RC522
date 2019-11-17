
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "MifClass.h"
#include "ISO14443-3A.h"
#include "KeyTools.h"


uint8_t UidCL1[ISO14443A_CL_UID_SIZE] = {0xE4, 0x70, 0x51, 0x73};
uint8_t uidLength;
uint8_t UidCL2[ISO14443A_CL_UID_SIZE];

extern uint32_t TagChal[], ReadChall[], ReadResp[];
extern uint8_t j_aut, ZeroFilter, EndOfFrame, GrabOK, KeyAB[], BlockNumber[];
static uint8_t X_9320;

static enum {
    STATE_HALT,
    STATE_IDLE,
    STATE_CHINESE_IDLE,
    STATE_CHINESE_WRITE,
    STATE_READY1,
    STATE_READY2,
    STATE_ACTIVE,
    STATE_AUTHING,
    STATE_AUTHED_IDLE,
    STATE_WRITE,
    STATE_INCREMENT,
    STATE_DECREMENT,
    STATE_RESTORE
} State;

//static uint8_t CardResponse[4];
//static uint8_t ReaderResponse[4];
//static uint8_t CurrentAddress;
//static uint8_t KeyInUse;
//static uint8_t BlockBuffer[MEM_BYTES_PER_BLOCK];
//static uint8_t AccessConditions[MEM_ACC_GPB_SIZE]; /* Access Conditions + General purpose Byte */
static uint8_t AccessAddress;
static uint16_t CardATQAValue;
static uint8_t CardSAKValue;
static bool FromHalt = false;

#define BYTE_SWAP(x) (((uint8_t)(x)>>4)|((uint8_t)(x)<<4))
#define NO_ACCESS 0x07

void ISO14443AAppendCRCA(void* Buffer, uint16_t ByteCount) {
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    *DataPtr++ = (Checksum >> 0) & 0x00FF;
    *DataPtr = (Checksum >> 8) & 0x00FF;
}

bool ISO14443ACheckCRCA(void* Buffer, uint16_t ByteCount)
{
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    return (DataPtr[0] == ((Checksum >> 0) & 0xFF)) && (DataPtr[1] == ((Checksum >> 8) & 0xFF));
}


bool ISO14443ASelect(void* Buffer, uint16_t* BitCount, uint8_t* UidCL, uint8_t SAKValue)
{
    uint8_t* DataPtr = (uint8_t*) Buffer;
    uint8_t NVB = DataPtr[1];
    //uint8_t CollisionByteCount = (NVB >> 4) & 0x0F;
    //uint8_t CollisionBitCount =  (NVB >> 0) & 0x0F;

    switch (NVB) {
    case ISO14443A_NVB_AC_START:
        /* Start of anticollision procedure.
        * Send whole UID CLn + BCC */
        DataPtr[0] = UidCL[0];
        DataPtr[1] = UidCL[1];
        DataPtr[2] = UidCL[2];
        DataPtr[3] = UidCL[3];
        DataPtr[4] = ISO14443A_CALC_BCC(DataPtr);

        *BitCount = ISO14443A_CL_FRAME_SIZE;

        X_9320 = 1;

        return false;

    case ISO14443A_NVB_AC_END:
        /* End of anticollision procedure.
        * Send SAK CLn if we are selected. */
        if (    (DataPtr[2] == UidCL[0]) &&
                (DataPtr[3] == UidCL[1]) &&
                (DataPtr[4] == UidCL[2]) &&
                (DataPtr[5] == UidCL[3]) ) {

            DataPtr[0] = SAKValue;
            ISO14443AAppendCRCA(Buffer, 1);

            *BitCount = ISO14443A_SAK_FRAME_SIZE;
            return true;
        } else {
            /* We have not been selected. Don't send anything. */
            *BitCount = 0;
            return false;
        }
    default:
    {
        uint8_t CollisionByteCount = ((NVB >> 4) & 0x0f) - 2;
        uint8_t CollisionBitCount  = (NVB >> 0) & 0x0f;
        uint8_t mask = 0xFF >> (8 - CollisionBitCount);
        // Since the UidCL does not contain the BCC, we have to distinguish here
        if (
                ((CollisionByteCount == 5 || (CollisionByteCount == 4 && CollisionBitCount > 0)) && memcmp(UidCL, &DataPtr[2], 4) == 0 && (ISO14443A_CALC_BCC(UidCL) & mask) == (DataPtr[6] & mask))
                ||
                (CollisionByteCount == 4 && CollisionBitCount == 0 && memcmp(UidCL, &DataPtr[2], 4) == 0)
                ||
                (CollisionByteCount < 4 && memcmp(UidCL, &DataPtr[2], CollisionByteCount) == 0 && (UidCL[CollisionByteCount] & mask) == (DataPtr[CollisionByteCount + 2] & mask))
        )
        {
            DataPtr[0] = UidCL[0];
            DataPtr[1] = UidCL[1];
            DataPtr[2] = UidCL[2];
            DataPtr[3] = UidCL[3];
            DataPtr[4] = ISO14443A_CALC_BCC(DataPtr);

            *BitCount = ISO14443A_CL_FRAME_SIZE;
        } else {
            *BitCount = 0;
        }
        return false;
    }
        /* TODO: No anticollision supported */
        *BitCount = 0;
        return false;
    }
}


bool ISO14443AWakeUp(void* Buffer, uint16_t* BitCount, uint16_t ATQAValue, bool FromHalt)
{
    uint8_t* DataPtr = (uint8_t*) Buffer;

    if ( ((! FromHalt) && (DataPtr[0] == ISO14443A_CMD_REQA)) ||
         (DataPtr[0] == ISO14443A_CMD_WUPA) ){
        DataPtr[0] = (ATQAValue >> 0) & 0x00FF;
        DataPtr[1] = (ATQAValue >> 8) & 0x00FF;

        *BitCount = ISO14443A_ATQA_FRAME_SIZE;

        return true;
    } else {
        *BitCount = 0;

        return false;
    }
}


void MifareClassicAppInit1K(void)
{
    State = STATE_IDLE;
    CardATQAValue = MFCLASSIC_1K_ATQA_VALUE;
    CardSAKValue = MFCLASSIC_1K_SAK_VALUE;
    FromHalt = false;
}


uint16_t MifareClassicAppProcess(uint8_t* Buffer, uint16_t BitCount){

	if ( (BitCount == 7) && (Buffer[0] == 0x50)){
		if(ZeroFilter){
			EndOfFrame = 1;
		}
		ZeroFilter = 1;
		X_9320 = 0;
		return ISO14443A_APP_NO_RESPONSE;
	}

    /* Wakeup and Request may occure in all states */
    if ( (BitCount == 7) &&
         /* precheck of WUP/REQ because ISO14443AWakeUp destroys BitCount */
         (((State != STATE_HALT) && (Buffer[0] == ISO14443A_CMD_REQA)) ||
         (Buffer[0] == ISO14443A_CMD_WUPA) )){
        FromHalt = (State == STATE_HALT);
        if (ISO14443AWakeUp(Buffer, &BitCount, CardATQAValue, FromHalt)) {
            AccessAddress = 0xff;
            State = STATE_READY1;
            return BitCount;
        }
    }

    switch(State) {
    case STATE_IDLE:
    case STATE_HALT:
        FromHalt = State == STATE_HALT;
        if (ISO14443AWakeUp(Buffer, &BitCount, CardATQAValue, FromHalt)) {
            State = STATE_READY1;
            return BitCount;
        }
        break;

    case STATE_READY1:
        if (ISO14443AWakeUp(Buffer, &BitCount, CardATQAValue, FromHalt)) {
            State = FromHalt ? STATE_HALT : STATE_IDLE;
            return ISO14443A_APP_NO_RESPONSE;
        } else if (Buffer[0] == ISO14443A_CMD_SELECT_CL1) {

            /* Load UID CL1 and perform anticollision */

            /* For Longer UIDs indicate that more UID-Bytes follow (-> CL2) */

//                MemoryReadBlock(UidCL1, MEM_UID_CL1_ADDRESS, MEM_UID_CL1_SIZE);
        	if (ISO14443ASelect(Buffer, &BitCount, UidCL1, CardSAKValue)) {
                        AccessAddress = 0xff; /* invalid, force reload */
            State = STATE_ACTIVE;
        	}
            return BitCount;
        } else {
            /* Unknown command. Enter HALT state. */
            State = STATE_IDLE;
        }
        break;

    case STATE_ACTIVE:
        if (ISO14443AWakeUp(Buffer, &BitCount, CardATQAValue, FromHalt)) {
            State = FromHalt ? STATE_HALT : STATE_IDLE;
            return ISO14443A_APP_NO_RESPONSE;
        } else if (Buffer[0] == CMD_HALT) {
            /* Halts the tag. According to the ISO14443, the second
            * byte is supposed to be 0. */
            if (Buffer[1] == 0) {
                if (ISO14443ACheckCRCA(Buffer, CMD_HALT_FRAME_SIZE)) {
                    /* According to ISO14443, we must not send anything
                    * in order to acknowledge the HALT command. */
                    //LogEntry(LOG_INFO_APP_CMD_HALT, NULL, 0);

                    State = STATE_HALT;
                    return ISO14443A_APP_NO_RESPONSE;
                } else {
                    Buffer[0] = NAK_CRC_ERROR;
                    return ACK_NAK_FRAME_SIZE;
                }
            } else {
                Buffer[0] = NAK_INVALID_ARG;
                return ACK_NAK_FRAME_SIZE;
            }
        } else if ( (Buffer[0] == CMD_AUTH_A) || (Buffer[0] == CMD_AUTH_B)) {
            if (ISO14443ACheckCRCA(Buffer, CMD_AUTH_FRAME_SIZE)) {

				KeyAB[j_aut] = Buffer[0] - CMD_AUTH_A;
				BlockNumber[j_aut] = Buffer[1];

            	uint8_t *p;
				TagChal[j_aut] = rand();
				p = (uint8_t*) (&TagChal[j_aut]);
				for (uint8_t i = 0; i < 4; i++) {
					Buffer[3 - i] = *(p++);
				}

                State = STATE_AUTHING;
                return CMD_AUTH_RB_FRAME_SIZE * BITS_PER_BYTE;
            } else {
                Buffer[0] = NAK_CRC_ERROR;
                return ACK_NAK_FRAME_SIZE;
            }

            /* EV1 READ_SIG command is */
            /* same as CMD_RESTORE but has no operand, rely on CRC here! */
      } else if ((Buffer[0] == CMD_SIG_READ) &&
                 (Buffer[1] == 0xe0) &&
                 (Buffer[2] == 0xb4)) {

			Buffer[0] = NAK_CRC_ERROR;
			return ACK_NAK_FRAME_SIZE;
        } else if (  (Buffer[0] == CMD_READ) || (Buffer[0] == CMD_WRITE) || (Buffer[0] == CMD_DECREMENT)
                  || (Buffer[0] == CMD_INCREMENT) || (Buffer[0] == CMD_RESTORE) || (Buffer[0] == CMD_TRANSFER) ) {
            State = STATE_IDLE;
            Buffer[0] = NAK_NOT_AUTHED;
            //LogEntry(LOG_ERR_APP_NOT_AUTHED, NULL, 0);
            return ACK_NAK_FRAME_SIZE;
        } else {
            /* Unknown command. Enter HALT state. */
            //LogEntry(LOG_INFO_APP_CMD_UNKNOWN, Buffer, (BitCount+7)/8);
            State = STATE_IDLE;
        return ISO14443A_APP_NO_RESPONSE;
        }
        break;

    case STATE_AUTHING:
    	if(BitCount != 8 * 8){
    		State = STATE_IDLE;
    		break;
    	}

    	if(!ZeroFilter || ( ZeroFilter && X_9320)){
    		conv(&ReadChall[j_aut], Buffer);
    		conv(&ReadResp[j_aut], &Buffer[4]);
    		GrabOK = 1;
    		j_aut++;
    		X_9320 = 0;
    	}

		State = STATE_IDLE;
        break;

    default:
        /* Unknown state? Should never happen. */
        break;

    }

    return ISO14443A_APP_NO_RESPONSE;
}
