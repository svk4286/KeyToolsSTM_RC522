
#include <stdlib.h>
#include "Get_A.h"
#include "Put_A.h"
#include "pn532.h"
#include "MifClass.h"
#include "Macros.h"
#include "RC522.h"
#include "KeyTools.h"

#define AUT_MAX 10

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern __IO uint32_t uwTick;
extern uint8_t UidCL1[], uidLength;
extern Uid uid;
extern uint8_t RC522;


uint8_t Tx[512], Rx[64];
uint16_t  fv_1;
uint8_t RxOK, GtCmd;
uint32_t TagChal[AUT_MAX], ReadChall[AUT_MAX], ReadResp[AUT_MAX];
uint8_t nkey;
uint8_t j_aut, ZeroFilter, GrabOK, EndOfFrame, KeyAB[AUT_MAX], BlockNumber[AUT_MAX];
uint8_t data_A[FRAME_SIZE][BUFFER_SIZE], data_B[FRAME_SIZE][BUFFER_SIZE];
uint8_t ndata_A[FRAME_SIZE], ndata_B[FRAME_SIZE];
uint8_t nd;
//uint8_t key[6];
MIFARE_Key key;
uint8_t bufferSize;


void conv(uint32_t *a, uint8_t *d){
	*a = d[0];
	for(int i = 1; i < 4; i++){
		*a <<= 8;
		*a += d[i];
	}
}



void InitPN532_1(){



}


void InitVirtCard(){
	uint16_t Adr;
	uint8_t r1 = 0x13;

	SAM_VirtualCard(&hspi1);
	Adr = 0x6106;
	writeRegister(&hspi1, Adr, r1);

	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	MifareClassicAppInit1K();

}


void HAL_UART_RxCpltCallback( UART_HandleTypeDef *  huart ) {

	if(!GtCmd){
		if(Rx[2] == 3){
			RxOK = 1;
			return;
		}
		HAL_UART_Receive_DMA(huart, Rx + 3, Rx[2] - 3);
		GtCmd = 1;
	}else{
		RxOK = 1;
	}
}


void keytools() {
	uint8_t i;

	if (Rx[0] != CMD) {
		return;
	}

	switch (Rx[1]) {

	case NOP:
		return;
		break;

	case GET_INFO:			// Версия ПО и Firmware PN532
		CS_LOW(&hspi1)
		;
		HAL_Delay(2);
		CS_HIGH(&hspi1);
		HAL_Delay(2);
		if (!(fv_1 = getFirmwareVersion(&hspi1, 100))) {
			Tx[0] = CMD + 1;
			Tx[1] = 0;
			Tx[2] = 4;
			Tx[3] = 1;				// Error code
			HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
			break;
		}
		Tx[0] = CMD + 1;
		Tx[1] = GET_INFO + 1;
		Tx[2] = 9;
		Tx[3] = (uint8_t) (fv_1 & 0xFF);
		Tx[4] = (uint8_t) (fv_1 >> 8);
		Tx[5] = 0;
		Tx[6] = 1;
		if(RC522){
			Tx[7] = 0;
			Tx[8] = 1;
		}
		else{
			Tx[7] = 0;
			Tx[8] = 0;
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	case GET_UID:		// UID метки
		if (RC522) {
			PCD_Init(&hspi2, 1000);
			HAL_Delay(5);
			if (!PICC_IsNewCardPresent(&hspi2, 100)
					|| !PICC_ReadCardSerial(&hspi2, 100) || uid.size != 4) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code
			} else {
				Tx[0] = CMD + 1;
				Tx[1] = GET_UID + 1;
				Tx[2] = 7;
				Tx[3] = uid.uidByte[3];
				Tx[4] = uid.uidByte[2];
				Tx[5] = uid.uidByte[1];
				Tx[6] = uid.uidByte[0];
			}
		} else {
			SAM_Config(&hspi1);
			if ((!readPassiveTargetID(&hspi1, PN532_MIFARE_ISO14443A, UidCL1,
					&uidLength, 50)) || uidLength != 4) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;			// Error code
				HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
				break;
			}
			Tx[0] = CMD + 1;
			Tx[1] = GET_UID + 1;
			Tx[2] = 7;
			Tx[3] = UidCL1[3];
			Tx[4] = UidCL1[2];
			Tx[5] = UidCL1[1];
			Tx[6] = UidCL1[0];
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;


	case GET_GRAB:
		InitVirtCard();
		UidCL1[0] = Rx[3];
		UidCL1[1] = Rx[4];
		UidCL1[2] = Rx[5];
		UidCL1[3] = Rx[6];

		if (!keygrab(&nkey)) {
			Tx[0] = CMD + 1;
			Tx[1] = 0;
			Tx[2] = 4;
			Tx[3] = 1;				// Error code
			HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
			break;
		}
		Tx[0] = CMD + 1;
		Tx[1] = GET_GRAB + 1;
		Tx[2] = 5;
		Tx[3] = ZeroFilter;
		Tx[4] = nkey;
		for (uint8_t i = 0; i < nkey; i++) {
			Tx[Tx[2]++] = KeyAB[i];
			Tx[Tx[2]++] = BlockNumber[i];
			*(uint32_t*) (&Tx[Tx[2]]) = TagChal[i];
			Tx[2] += 4;
			*(uint32_t*) (&Tx[Tx[2]]) = ReadChall[i];
			Tx[2] += 4;
			*(uint32_t*) (&Tx[Tx[2]]) = ReadResp[i];
			Tx[2] += 4;
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	case AUTHENT_KEY:
		if(RC522){
			for(i = 0; i < MF_KEY_SIZE; i++){
				key.keyByte[i] = Rx[i + 5];
			}
			if (STATUS_OK != PCD_Authenticate(&hspi2, Rx[4] + 0x60, Rx[3], &key, &uid, 100)) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = AUTHENT_KEY + 1;
				Tx[2] = 3;
			}
		}
		else{
			if (!mifareclassic_AuthenticateBlock(&hspi1, UidCL1, Rx[3], Rx[4], (Rx + 5))){
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code
			} else {
				Tx[0] = CMD + 1;
				Tx[1] = AUTHENT_KEY + 1;
				Tx[2] = 3;
			}
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	case READ_BLOCK:
		if(RC522){
			bufferSize = 18;
			if (STATUS_OK != MIFARE_Read(&hspi2, Rx[3], (Tx + 3), &bufferSize, 1000)) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = READ_BLOCK + 1;
				Tx[2] = 3 + 16;
			}
		}
		else{
			if (!mifareclassic_ReadDataBlock(&hspi1, Rx[3], (Tx + 3))) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = READ_BLOCK + 1;
				Tx[2] = 3 + 16;
			}
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	case WRITE_BLOCK:
		if(RC522){
			bufferSize = 16;
			if (STATUS_OK != MIFARE_Write(&hspi2, Rx[3], (Rx + 4), bufferSize, 1000)) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = WRITE_BLOCK + 1;
				Tx[2] = 3;
			}
		}
		else{
			if (!mifareclassic_WriteDataBlock(&hspi1, Rx[3], (Rx + 4))) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = WRITE_BLOCK + 1;
				Tx[2] = 3;
			}
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	case UNLOCK:
		if(RC522){
			if (!MIFARE_OpenUidBackdoor(&hspi2, 1000)) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = UNLOCK + 1;
				Tx[2] = 3;
			}
		}
		else{
			if (!unlock(&hspi1)) {
				Tx[0] = CMD + 1;
				Tx[1] = 0;
				Tx[2] = 4;
				Tx[3] = 1;				// Error code

			} else {
				Tx[0] = CMD + 1;
				Tx[1] = UNLOCK + 1;
				Tx[2] = 3;
			}
		}
		HAL_UART_Transmit(&huart1, Tx, Tx[2], 1000);
		break;

	default:
		break;

	}

}


uint8_t keygrab(uint8_t *nkey){

	uint16_t inbit, outbit;
	uint8_t cnt[170], ncnt;
	uint8_t  lastbit = 0;
	uint8_t scs;

	nd = 0;
	ZeroFilter = 0;
	GrabOK = 0;
	EndOfFrame = 0;
	j_aut = 0;

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	srand(uwTick);


	while(1){

		do{
			scs = getStream_A(cnt, &ncnt, 1000);
			if(!scs){
				return 0;
			}
		}while(!ncnt);
		inbit = ConvertStream_A(cnt, ncnt, data_A[nd], &ndata_A[nd], &lastbit);

		for(uint8_t i = 0; i < ndata_A[nd]; i++){
			data_B[nd][i] = data_A[nd][i];
		}
		outbit = MifareClassicAppProcess( data_B[nd], inbit);
		if(EndOfFrame){
			break;
		}
		if(GrabOK){
			GrabOK = 0;
			if(!ZeroFilter || (j_aut >= AUT_MAX)){
				break;
			}
		}

		ndata_B[nd] = 0;
		if(outbit == 4){
			SendBits_A(data_B[nd][0], 4, lastbit);
			ndata_B[nd] = 1;
		}
		else if(outbit >= 8){
			ndata_B[nd] = outbit  >> 3;
			PutData_A(data_B[nd], ndata_B[nd], lastbit);
		}
		nd++;
		if(nd >= FRAME_SIZE){
			return 0;
		}
	}

	*nkey = j_aut;
	return 1;
}
