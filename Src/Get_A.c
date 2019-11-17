#include "Get_A.h"
#include "KeyTools.h"

#include "Macros.h"

extern TIM_HandleTypeDef htim1;
extern __IO uint32_t uwTick;

#define TCNT htim1.Instance->CNT

#define ONE 90
#define OT 155
#define TWO 220
#define D 20


uint8_t insbit(uint8_t *buf, uint8_t *ibit, uint8_t *ibyte, uint8_t *par, uint8_t b) {
	if ((*ibit) < 8) {
		(*par) ^= b;
		b <<= (*ibit)++;
		*buf |= b;
	}
	else {
		*(buf + BUFFER_PAR) = (*par)^1 + (b << 4);
		(*ibyte)++;
		(*ibit) = 0;
		(*par) = 0;
	}
	return 1;
}


uint8_t getStream_A(uint8_t *cnt, uint8_t *n, uint32_t timeout) {
	uint16_t Tprev, t1;
	uint8_t i;
	uint32_t tickstart;
	tickstart = uwTick;
	*n = 0;

	TCNT = 0;
	while((!TCNT) &&  ((uwTick - tickstart) < timeout));				// Ждем несущей частоты
	if(!TCNT){
		return 0;
	}

	TCNT = 0;
	Tprev = TCNT;
	i = 0;
	while ( (TCNT != Tprev) && ((uwTick - tickstart) < timeout)) {	// Ожидаем 1-го импульса
		Tprev = TCNT;
	}
	if( TCNT != Tprev){
		return 0;
	}
//	t1 = TCNT;
	do {
		TCNT = 0;
		while ((!TCNT) && ((uwTick - tickstart) < timeout));
		if(!TCNT){
			return 0;
		}
		while (((t1 = TCNT) != Tprev) && ( TCNT < (TWO + 20))) {
			Tprev = TCNT;
		}
		if (TCNT != Tprev)
			break;
		if ((t1 > (ONE - D)) && (t1 < (ONE + D))) {
			cnt[i++] = 1;
		}
		else{
			if ((t1 > (OT - D)) && (t1 < (OT + D))) {
				cnt[i++] = 2;
			}
			else{
				if ((t1 > (TWO - D)) && (t1 < (TWO + D))) {
					cnt[i++] = 3;
				}
				else{
					return 0;
				}
			}
		}

	} while (1);
	*n = i;
	return 1;
}


uint16_t ConvertStream_A(uint8_t *cnt, uint8_t n, uint8_t *data, uint8_t *nbuf, uint8_t *lastbit) {

	uint8_t prev = 0, i;
	uint8_t ibyte = 0, ibit = 0, par = 0;

	*nbuf = 20;
	for (i = 0; i < *nbuf; i++) {
		data[i] = 0;
		data[i + BUFFER_PAR] = 0;
	}
	par = 0;

	if ( cnt[0] == 3 ) { // Проверка стартового бита
		return 0;
	} else if (cnt[0] == 2) {
		insbit(&data[0], &ibit, &ibyte, &par, 1);
		prev = 1;
	}

	for (i = 1; i < n; i++) {

		if (cnt[i] == 1) {
			insbit(&data[ibyte], &ibit, &ibyte, &par, prev);
			*lastbit = prev;
		} else if (cnt[i] == 2) {
			insbit(&data[ibyte], &ibit, &ibyte, &par, 0);
			*lastbit = 0;
			if (prev == 0) {
				insbit(&data[ibyte], &ibit, &ibyte, &par, 1);
				prev = 1;
				*lastbit = 1;
			} else {
				prev = 0;
			}
		} else if (cnt[i] == 3) {
			if (prev == 0)
				return 0;
			insbit(&data[ibyte], &ibit, &ibyte, &par, 0);
			insbit(&data[ibyte], &ibit, &ibyte, &par, 1);
			prev = 1;
			*lastbit = 1;
		}
		if (ibyte >= *nbuf)
			return 0;
	}

	if (ibit == 0) {
		*nbuf = ibyte;
		return ibyte * 8;
	}
	if ((ibit == 7) && (ibyte == 0)) {
		*nbuf = ++ibyte;
		return 7;

	}

	return 0;
}

