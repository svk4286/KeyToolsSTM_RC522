#include "Put_A.h"

#define __OC4_FORCE_ACTIVE(__HANDLE__) \
                        do { uint16_t tmp; \
                          tmp = (__HANDLE__)->Instance->CCMR2; \
                          tmp &= ~0x7000; tmp  |= 0x5000; \
                          (__HANDLE__)->Instance->CCMR2 = tmp; \
                        } while(0U)

#define __OC4_FORCE_INACTIVE(__HANDLE__) \
                        do { uint16_t tmp; \
                          tmp = (__HANDLE__)->Instance->CCMR2; \
                          tmp &= ~0x7000; tmp  |= 0x4000; \
                          (__HANDLE__)->Instance->CCMR2 = tmp; \
                        } while(0U)

#define __OC4_TOGGLE(__HANDLE__) \
                        do { uint16_t tmp; \
                          tmp = (__HANDLE__)->Instance->CCMR2; \
                          tmp &= ~0x7000; tmp  |= 0x3000; \
                          (__HANDLE__)->Instance->CCMR2 = tmp; \
                        } while(0U)

#define __OC4_INACTIVE(__HANDLE__) \
                        do { uint16_t tmp; \
                          tmp = (__HANDLE__)->Instance->CCMR2; \
                          tmp &= ~0x7000; tmp  |= 0x2000; \
                          (__HANDLE__)->Instance->CCMR2 = tmp; \
                        } while(0U)

#define __OC4_ACTIVE(__HANDLE__) \
                        do { uint16_t tmp; \
                          tmp = (__HANDLE__)->Instance->CCMR2; \
                          tmp &= ~0x7000; tmp  |= 0x1000; \
                          (__HANDLE__)->Instance->CCMR2 = tmp; \
                        } while(0U)

#define __OC4_FROZEN(__HANDLE__)  (__HANDLE__)->Instance->CCMR2 &= ~0x7000

#define PUTBIT		if(!putbit())	return 0


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern __IO uint32_t uwTick;
static 	uint32_t tickstart;

uint16_t  FWT_0 = 1172 - 8;
uint16_t  FWT_1 = 1236 - 8;

#define ETU  128
#define HALF_ETU 64

#define Tout 4

#define Takt 763
#define Takt2 423
#define Sub 83

#define WAIT_LOW   while( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) )
#define WAIT_HIGH  while( ! HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) )
#define WAIT_EVENT       __HAL_TIM_CLEAR_IT( &htim1, TIM_FLAG_CC4);\
        while(! __HAL_TIM_GET_FLAG( &htim1, TIM_FLAG_CC4 ) && ((uwTick - tickstart) < Tout));\
        if(((uwTick - tickstart) >= Tout))\
			return 0;

#define ADD_T( T ) (&htim1)->Instance->CCR4 += T
#define SET_T(T)    (&htim1)->Instance->CCR4 = T


static	uint16_t fwt;
static	uint8_t prev, mask, bit, par = 0;


uint8_t putbit(){

	if( bit != prev ){
		ADD_T( ETU );
		WAIT_EVENT;
		prev = bit;
	}
	else{
		ADD_T( HALF_ETU);
		WAIT_EVENT;
		ADD_T( HALF_ETU);
		WAIT_EVENT;
	}
	return 1;
}


uint8_t PutData_A(uint8_t *buf, uint8_t nbuf, uint8_t lastbit){

	tickstart = uwTick;
	prev = 1;
	par = 0;
	if(lastbit){
		fwt = FWT_1;
	}
	else{
		fwt = FWT_0;
	}

         __OC4_FORCE_INACTIVE(&htim1);
         __OC4_TOGGLE(&htim1);
         (&htim1)->Instance->CCER |= 0x1000;

        htim2.Instance->CNT = 0;

        if(htim1.Instance->CNT < fwt){
          SET_T(fwt);
          WAIT_EVENT;
        }
        else
        {
          htim1.Instance->CNT = 0;
          SET_T(0);
          ADD_T( HALF_ETU );
          WAIT_EVENT;
        }

        ADD_T( HALF_ETU );
        WAIT_EVENT;


	for(int i = 0; i < nbuf; i++){
		mask = 0x01;
		for(int j = 0; j < 8; j++){
			bit = (buf[i] & mask) ? 1 : 0;
			par ^= bit;
			PUTBIT;
			mask <<= 1;
		}
		bit  = par ^ 1;
		PUTBIT;
		par = 0;
	}
        __OC4_INACTIVE(&htim1);
        ADD_T( HALF_ETU );
        WAIT_EVENT;
        __OC4_FROZEN(&htim1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0xffff);
        return 1;

}


uint8_t PutStream_A(uint16_t *stream, uint16_t ns, uint8_t lastbit){

	uint16_t i,j;
	tickstart = uwTick;
	prev = 1;
	par = 0;
//	if(lastbit){
//		fwt = FWT_1;
//	}
//	else{
//		fwt = FWT_0;
//	}

	__OC4_FORCE_INACTIVE(&htim1);
	__OC4_TOGGLE(&htim1);
	(&htim1)->Instance->CCER |= 0x1000;

	htim2.Instance->CNT = 0;

//	if (htim1.Instance->CNT < fwt) {
//		SET_T(fwt);
//		WAIT_EVENT;
//	}
//	else
//	{
		htim1.Instance->CNT = 0;
		SET_T(0);
		ADD_T( HALF_ETU );
		WAIT_EVENT;
//	}

	for( i = 0; (stream[ i ] < (Sub+5)) && (stream[ i ] > (Sub - 5)) && (i < ns); i++);
	if( i == ns ){
		return 0;
	}
	ADD_T( HALF_ETU);
	WAIT_EVENT;

	while( i < ns){
		if((stream[i] > (Takt - 5)) && (stream[i] < (Takt + 5))){
			ADD_T(ETU );
			WAIT_EVENT;
			i++;
		}
		else{
			if((stream[i] > (Takt2 - 5)) || (stream[i] < (Takt2 + 5))){
				ADD_T( HALF_ETU);
				WAIT_EVENT;
				i++;
			}
		}
		for (j = 0;(stream[i] < (Sub + 5)) && (stream[i] > (Sub - 5)) && (i < ns);i++, j++);
		if ((j == 3) || (j == 4)){
			ADD_T(HALF_ETU);
			WAIT_EVENT;
		}
		else {
			if((j == 7) || (j == 8)) {
				ADD_T( ETU );
				WAIT_EVENT;
			}
		}
	}
	__OC4_FORCE_INACTIVE(&htim1);
    __OC4_FROZEN(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0xffff);

	return 1;
}


uint8_t SendBits_A(uint8_t data, uint8_t ndata, uint8_t lastbit){

	tickstart = uwTick;
	prev = 1;
	par = 0;
	if(lastbit){
		fwt = FWT_1;
	}
	else{
		fwt = FWT_0;
	}

         __OC4_FORCE_INACTIVE(&htim1);
         __OC4_TOGGLE(&htim1);
         (&htim1)->Instance->CCER |= 0x1000;

        htim2.Instance->CNT = 0;

        if(htim1.Instance->CNT < fwt){
          SET_T(fwt);
          WAIT_EVENT;
        }
        else
        {
          htim1.Instance->CNT = 0;
          SET_T(0);
          ADD_T( HALF_ETU );
          WAIT_EVENT;
        }

        ADD_T( HALF_ETU );
        WAIT_EVENT;

		mask = 0x01;
		for(int j = 0; j < ndata; j++){
			bit = (data & mask) ? 1 : 0;
			par ^= bit;
			PUTBIT;
			mask <<= 1;
		}
        __OC4_INACTIVE(&htim1);
        ADD_T( HALF_ETU );
        WAIT_EVENT;
        __OC4_FROZEN(&htim1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0xffff);
        return 1;
}




