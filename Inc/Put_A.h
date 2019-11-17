

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PUT_A_H
#define __PUT_A_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"

uint8_t PutData_A(uint8_t *buf, uint8_t n, uint8_t lastbit);
uint8_t PutStream_A(uint16_t *stream, uint16_t ns, uint8_t lastbit);
uint8_t SendBits_A(uint8_t data, uint8_t ndata, uint8_t lastbit);

#ifdef __cplusplus
}
#endif

#endif /* __PUT_A_H */


