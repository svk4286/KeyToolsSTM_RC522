
#ifndef __GET_A_H
#define __GET_A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
  
uint8_t getStream_A(uint8_t *cnt, uint8_t *n, uint32_t timeout);
uint16_t ConvertStream_A(uint8_t *cnt, uint8_t n, uint8_t *data, uint8_t *nbuf, uint8_t *lastbit);
uint8_t insbit(uint8_t *buf, uint8_t *ibit, uint8_t *ibyte, uint8_t *par, uint8_t b);

#endif      //__GET_A_H
