#ifndef __MACROS_H
#define __MACROS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
  
#define WAIT_EVENT(__HANDLE__, __FLAG__, T)       __HAL_TIM_CLEAR_IT( __HANDLE__, __FLAG__);\
                         while((! __HAL_TIM_GET_FLAG( __HANDLE__, __FLAG__ )) && \
                        		 ((__HANDLE__)->Instance->CNT < T))

#define CNT(__HANDLE__)  (__HANDLE__)->Instance->CNT
#define TIM_Start(__HANDLE__)  (__HANDLE__)->Instance->CR1 |= 0x01;
#define TIM_Stop(__HANDLE__)  (__HANDLE__)->Instance->CR1 &= ~0x01;
                           
//#define B10_SET   		GPIOB->BSRR = GPIO_PIN_10
//#define B10_RESET    	GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16U
#define B10_STROBE 		GPIOB->BSRR = GPIO_PIN_10;\
						GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16U



#endif      //__MACROS_H 
