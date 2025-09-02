/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Ibrahim
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPCLK2Value();
uint32_t RCC_GetPll_Output();

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
