/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jul 25, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_Prescalar_Div_Val[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescalar_Div_Val[4] = {2, 4, 8, 16};

uint32_t RCC_GetPll_Output()
{
	return 0;
}
uint32_t RCC_GetPCLK2Value()
{
	// TODO
	uint8_t temp = (RCC->CFGR >> 2) & 0x03;		//System clk switch status
	uint16_t ahbp;
	uint8_t apbp2;

	uint32_t sysclk, PClk2;

	if(temp==0)
		sysclk = 16000000;
	else if(temp==1)
		sysclk = 8000000;
	else if(temp==2)
		sysclk = RCC_GetPll_Output();

	temp = (RCC->CFGR >> 4) & 0x0F;			//HPRE bits (AHB prescalar bits)
	if(temp < 8)
		ahbp = 1;
	else
		ahbp = AHB_Prescalar_Div_Val[temp-8];

	temp = (RCC->CFGR >> 13) & 0x07;			//PPRE2 bits (APB2 prescalar bits)
	if(temp < 4)
		apbp2 = 1;
	else
		apbp2 = APB_Prescalar_Div_Val[temp-4];

	PClk2 = (sysclk / ahbp) / apbp2;

	return PClk2;
}
uint32_t RCC_GetPCLK1Value()
{
	uint8_t temp = (RCC->CFGR >> 2) & 0x03;		//System clk switch status
	uint16_t ahbp;
	uint8_t apbp1;

	uint32_t sysclk, PClk1;

	if(temp==0)
		sysclk = 16000000;
	else if(temp==1)
		sysclk = 8000000;
	else if(temp==2)
		sysclk = RCC_GetPll_Output();

	temp = (RCC->CFGR >> 4) & 0x0F;			//HPRE bits (AHB prescalar bits)
	if(temp < 8)
		ahbp = 1;
	else
		ahbp = AHB_Prescalar_Div_Val[temp-8];

	temp = (RCC->CFGR >> 10) & 0x07;			//PPRE1 bits (APB1 prescalar bits)
	if(temp < 4)
		apbp1 = 1;
	else
		apbp1 = APB_Prescalar_Div_Val[temp-4];

	PClk1 = (sysclk / ahbp) / apbp1;

	return PClk1;
}
