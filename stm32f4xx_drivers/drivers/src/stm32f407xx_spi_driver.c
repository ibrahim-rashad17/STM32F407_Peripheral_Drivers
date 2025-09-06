/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - API to enable or disable clock for SPIx peripherals
 *
 * @param[in]         - pointer to base address of the SPI peripheral
 * @param[in]         - ENABLE/DISABLE macros
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - API to initialize SPIx peripherals
 *
 * @param[in]         - pointer to SPI Handle structure
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	uint32_t tempreg = 0;

	//Turn on the peripheral clock
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	//Enable the SPI peripheral
	tempreg |= (1 << SPI_CR1_SPE);

	//Configure the SPI device mode
	tempreg |= ((pSPI_Handle->SPI_Config.DeviceMode & 0x01) << SPI_CR1_MSTR);

	//Configure bus mode (If full duplex then no need to change anything)
	if(pSPI_Handle->SPI_Config.BusConfig == SPI_BUS_CNFG_HALF_DUPLEX)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CNFG_SIMPLEX_RX_ONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}
	else	//Full duplex
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	//Configure CPOL and CPHA
	tempreg |= ((pSPI_Handle->SPI_Config.CPOL & 0x01) << SPI_CR1_CPOL);
	tempreg |= ((pSPI_Handle->SPI_Config.CPHA & 0x01) << SPI_CR1_CPHA);

	//Configure the data frame format (8 bits by default)
	tempreg |= ((pSPI_Handle->SPI_Config.DFF & 0x01) << SPI_CR1_DFF);

	//Clock speed
	tempreg |= ((pSPI_Handle->SPI_Config.Clk_Speed & 0x07) << SPI_CR1_BR);

	//Software slave management
	if(pSPI_Handle->SPI_Config.SSM == SPI_SSM_ENABLE)
	{
		tempreg |= (SPI_SSM_ENABLE << SPI_CR1_SSM);
		tempreg |= ((pSPI_Handle->SPI_Config.SSI & 0x1) << SPI_CR1_SSI);
	}

	//Write the changes to peripheral control register
	pSPI_Handle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Set the selected SPIx peripheral to default
 *
 * @param[in]         - pointer to SPI Handle structure
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -

 */
void SPI_DeInit(SPI_Handle_t *pSPI_Handle)
{
	if(pSPI_Handle->pSPIx == SPI1)
	{
		RCC->APB2RSTR |= (1 << 12);
		RCC->APB2RSTR &= ~(1 << 12);
	}
	else if(pSPI_Handle->pSPIx == SPI2)
	{
		RCC->APB1RSTR |= (1 << 14);
		RCC->APB1RSTR &= ~(1 << 14);
	}
	else if(pSPI_Handle->pSPIx == SPI3)
	{
		RCC->APB1RSTR |= (1 << 15);
		RCC->APB1RSTR &= ~(1 << 15);
	}
}
