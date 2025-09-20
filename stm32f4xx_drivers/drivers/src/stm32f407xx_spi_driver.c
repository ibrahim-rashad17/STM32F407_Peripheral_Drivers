/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx_spi_driver.h"

static uint8_t Get_SPI_FlagStatus(SPI_RegDef_t *pSPIx, uint8_t FLAG)
{
	if( (pSPIx->SR) & (1 << FLAG) )
	{
		return HIGH;
	}
	else
	{
		return LOW;
	}
}

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

	//Configure the SPI device mode
	tempreg |= ((pSPI_Handle->SPI_Config.DeviceMode & 0x01) << SPI_CR1_MSTR);

	//Configure bus mode (If full duplex then no need to change anything)
	if(pSPI_Handle->SPI_Config.BusConfig == SPI_BUS_CNFG_HALF_DUPLEX)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPI_Handle->SPI_Config.BusConfig == SPI_BUS_CNFG_SIMPLEX_RX_ONLY)
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

/*********************************************************************
 * @fn      		  - SPI_TransmitData
 *
 * @brief             - Transmit Len no of bytes
 *
 * @param[in]         - pointer to SPI Handle structure
 * @param[in]         - pointer to Tx Buffer
 * @param[in]         - Length of data to be transmitted
 *
 * @return            - none
 *
 * @Note              -	This is a blocking API

 */
void SPI_TransmitData(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBfr, uint32_t len)
{
	for(uint32_t i=0;i<len;i++)
	{
		while(!(Get_SPI_FlagStatus(pSPI_Handle->pSPIx, SPI_FLAG_TXE)));

		if(pSPI_Handle->SPI_Config.DFF == SPI_DFF_8BIT)
		{
			pSPI_Handle->pSPIx->DR = *pTxBfr;
			pTxBfr++;
		}
		else
		{
			//16 bit format
			pSPI_Handle->pSPIx->DR = *(uint16_t*)pTxBfr;
			(uint16_t*)pTxBfr++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receive Len no of bytes
 *
 * @param[in]         - pointer to SPI Handle structure
 * @param[in]         - pointer to Rx Buffer
 * @param[in]         - Length of data to be received
 *
 * @return            - none
 *
 * @Note              -	This is a blocking API

 */
void SPI_ReceiveData(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBfr, uint32_t len)
{
	for(uint32_t i=0;i<len;i++)
	{
		while(!(Get_SPI_FlagStatus(pSPI_Handle->pSPIx, SPI_FLAG_RXNE)));

		if(pSPI_Handle->SPI_Config.DFF == SPI_DFF_8BIT)
		{
			*pRxBfr = pSPI_Handle->pSPIx->DR & 0xFF;
			pRxBfr++;
		}
		else
		{
			//16 bit format
			*(uint16_t*)pRxBfr = pSPI_Handle->pSPIx->DR & 0xFFFF;
			(uint16_t*)pRxBfr++;
		}
	}
}
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -	API to ENABLE/DISABLE the SPIx peripheral
 *
 * @param[in]         -	Pointer to SPIx peripheral
 * @param[in]         -	ENABLE or DISABLE macro
 *
 * @return            -	none
 *
 * @Note              -	Always call this API to enable SPI before communication and again call this
 * 						API to disable SPI after communication
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -	SPI Slave Select Internal Configuration
 * 						This API configures the SPI SSI bit.
 * 						Value of SSI bit is forced onto the NSS pin
 *
 * @param[in]         -	Pointer to SPIx peripheral
 * @param[in]         -	ENABLE or DISABLE macro
 *
 * @return            -	none
 *
 * @Note              -	SSI bit is effective only if SSM is enabled

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -	Software slave output enable config
 *
 * @param[in]         -	Pointer to SPIx peripheral
 * @param[in]         -	ENABLE or DISABLE macro
 *
 * @return            -	none
 *
 * @Note              -	When SSM is disabled, then if SSOE bit is set, the SPE bit value
 * 						determines the NSS pin value.
 * 						SPE = 1 --> NSS = 0
 * 						SPE = 0 --> NSS = 1
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
