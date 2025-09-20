/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Sep 4, 2025
 *      Author: Ibrahim
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * SPI Status Flags
 */
#define SPI_FLAG_RXNE			0
#define SPI_FLAG_TXE			1
#define SPI_FLAG_CRCERR			4
#define SPI_FLAG_MODF			5
#define SPI_FLAG_OVR			6
#define SPI_FLAG_BSY			7


/*
 * @SPI_DeviceMode
 */
#define SPI_DEV_MODE_MASTER				1
#define SPI_DEV_MODE_SLAVE				0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CNFG_FULL_DUPLEX		0
#define SPI_BUS_CNFG_HALF_DUPLEX		1
#define SPI_BUS_CNFG_SIMPLEX_RX_ONLY	2

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT		0
#define SPI_DFF_16BIT		1

/*
 * @SPI_SSM
 */
#define SPI_SSM_ENABLE		1
#define SPI_SSM_DISABLE		0

/*
 * @SPI_CLK_SPEED
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

typedef struct
{
	uint8_t 	DeviceMode;				/* Refer @SPI_DeviceMode */
	uint8_t		BusConfig;				/* Refer @SPI_BusConfig */
	uint8_t 	DFF;					/* Data Frame Format, refer @SPI_DFF */
	uint8_t 	CPOL;					/* Clock Polarity , refer @SPI_CPOL */
	uint8_t		CPHA;					/* Clock Phase, refer @SPI_CPHA */
	uint8_t		SSM;					/* Software Slave Management, refer @SPI_SSM */
	uint8_t 	SSI;					/* Internal slave select (only if SSM enabled) */
	uint8_t 	Clk_Speed;				/* SPI Clock speed (depends on PClk), refer @SPI_CLK_SPEED */
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t	*pSPIx;			/* Holds the base address of SPI peripheral */
	SPI_Config_t	SPI_Config;		/* SPI User Configuration structure */
}SPI_Handle_t;


/* SPI Peripheral Initialization/De-initialization APIs */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_Handle_t *pSPI_Handle);

/* SPI data transfer blocking APIs */
void SPI_TransmitData(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBfr, uint32_t len);
void SPI_ReceiveData(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBfr, uint32_t len);

/* SPI Clock Control API */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/* SPI Peripheral Control API */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
