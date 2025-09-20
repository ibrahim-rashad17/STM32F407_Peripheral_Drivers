/*
 * spi_tx_test.c
 *
 *  Created on: Sep 19, 2025
 *      Author: Ibrahim
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include <string.h>
#include <stdlib.h>

GPIO_Handle_t spi_gpio;
SPI_Handle_t spi2;

char *TxData = "Hello World\r\n";

void SPI_IO_Init()
{
	spi_gpio.pGPIOx = GPIOB;
	spi_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	spi_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;	//PB15 - SPI2 MOSI
	spi_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpio_init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;	//PB14 - SPI2 MISO
	gpio_init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;	//PB13 - SPI2 SCLK
	gpio_init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;	//PB12 - SPI2 NSS
	spi_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	gpio_init(&spi_gpio);
}

int main()
{
	//Initialize SPI GPIOs
	SPI_IO_Init();

	spi2.pSPIx = SPI2;
	spi2.SPI_Config.BusConfig = SPI_BUS_CNFG_FULL_DUPLEX;
	spi2.SPI_Config.Clk_Speed = SPI_SCLK_SPEED_DIV256;
	spi2.SPI_Config.DeviceMode = SPI_DEV_MODE_MASTER;
	spi2.SPI_Config.DFF = SPI_DFF_8BIT;
	spi2.SPI_Config.SSM = SPI_SSM_ENABLE;
	spi2.SPI_Config.SSI = 1;
	spi2.SPI_Config.CPHA = 0;
	spi2.SPI_Config.CPOL = 0;

	SPI_Init(&spi2);

	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_TransmitData(&spi2, (uint8_t*)TxData, strlen(TxData));
	while(! ((SPI2->SR >> SPI_SR_TXE) & 0x01) );
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
