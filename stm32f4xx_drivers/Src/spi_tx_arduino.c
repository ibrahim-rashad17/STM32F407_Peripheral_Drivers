/*
 * spi_tx_arduino.c
 *
 *  Created on: Sep 20, 2025
 *      Author: Ibrahim
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include <string.h>
#include <stdlib.h>

#define BTN_PRESSED		1

GPIO_Handle_t spi_gpio;
SPI_Handle_t spi2;
GPIO_Handle_t GPIOBtn;

char *TxData = "Hello World\r\n";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

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

void Btn_Init()
{
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	gpio_init(&GPIOBtn);

	GPIO_IRQConfig(EXTI0, ENABLE);
}

int main()
{
	//Initialize SPI GPIOs
	SPI_IO_Init();

	//Enable Button Interrupt
	Btn_Init();

	spi2.pSPIx = SPI2;
	spi2.SPI_Config.BusConfig = SPI_BUS_CNFG_FULL_DUPLEX;
	spi2.SPI_Config.Clk_Speed = SPI_SCLK_SPEED_DIV32;
	spi2.SPI_Config.DeviceMode = SPI_DEV_MODE_MASTER;
	spi2.SPI_Config.DFF = SPI_DFF_8BIT;
	spi2.SPI_Config.SSM = SPI_SSM_DISABLE;
	spi2.SPI_Config.CPHA = SPI_CPHA_LOW;
	spi2.SPI_Config.CPOL = SPI_CPOL_LOW;

	SPI_Init(&spi2);
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		if(gpio_read_inp_pin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();

			SPI_PeripheralControl(SPI2, ENABLE);

			uint8_t datalen = strlen(TxData);
			SPI_TransmitData(&spi2, &datalen, 1);

			SPI_TransmitData(&spi2, (uint8_t*)TxData, strlen(TxData));

			while( (SPI2->SR >> SPI_SR_BSY) & 0x01 );

			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}

	return 0;
}
