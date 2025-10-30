/*
 * spi_cmd_handling.c
 *
 *  Created on: Sep 22, 2025
 *      Author: Ibrahim
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include <string.h>
#include <stdlib.h>

#define BTN_PRESSED		1

#define NACK 0xA5
#define ACK 0xF5

//command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   	0
#define ANALOG_PIN1   	1
#define ANALOG_PIN2   	2
#define ANALOG_PIN3   	3
#define ANALOG_PIN4   	4

#define	LED_PIN			9

uint8_t cmd[5] =
{
		COMMAND_LED_CTRL, COMMAND_SENSOR_READ, COMMAND_LED_READ, \
		COMMAND_PRINT, COMMAND_ID_READ
};

GPIO_Handle_t spi_gpio;
SPI_Handle_t spi2;
GPIO_Handle_t GPIOBtn;
uint8_t cmd_indx = 0;

char *TxData = "Hello World\r\n";

void delay(void)
{
	for(uint32_t i = 0 ; i < 600000 ; i ++);
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

uint8_t SendCmdToArduino(uint8_t cmd_num)
{
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_TransmitData(&spi2, &cmd_num, 1);
	SPI_ReceiveData(&spi2, &dummy_read, 1);

	SPI_TransmitData(&spi2, &dummy_write, 1);
	uint8_t ack;
	SPI_ReceiveData(&spi2, &ack, 1);

	return ack;
}

void SendArgs(uint8_t cmd_num)
{
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;
	uint8_t data;
	switch (cmd_num)
	{
	case COMMAND_LED_CTRL:
	{
		data = LED_PIN;
		SPI_TransmitData(&spi2, &data, 1);
		SPI_ReceiveData(&spi2, &dummy_read, 1);
		data = LED_ON;
		SPI_TransmitData(&spi2, &data, 1);
		SPI_ReceiveData(&spi2, &dummy_read, 1);

		printf("Sent LED control command \r\n");
	}break;
	case COMMAND_SENSOR_READ:
	{
		data = ANALOG_PIN1;
		SPI_TransmitData(&spi2, &data, 1);
		SPI_ReceiveData(&spi2, &dummy_read, 1);

		printf("Sent LED sensor command \r\n");

		SPI_TransmitData(&spi2, &dummy_write, 1);
		SPI_ReceiveData(&spi2, &data, 1);

		printf("Analog pin value is %d\r\n",data);
	}break;
	case COMMAND_LED_READ:
	{
		data = LED_PIN;
		SPI_TransmitData(&spi2, &data, 1);
		SPI_ReceiveData(&spi2, &dummy_read, 1);

		SPI_TransmitData(&spi2, &dummy_write, 1);
		SPI_ReceiveData(&spi2, &data, 1);

		if(data == LED_ON)
		{
			printf("LED is ON\r\n");
		}
		else
		{
			printf("LED is off\r\n");
		}
	}break;
	case COMMAND_PRINT:
	{
		char *TxStr = "Hello from STM32\r\n";
		data = strlen(TxStr);

		//Transmitting the length
		SPI_TransmitData(&spi2, &data, 1);
		SPI_ReceiveData(&spi2, &dummy_read, 1);

		//Transmitting the string
		for(int i=0;i<strlen(TxStr);i++)
		{
			SPI_TransmitData(&spi2, (uint8_t*)&TxStr[i], 1);
			SPI_ReceiveData(&spi2, &dummy_read, 1);
		}

		printf("Sent %s \r\n",TxStr);
	}break;
	case COMMAND_ID_READ:
	{
		printf("Sent Arduino board ID command \r\n");
		char id[11];
		for(int i=0;i<10;i++)
		{
			SPI_TransmitData(&spi2, &dummy_write, 1);
			SPI_ReceiveData(&spi2, (uint8_t*)&id[i], 1);
		}

		id[10] = '\0';
		printf("The Arduino board ID is %s\r\n", id);
	}break;

	default:
		break;
	}
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
			uint8_t rsp = SendCmdToArduino(cmd[cmd_indx]);

			if(rsp == ACK)
			{
				printf("Received ACK\r\n");
				SendArgs(cmd[cmd_indx]);
				cmd_indx++;
				if(cmd_indx>4)
				{
					cmd_indx = 0;
				}
			}
			else
			{
				cmd_indx = 0;
				printf("Received NACK\r\n");
			}
			while( (SPI2->SR >> SPI_SR_BSY) & 0x01 );
//			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}

	return 0;
}

