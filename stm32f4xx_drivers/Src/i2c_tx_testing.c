/*
 * i2c_tx_testing.c
 *
 *  Created on: Jun 2, 2025
 *      Author: Ibrahim
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32f407xx.h"

#define I2C_SELF_ADDR	0x61

uint8_t data[] = "Testing I2C-Tx driver\r\n";
I2C_Handle_t I2C;
GPIO_Handle_t I2C_Gpio;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void I2C1_Gpio_Init()
{
	memset(&I2C_Gpio,0,sizeof(I2C_Gpio));

	I2C_Gpio.pGPIOx = GPIOB;
	I2C_Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_Gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_Gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PULLUP;
	I2C_Gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	I2C_Gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	//SDA - I2C1
	I2C_Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_init(&I2C_Gpio);

	//SCL - I2C1
	I2C_Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpio_init(&I2C_Gpio);

}

void Push_Btn_Init()
{
	GPIO_Handle_t GPIOBtn;

	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	gpio_init(&GPIOBtn);
}

void I2C1_Init()
{
	I2C.pI2Cx = I2C1;
	I2C.i2c_config.DeviceAddress = I2C_SELF_ADDR;
	I2C.i2c_config.I2C_Ack = I2C_ACK_ENABLE;
	I2C.i2c_config.I2C_SCLK_Speed = I2C_SPEED_SM_100K;

	i2c_init(&I2C);
}

int main(void)
{
	Push_Btn_Init();

	I2C1_Gpio_Init();

	I2C1_Init();

	while(1)
	{
		while(! gpio_read_inp_pin(GPIOA, GPIO_PIN_NO_0) );

		delay();

		I2C_MasterSendData(&I2C, data, 0x61, strlen((char*)data),DISABLE);

	}

	return 0;
}

