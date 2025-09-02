/*
 * i2c_slv_txrx_testing.c
 *
 *  Created on: Jul 8, 2025
 *      Author: Ibrahim
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

#define I2C_SELF_ADDR	0x61

uint8_t tx_data[] = "Reply from STM32 slave\r\n";
I2C_Handle_t I2C;
GPIO_Handle_t I2C_Gpio;

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
	I2C1_Gpio_Init();

	I2C1_Init();

	I2C_IRQConfig(I2C1_EV, ENABLE);
	I2C_IRQConfig(I2C1_ER, ENABLE);

	while(1);

	return 0;
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv)
{
	static uint8_t command_no = 0, cnt = 0;

	if(AppEv == I2C_EV_DATA_RCV)
	{
		command_no = I2C_SlaveReceiveData(pI2C_Handle->pI2Cx);
	}
	else if(AppEv == I2C_EV_DATA_REQ)
	{
		if(command_no == 0x51)
			I2C_SlaveSendData(pI2C_Handle->pI2Cx, strlen((char*)tx_data));
		else if(command_no == 0x52)
			I2C_SlaveSendData(pI2C_Handle->pI2Cx, tx_data[cnt++]);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		//Master has sent NACK and doesnt want more data
		cnt = 0;
		command_no = 0xff;
	}
	else if(AppEv == I2C_EV_STOP_DETECTED)
	{
		//Slave has detected STOP condition (only in slave mode this occurs)
		//If needed u can write some print statements or anything else based on ur application
	}
}
