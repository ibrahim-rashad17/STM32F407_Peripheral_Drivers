/*
 * usart_tx_test.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

uint8_t msg[] = "Ibrahim\r\n";
GPIO_Handle_t USART_GPIO_Handle;
USART_Handle_t USART_Handle;

uint8_t rxbuffer[128];

uint8_t rcvd_data;

int main()
{
	memset(&USART_GPIO_Handle, 0, sizeof(USART_GPIO_Handle));
	memset(&USART_Handle, 0, sizeof(USART_Handle));

	int a,b,c;

	a = 10;
	b = 20;
	c = a+b;
	//TX Pin - PA2
	USART_GPIO_Handle.pGPIOx = GPIOA;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	gpio_init(&USART_GPIO_Handle);
	//RX Pin - PA3
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	USART_GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_init(&USART_GPIO_Handle);

	USART_Handle.pUSARTx = USART2;
	USART_Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART_Handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	USART_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Handle.USART_Config.USART_Parity = USART_PARITY_DISABLE;


	USART_Init(&USART_Handle);

	//Enable the interrupt
	USART_IRQConfig(USART2_IT, ENABLE);

	//Enable IT reception
	USART_ReceiveDataIT(&USART_Handle, &rcvd_data, NO_SPECIFIED_LENGTH);

	USART_SendData(&USART_Handle, msg, strlen((char*)msg));
	printf("Hello from STM32\r\n");

	printf("Sum of a+b : %d + %d = %d\r\n",a,b,c);

	while(1);

	return 0;
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&USART_Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t Event)
{
	if(Event == USART_EV_RX_CMPLT)
	{
		printf("%c\r\n", (char)rcvd_data);
	}
}
