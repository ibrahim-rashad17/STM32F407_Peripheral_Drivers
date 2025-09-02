/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 23-May-2024
 *      Author: Ibrahim
 */
#include "stm32f407xx_gpio_driver.h"

void GPIO_PeriClk_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

void gpio_init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp;
	uint8_t mode;

	//Enable peripheral clock
	GPIO_PeriClk_Control(pGPIO_Handle->pGPIOx, ENABLE);

	//1. Configuring mode
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode;
	mode=temp;

	//Clear the MODER bits (Bits cleared = input mode)
	pGPIO_Handle->pGPIOx->MODER &= ~(3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));

	if(mode <= GPIO_MODE_ANALOG)
	{
		pGPIO_Handle->pGPIOx->MODER |= (temp << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		if(mode == GPIO_MODE_OUT)
		{
			//Configure output type and speed (First doing speed)
			temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed;
			pGPIO_Handle->pGPIOx->OSPEEDR &= ~(3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIO_Handle->pGPIOx->OSPEEDR |= (temp << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));

			//Configuring op type
			temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType;
			pGPIO_Handle->pGPIOx->OTYPER &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIO_Handle->pGPIOx->OTYPER |= (temp << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configuring PUPD
		temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl;
		pGPIO_Handle->pGPIOx->PUPDR &= ~(3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->PUPDR |= (temp << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));

		if(mode == GPIO_MODE_ALTFN)
		{
			//configure the alt function registers.
			uint8_t temp1, temp2;

			temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber  % 8;
			pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
			pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
		}
	}
	else	//Interrupt mode (MODER is already configured as input)
	{
		//Enable SYSCFG first
		SYSCFG_PCLK_EN();

		if(mode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(mode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(mode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Do port selection in SYSCFG_EXTI reg
		uint8_t ofst,temp1,portcode;
		ofst = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4) ;
		temp1 = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4) ;

		portcode = GPIO_BASEADDR_TO_CODE(pGPIO_Handle->pGPIOx);

		SYSCFG->EXTICR[ofst] &= ~(0xF << (4*temp1));
		SYSCFG->EXTICR[ofst] |= (portcode << (4*temp1));

		//3. Enable the IRQ by unmasking it in EXTI reg
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}

}

uint16_t gpio_read_inp_port(GPIO_RegDef_t *pGPIOx)
{
	uint16_t port_val = (pGPIOx->IDR) & 0xFFFF;

	return port_val;
}

uint8_t gpio_read_inp_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint16_t pin_val = ((pGPIOx->IDR & 0xFFFF) >> PinNum) & 0x01;

	return pin_val;
}

void gpio_write_to_op_port(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (value & 0xFFFF);
}

void gpio_write_to_op_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value)
{
	value &= 0x01;
	pGPIOx->ODR &= ~(1 << PinNum);
	pGPIOx->ODR |= (value << PinNum);
}

void gpio_toggle_op_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= (1 << PinNum);
}

void GPIO_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di)
{
	uint8_t temp;

	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER0) = (1 << temp);
		}
		else if(IRQNumber < 64)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER1) = (1 << temp);
		}
		else if(IRQNumber < 96)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER2) = (1 << temp);
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER0) = (1 << temp);
		}
		else if(IRQNumber < 64)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER1) = (1 << temp);
		}
		else if(IRQNumber < 96)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER2) = (1 << temp);
		}
	}
}

void GPIO_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t *pPrioRegAddr = (uint8_t*)NVIC_PR_BASE_ADDR + (IRQNumber & 0xFF);

	IRQPriority  = IRQPriority << NO_PR_BITS_IMPLEMENTED;	//4 priority bits implemented

	*pPrioRegAddr = IRQPriority;
}

void GPIO_IRQHandling(uint8_t PinNum)
{
	EXTI->PR |= (1 << PinNum);
}
