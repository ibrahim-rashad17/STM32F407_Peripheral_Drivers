/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx_usart_driver.h"

static void USART_HandleRXNEInterrupt(USART_Handle_t *pUSARTHandle);
static void USART_HandleTXEInterrupt(USART_Handle_t *pUSARTHandle);
static void USART_HandleTCInterrupt(USART_Handle_t *pUSARTHandle);
static void USART_CloseReception(USART_Handle_t *pUSARTHandle);

void USART_Peri_Clk_Control(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}

void USART_Init(USART_Handle_t *pUSART_Handle)
{
	uint32_t tempreg = 0;

	/* Enable Peripheral clock */
	USART_Peri_Clk_Control(pUSART_Handle->pUSARTx, ENABLE);

	/******************************** Configuration of CR1******************************************/

	//Enable the USART
	tempreg |= (1 << USART_CR1_UE);

	//Configuring mode
	if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}

	// Configure word length
	tempreg |= pUSART_Handle->USART_Config.USART_WordLength << USART_CR1_M ;

	//Parity control
	if(pUSART_Handle->USART_Config.USART_Parity == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);
	}
	else if(pUSART_Handle->USART_Config.USART_Parity == USART_PARITY_EN_ODD)
	{
		tempreg |= ( (1 << USART_CR1_PCE) | (1 << USART_CR1_PS) );
	}

	pUSART_Handle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/
	tempreg = 0;

	//Configure the no of stop bits
	tempreg |= (pUSART_Handle->USART_Config.USART_StopBits << USART_CR2_STOP);

	pUSART_Handle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/
	tempreg = 0;

	//Configure hardware flow control
	tempreg |= (pUSART_Handle->USART_Config.USART_HWFlowControl << USART_CR3_RTSE);

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSART_Handle->pUSARTx,pUSART_Handle->USART_Config.USART_Baud);
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - API to set the baudrate specified by user
 *
 * @param[in]         - Base address to the USART peripheral
 * @param[in]         - Baud rate
 *
 * @return            - none
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PClkx, USARTdiv;
	uint16_t mantissa, fraction;

	uint8_t over8 = (pUSARTx->CR1 >> USART_CR1_OVER8) & 0x01;

	if((pUSARTx == USART1) || (pUSARTx == USART6))
		PClkx = RCC_GetPCLK2Value();
	else
		PClkx = RCC_GetPCLK1Value();

	if(over8)
		USARTdiv = (25 * PClkx) / (2 * BaudRate);
	else
		USARTdiv = (25 * PClkx) / (4 * BaudRate);

	mantissa = USARTdiv / 100;
	fraction = USARTdiv - (mantissa*100);

	if(over8)
	{
		fraction = (fraction*8) + 50;	//to round off
		fraction = fraction / 100;

		if(fraction > 7)	//Max 3 bits for oversampling by 8 (hence adding carry to mantissa as per RM)
		{
			mantissa += (mantissa + (fraction-7));
		}
	}
	else
	{
		fraction = (fraction*16) + 50;
		fraction = fraction / 100;

		if(fraction > 15)	//Max 4 bits for oversampling by 8
		{
			mantissa += (mantissa + (fraction-15));
		}
	}

	uint16_t USART_BRR = (mantissa << 4) | (fraction & 0xF);

	pUSARTx->BRR = USART_BRR;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         - Handle to USART peripheral structure
 * @param[in]         - Pointer to tx buffer
 * @param[in]         - Length of data to be sent
 *
 * @return            - none
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	for(uint32_t i=0;i<Len;i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_Parity==USART_PARITY_DISABLE)
			{
				//All 9 bits user data
				pdata = (uint16_t*) pTxBuffer;
				pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pUSARTHandle->pUSARTx->DR = *pTxBuffer & 0xFF;
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = *pTxBuffer & 0xFF;
			pTxBuffer++;
		}
	}

	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         - Handle to USART peripheral structure
 * @param[in]         - Pointer to tx buffer
 * @param[in]         - Length of data to be sent
 *
 * @return            - none
 *
 * @Note              - Resolve all the TODOs

 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i=0;i<Len;i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_Parity==USART_PARITY_DISABLE)
			{
				//All 9 bits user data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR) & (uint16_t)0x1FF;
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				*pRxBuffer = pUSARTHandle->pUSARTx->DR & 0xFF;
				pRxBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config.USART_Parity==USART_PARITY_DISABLE)
			{
				*pRxBuffer = pUSARTHandle->pUSARTx->DR & 0xFF;
				pRxBuffer++;
			}
			else
			{
				//Read only 7 bits as 1 bit for parity
				*pRxBuffer = pUSARTHandle->pUSARTx->DR & 0x7F;
				pRxBuffer++;
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -	This is Interrupt based Data Reception, here rx interrupt is enabled and state
 * 						is changed. You need to call IRQHandling API in corresponding IRQ_Handler for
 * 						receiving data in the buffer passed in argument
 *
 * @param[in]         - Handle to USART peripheral structure
 * @param[in]         -	Pointer to RxBuffer
 * @param[in]         - Length of data to be recieved
 *
 * @return            - Busystate of USART
 *
 * @Note              -	Data reception occurs byte by byte
 *
 * 						Incase you want to keep the receive interrupt always enabled,
 * 						pass the macro NO_SPECIFIED_LENGTH in the Len field

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint16_t Len)
{
	uint8_t busystate = pUSARTHandle->RxBusyState;

	if(pUSARTHandle->RxBusyState != USART_BSY_IN_RX)
	{
		pUSARTHandle->RxBusyState = USART_BSY_IN_RX;

		pUSARTHandle->RxBuf = pRxBuffer;
		pUSARTHandle->RxLen = Len;

		/* Enable the interrupt bits */
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return busystate;
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint8_t tcie, rxneie, txeie;
	uint8_t tc, rxne, txe;

	/* Event flags */
	rxne 	= USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE);
	txe 	= USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE);
	tc 		= USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC);

	/* Interrupt control bits */
	tcie	= (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_TCIE) & 0x01 ;
	txeie 	= (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_TXEIE) & 0x01 ;
	rxneie 	= (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_RXNEIE) & 0x01 ;

	/* Interrupt due to data reception */
	if(rxne && rxneie)
	{
		USART_HandleRXNEInterrupt(pUSARTHandle);
	}

	if(txe && txeie)
	{
		USART_HandleTXEInterrupt(pUSARTHandle);
	}

	if(tc && tcie)
	{
		USART_HandleTCInterrupt(pUSARTHandle);
	}
}

static void USART_HandleRXNEInterrupt(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->RxLen > 0)
	{
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_Parity==USART_PARITY_DISABLE)
			{
				//All 9 bits user data
				*(uint16_t*)pUSARTHandle->RxBuf = (pUSARTHandle->pUSARTx->DR) & (uint16_t)0x1FF;

				if(pUSARTHandle->RxLen != NO_SPECIFIED_LENGTH)
				{
					pUSARTHandle->RxBuf += 2;
					pUSARTHandle->RxLen -= 2;
				}
			}
			else
			{
				*pUSARTHandle->RxBuf = pUSARTHandle->pUSARTx->DR & 0xFF;

				if(pUSARTHandle->RxLen != NO_SPECIFIED_LENGTH)
				{
					pUSARTHandle->RxBuf++;
					pUSARTHandle->RxLen--;
				}
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config.USART_Parity==USART_PARITY_DISABLE)
			{
				*pUSARTHandle->RxBuf = pUSARTHandle->pUSARTx->DR & 0xFF;

				if(pUSARTHandle->RxLen != NO_SPECIFIED_LENGTH)
				{
					pUSARTHandle->RxBuf++;
					pUSARTHandle->RxLen--;
				}
			}
			else
			{
				//Read only 7 bits as 1 bit for parity
				*pUSARTHandle->RxBuf = pUSARTHandle->pUSARTx->DR & 0x7F;

				if(pUSARTHandle->RxLen != NO_SPECIFIED_LENGTH)
				{
					pUSARTHandle->RxBuf++;
					pUSARTHandle->RxLen--;
				}
			}
		}
	}

	if(pUSARTHandle->RxLen == NO_SPECIFIED_LENGTH)
	{
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_RX_CMPLT);
	}
	else
	{
		if(!(pUSARTHandle->RxLen))
		{
			USART_CloseReception(pUSARTHandle);
			USART_ApplicationEventCallback(pUSARTHandle, USART_EV_RX_CMPLT);
		}
	}
}

static void USART_HandleTXEInterrupt(USART_Handle_t *pUSARTHandle)
{

}
static void USART_HandleTCInterrupt(USART_Handle_t *pUSARTHandle)
{

}

static void USART_CloseReception(USART_Handle_t *pUSARTHandle)
{
	pUSARTHandle->RxBusyState = USART_READY;
	pUSARTHandle->RxBuf = NULL;
	pUSARTHandle->RxLen = 0;

	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RE);
}
void USART_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di)
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

void USART_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t *pPrioRegAddr = (uint8_t*)NVIC_PR_BASE_ADDR + (IRQNumber & 0xFF);

	IRQPriority  = IRQPriority << NO_PR_BITS_IMPLEMENTED;	//4 priority bits implemented

	*pPrioRegAddr = IRQPriority;
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t Event)
{

}
