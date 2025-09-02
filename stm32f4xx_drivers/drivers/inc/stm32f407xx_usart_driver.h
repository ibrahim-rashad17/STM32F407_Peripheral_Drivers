/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jul 24, 2025
 *      Author: Ibrahim
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

#define NO_SPECIFIED_LENGTH		0xFFFFU

/* USART Event messages */
#define USART_EV_RX_CMPLT		0
#define USART_EV_TX_CMPLT		1

/* @Word_Length
 *  USART Frame Word Length*/
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_Baud
 *Possible options for USART_Mode
 */
#define USART_MODE_TX		0
#define USART_MODE_RX		1
#define USART_MODE_TXRX		2

/* USART Communication state */
#define USART_READY			0
#define USART_BSY_IN_TX		1
#define USART_BSY_IN_RX		2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_RTS    	1
#define USART_HW_FLOW_CTRL_CTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART flags
 */
#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

typedef struct
{
	uint8_t  USART_Mode;	 				/* !< Refer @USART_Mode > 			*/
	uint8_t  USART_Parity;					/* !< Refer @USART_ParityControl > 	*/
	uint8_t  USART_HWFlowControl;			/* !< Refer @USART_HWFlowControl > 	*/
	uint8_t  USART_WordLength;    			/* !< Refer @Word_Length > 			*/
	uint8_t  USART_StopBits;				/* !< Refer @USART_NoOfStopBits > 	*/
	uint32_t USART_Baud;					/* !< Baud rate @USART_Baud > 		*/
}USART_Config_t;

typedef struct
{
	USART_Config_t	USART_Config;	/* !< Configuration structure for user > */
	USART_RegDef_t  *pUSARTx;		/* !< Holds the base address of USART peripheral > */

	uint32_t		RxLen;
	uint32_t		TxLen;

	uint8_t			*TxBuf;
	uint8_t			*RxBuf;

	uint8_t 		TxBusyState;
	uint8_t			RxBusyState;
}USART_Handle_t;

/* USART Initialization */
void USART_Init(USART_Handle_t *USART_Handle);
void USART_DeInit(USART_Handle_t *USART_Handle);

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint16_t Len);

/* USART Clock and baud rate control */
void USART_Peri_Clk_Control(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/* Interrupt config APIs */
void USART_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di);
void USART_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority);

/* Interrupt Handling API */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t Event);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
