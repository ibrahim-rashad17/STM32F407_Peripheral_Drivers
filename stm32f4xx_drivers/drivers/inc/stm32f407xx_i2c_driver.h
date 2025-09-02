/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 9, 2025
 *      Author: Ibrahim
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"


//I2C EVENT MESSAGES
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP_DETECTED	2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_RCV			8
#define I2C_EV_DATA_REQ			9

/* I2C ACK CONTROL MACROS */
#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

/* I2C SCLK SPEED */
#define I2C_SPEED_SM_80K		80000
#define I2C_SPEED_SM_100K		100000
#define I2C_SPEED_FM_200K		200000
#define I2C_SPEED_FM_400K		400000

/* I2C Fast Mode Duty Cycle */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


/* I2C Communication states */
#define I2C_READY			0
#define I2C_BSY_IN_RX		1
#define I2C_BSY_IN_TX		2

#define I2C_WRITE_MODE		0
#define I2C_READ_MODE		1

typedef struct
{
	uint8_t DeviceAddress;	 	/* !< Only when device acts as slave mode > */
	uint8_t I2C_Ack;			/* !< Acknowledgment > */
	uint32_t I2C_SCLK_Speed;	/* !< Serial clock speed > */
	uint8_t I2C_FMDutyCycle;    /* !< Duty Cycle in Fast mode > */
}I2C_Config_t;

typedef struct
{
	I2C_Config_t i2c_config;	 /* !< Configuration structure > */
	I2C_RegDef_t *pI2Cx;		 /* !< Holds the base address of the I2C peripheral > */

	uint8_t *pTxBuffer;			/* !< Holds the pointer to the Tx Buffer > */
	uint8_t *pRxBuffer;			/* !< Holds the pointer to the Rx Buffer > */
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxRxState;			/* !< I2C communication state > */
	uint8_t Sr;					/* !< Holds repeated start value > */
	uint8_t DevAddr;			/* !< Holds the Slave/Device Address > */
	uint8_t RxSize;
}I2C_Handle_t;



/******************* I2C DRIVER APIs PROTOTYPES**************************/

//APIs for initializing and deinitializing I2C
void i2c_init(I2C_Handle_t *pI2C_Handle);
void i2c_deinit(I2C_RegDef_t *pI2Cx);

//API to control peripheral clock
void I2C_PeriClk_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

//Sending/Receiving APIs
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t SlaveAddr, uint8_t Len, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

//IRQ Configuration and ISR Handling
void I2C_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di);
void I2C_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2C_Handle);


//Some additionals APIs
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


//Application call back
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
