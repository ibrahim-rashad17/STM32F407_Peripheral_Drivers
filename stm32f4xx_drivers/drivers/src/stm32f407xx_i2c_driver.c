/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 9, 2025
 *      Author: Ibrahim
 */

#include "stm32f407xx_i2c_driver.h"

static uint8_t Get_I2C_FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FLAG);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t rw);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C_Handle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C_Handle);

static void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle);
static void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}

void i2c_init(I2C_Handle_t *pI2C_Handle)
{
	uint32_t tempreg =0;
	uint16_t ccr_value = 0;
	uint8_t trise = 0;

	I2C_PeripheralControl(pI2C_Handle->pI2Cx, ENABLE);

	//1. Configuring the ack
	if(pI2C_Handle->i2c_config.I2C_Ack == ENABLE)
		I2C_ManageAcking(pI2C_Handle->pI2Cx, ENABLE);
	else
		I2C_ManageAcking(pI2C_Handle->pI2Cx, DISABLE);

	//2. Configure own address register
	uint8_t temp = pI2C_Handle->i2c_config.DeviceAddress & 0xFE;
	pI2C_Handle->pI2Cx->OAR1 &= ~(0x7F << 1);
	pI2C_Handle->pI2Cx->OAR1 |= (temp << 1);

	pI2C_Handle->pI2Cx->OAR1 |= (1 << 14);	//Mentioned in ref manual to always keep at 1

	//Configuring the serial clock (first configure the FREQ field)
	uint32_t freq = RCC_GetPCLK1Value();

	temp = freq/6;		//Frequency is in MHz

	pI2C_Handle->pI2Cx->CR2 &= 0x3F;	//Clearing bits
	pI2C_Handle->pI2Cx->CR2 |= ((temp & 0x3F) << I2C_CR2_FREQ);

	//Configuring the CCR
	if(pI2C_Handle->i2c_config.I2C_SCLK_Speed <= I2C_SPEED_SM_100K)
	{
		ccr_value = (pI2C_Handle->i2c_config.I2C_SCLK_Speed) / (2 * RCC_GetPCLK1Value());
		tempreg = ccr_value & 0x0FFF;

		pI2C_Handle->pI2Cx->CCR = tempreg;
	}
	else
	{
		//Check for Duty cycle and take steps based on that
		if(pI2C_Handle->i2c_config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (pI2C_Handle->i2c_config.I2C_SCLK_Speed) / (3 * RCC_GetPCLK1Value());
			tempreg = ccr_value & 0x0FFF;

			pI2C_Handle->pI2Cx->CCR = tempreg;
		}
		else
		{
			ccr_value = (pI2C_Handle->i2c_config.I2C_SCLK_Speed) / (25 * RCC_GetPCLK1Value());
			tempreg = ccr_value & 0x0FFF;

			tempreg |= (1 << I2C_CCR_DUTY);

			pI2C_Handle->pI2Cx->CCR = tempreg;
		}

		//Fast mode
		tempreg |= (1 << (I2C_CCR_FS));
	}

	/* Configure TRISE
	 *
	 * As per I2C specifications:-
	 * 		Max Trise for standard mode = 1000ns
	 * 		Max Trise for fast mode	    = 300ns
	 *
	 * Refer RM0090 to get Trise register details
	 *
	 */

	if(pI2C_Handle->i2c_config.I2C_SCLK_Speed <= I2C_SPEED_SM_100K)		//Standard mode
	{
		trise = ( RCC_GetPCLK1Value() / (1000000U) ) + 1;
	}
	else	//Fast mode
	{
		trise = ( (RCC_GetPCLK1Value() * 300) / (1000000000U) ) + 1;
	}

	pI2C_Handle->pI2Cx->TRISE |= (trise & 0x3F);

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -	API to send data as master (blocking API)
 *
 * @param[in]         - I2C Peripheral Handle
 * @param[in]         -	Pointer to TxBuffer
 * @param[in]         - Slave Address
 * @param[in]		  - Length of data to be transmitted
 * @param[in]         - Repeated start (Enable or Disable)
 *
 * @return            - none
 *
 * @Note              -
 */
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t SlaveAddr, uint8_t Len, uint8_t Sr)
{
	//1. Generate start condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	//2. Wait for SB Flag to be set
	while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB)) );

	//3. Write the slave addrs to DR (R/W' bit = 0 for write operation)
	I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, SlaveAddr, I2C_WRITE_MODE);

	//4. Wait for ADDR Flag to be set
	while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR)) );

	//5. Clear the ADDR Flag by reading SR1 and SR2 registers
	I2C_ClearADDRFlag(pI2C_Handle);

	//6. Transmit data until len becomes zero
	while(Len > 0)
	{
		while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE)) );
		pI2C_Handle->pI2Cx->DR = (*pTxBuffer & 0xFF);
		pTxBuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE)) );
	while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF)) );

	//8. Generate stop condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == DISABLE)
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1.Generate START condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	//2.confirm that start generation was completed
	while(! Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) );

	//3. Write the slave addrs to DR (R/W' bit = 1 for read operation)
	I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, SlaveAddr, I2C_READ_MODE);

	//4. wait until address is compleate by checking ADDR flag in SR1
	while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR)) );

	if(Len == 1)
	{
		// Read a single byte from Slave

		//1. Disable Acking
		I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

		//2.Clear the ADDR flag
		I2C_ClearADDRFlag(pI2C_Handle);

		//3.wait until RXNE = 1
		while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE)) );

		//4.Generate STOP condition
		if(Sr == DISABLE)
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

		//.5Read data in the buffer DR
		*pRxBuffer = pI2C_Handle->pI2Cx->DR & 0xFF;

	}

	if(Len>1)			//read multiple bytes
	{
		//Clear address flag
		I2C_ClearADDRFlag(pI2C_Handle);

		for(uint32_t i=Len;i>0;i--)
		{
			//Wait for RXNE = 1
			while( !(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE)) );

			if(i == 2)
			{
				// Disable Acking
				pI2C_Handle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				//Generate stop condition (as per RM)
				if(Sr == DISABLE)
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
			}

			//Read DR and increment buffer ptr
			*pRxBuffer = pI2C_Handle->pI2Cx->DR & 0xFF;
			pRxBuffer++;
		}

	}

	//re-enable ACKing
	if(pI2C_Handle->i2c_config.I2C_Ack == ENABLE)
		pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	uint8_t data;

	data = (uint8_t)pI2Cx->DR & 0xFF;
	return data;
}
/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -	API to trigger I2C master data transmission (Interrupt based)
 *
 * @param[in]         - I2C Peripheral Handle
 * @param[in]         -	Pointer to RxBuffer
 * @param[in]         - Length of data to be received
 * @param[in]         - Repeated start (Enable or Disable)
 *
 * @return            - I2C Peripheral state
 *
 * @Note              - If you dont return value as I2C_READY, it means I2C has
 * 						failed to start. It is likely busy in TX/RX.
 * 						Try again later
 * 						Also Interrupt Handling doesnt happen here. Here only required
 * 						flags are set and buffers are initialized etc.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BSY_IN_TX) && (busystate != I2C_BSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -	API to trigger I2C master data reception (Interrupt based)
 *
 * @param[in]         - I2C Peripheral Handle
 * @param[in]         -	Pointer to RxBuffer
 * @param[in]         - Length of data to be received
 * @param[in]         - Repeated start (Enable or Disable)
 *
 * @return            - I2C Peripheral state
 *
 * @Note              - If you dont return value as I2C_READY, it means I2C has
 * 						failed to start. It is likely busy in TX/RX.
 * 						Try again later
 * 						Also Interrupt Handling doesnt happen here. Here only required
 * 						flags are set and buffers are initialized etc.

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BSY_IN_TX) && (busystate != I2C_BSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxSize = Len;
		pI2CHandle->TxRxState = I2C_BSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle)
{
	uint8_t temp1,temp2,temp3;

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB);
	temp2 = pI2C_Handle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);
	temp3 = pI2C_Handle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);

	//Check due to which event the interrupt has occurred
	if(temp1 && temp3)
	{
		//Interrupt due to start bit
		if(pI2C_Handle->TxRxState == I2C_BSY_IN_TX)
			I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_WRITE_MODE);
		else if(pI2C_Handle->TxRxState == I2C_BSY_IN_RX)
			I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_READ_MODE);
	}

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR);

	if(temp1 && temp3)
	{
		//Interrupt due to ADDR phase completion
		I2C_ClearADDRFlag(pI2C_Handle);
	}

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF);

	if(temp1 && temp3)
	{
		//Interrupt due to BTF
		if(pI2C_Handle->TxRxState == I2C_BSY_IN_TX)
		{
			if(Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE))
			{
				if( !(pI2C_Handle->TxLen) )
				{
					if(pI2C_Handle->Sr)
						I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

					//Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2C_Handle);

					I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_TX_CMPLT);
				}
			}

		}
	}

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_STOPF);

	if(temp1 & temp3)
	{
		//Interrupt due to STOPF (slave mode) , To clear STOPF, Read SR1 and write to CR1 (SR1 already read above)
		pI2C_Handle->pI2Cx->CR1 |= 0x0000;

		//Notify the application
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_STOP_DETECTED);
	}

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE);

	//Interrupt due to TXE Flag Set
	if(temp1 & temp2 & temp3)
	{
		if( (pI2C_Handle->pI2Cx->CR2) & (1 << I2C_SR2_MSL) )
		{
			if(pI2C_Handle->TxRxState == I2C_BSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2C_Handle);
			}
		}
		else
		{
			if( (pI2C_Handle->pI2Cx->SR2) & (1 << I2C_SR2_TRA) )
			{
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp1 = Get_I2C_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE);

	//Interrupt due to RXNE flag set
	if(temp1 & temp2 & temp3)
	{
		if( (pI2C_Handle->pI2Cx->CR2) & (1 << I2C_SR2_MSL) )
		{
			if(pI2C_Handle->TxRxState == I2C_BSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2C_Handle);
			}
		}
		else
		{
			if( !(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) )
			{
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_RCV);
			}
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C_Handle)
{
	if(pI2C_Handle->TxLen > 0)
	{
		//Load data into DR
		pI2C_Handle->pI2Cx->DR = *(pI2C_Handle->pTxBuffer) & 0xFF;

		//Point to next item in buffer
		pI2C_Handle->pTxBuffer++;

		//Decrement the Length
		pI2C_Handle->TxLen--;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C_Handle)
{
	if(pI2C_Handle->RxSize == 1)
	{
		/* Note: Ack is disabled b4 clearing ADDR flag for 1 byte data reception*/

		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->DR & 0xFF;
		pI2C_Handle->pRxBuffer++;
		pI2C_Handle->RxLen--;
	}
	else if(pI2C_Handle->RxSize > 1)
	{
		if(pI2C_Handle->RxLen == 2)
		{
			I2C_ManageAcking(pI2C_Handle->pI2Cx, DISABLE);
		}

		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->DR & 0xFF;
		pI2C_Handle->pRxBuffer++;
		pI2C_Handle->RxLen--;
	}

	if(pI2C_Handle->RxLen == 0)
	{
		if(pI2C_Handle->Sr == DISABLE)
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

		I2C_CloseReceiveData(pI2C_Handle);

		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -	This API clears the I2C Error Flag and notifies the user application
 *
 * @param[in]         - Handle to Peripheral structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_TIMEOUT);
	}

}

static void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle)
{
	//Disable the interrupt flags
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset the handle structure
	pI2C_Handle->TxLen = 0;
	pI2C_Handle->pTxBuffer = NULL;
	pI2C_Handle->TxRxState = I2C_READY;
}

static void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle)
{
	//Disable the interrupt flags
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset the handle structure
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->pRxBuffer = NULL;
	pI2C_Handle->TxRxState = I2C_READY;

	//Re-enable acking (if enabled by user)
	if(pI2C_Handle->Sr == ENABLE)
		I2C_ManageAcking(pI2C_Handle->pI2Cx, ENABLE);
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t rw)
{
	if(rw == I2C_WRITE_MODE)
		pI2Cx->DR = ( (SlaveAddr << 1) & 0xFE );
	else
		pI2Cx->DR = ( (SlaveAddr << 1) | 0x01 );
}

static uint8_t Get_I2C_FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FLAG)
{
	if( (pI2Cx->SR1) & (1 << FLAG) )
		return HIGH;
	return LOW;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle)
{
	uint32_t dummy_read;

	//If in master mode and Busy in Rx
	if((pI2C_Handle->TxRxState == I2C_BSY_IN_RX) && (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL) ))
	{
		if(pI2C_Handle->RxSize == 1)
		{
			I2C_ManageAcking(pI2C_Handle->pI2Cx, DISABLE);
		}
	}

	//Sequence to clear ADDR flag
	dummy_read = pI2C_Handle->pI2Cx->SR1;
	dummy_read = pI2C_Handle->pI2Cx->SR2;
	(void)dummy_read;
}

void I2C_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di)
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

void I2C_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t *pPrioRegAddr = (uint8_t*)NVIC_PR_BASE_ADDR + (IRQNumber & 0xFF);

	IRQPriority  = IRQPriority << NO_PR_BITS_IMPLEMENTED;	//4 priority bits implemented

	*pPrioRegAddr = IRQPriority;
}

