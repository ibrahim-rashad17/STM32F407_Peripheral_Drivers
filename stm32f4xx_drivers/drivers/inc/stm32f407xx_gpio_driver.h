/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 20, 2024
 *      Author: Ibrahim
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15


/*
 * @GPIO_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_OP_SPEED_LOW			0
#define GPIO_OP_SPEED_MEDIUM		1
#define GPIO_OP_SPEED_FAST			2
#define GPIO_OP_SPEED_HIGH			3

/*
 * @GPIO_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_PUPD_NONE   				0
#define GPIO_PUPD_PULLUP				1
#define GPIO_PUPD_PULLDOWN				2

/*Configuration structure for a GPIO pin*/
typedef struct
{
	uint8_t 	GPIO_PinNumber;
	uint8_t 	GPIO_PinMode;
	uint8_t 	GPIO_PinSpeed;
	uint8_t 	GPIO_PinOPType;
	uint8_t 	GPIO_PinPuPdControl;
	uint8_t 	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/* Handle structure for GPIO pin
 *
 * User application will fill the structure and give to a driver API
 */

typedef struct
{
	GPIO_RegDef_t		*pGPIOx; 		/*!< This pointer holds the base address of GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t	GPIO_PinConfig; /*!< This holds the GPIO pin configuration settings >*/
}GPIO_Handle_t;

/******************* GPIO DRIVER APIs PROTOTYPES**************************/

//APIs for initializing and deinitializing GPIO
void gpio_init(GPIO_Handle_t *pGPIO_Handle);
void gpio_deinit(GPIO_RegDef_t *pGPIOx);

//APIs to read data from port/pin
uint16_t gpio_read_inp_port(GPIO_RegDef_t *pGPIOx);
uint8_t gpio_read_inp_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

//APIs to write data to an output port/pin
void gpio_write_to_op_port(GPIO_RegDef_t *pGPIOx, uint16_t value);
void gpio_write_to_op_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value);

void gpio_toggle_op_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

//IRQ Configuration and ISR Handling
void GPIO_IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di);
void GPIO_IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

//API to control peripheral clock
void GPIO_PeriClk_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
