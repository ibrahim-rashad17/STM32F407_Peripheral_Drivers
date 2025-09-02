/*
 * stm32f407xx.h
 *
 *  Created on: Feb 2, 2024
 *      Author: Ibrahim
 *
 *		This is an MCU specific header file, refer the RM of MCU
 *		In this case RM0090 is referred
 *
 *      We are developing drivers for GPIOs, I2C and SPI
 *      Hence only base addresses required for these will be documented here
 *
 *      Peripherals on AHB1 bus :
 *      	1. GPIOA to GPIOI
 *      	2. RCC
 *
 *      Peripherals on APB1 bus :
 *      	1. 12C 1,2,3
 *      	2. USART 2,3
 *      	3. UART 4,5
 *      	4. SPI 2,3
 *
 *      Peripherals on APB2 bus :
 *      	1. SPI 1,4,5,6
 *      	2. USART 1,6
 *      	3. EXTI
 *      	4. SYSCFG
 *
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo 	volatile

//Generic Macros
#define HIGH			1
#define LOW 			0
#define SET				HIGH
#define RESET			LOW
#define ENABLE			HIGH
#define DISABLE			LOW
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

/**********************************Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	0xE000E400

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: complete this list for other peripherals which we need as we progress
 */

typedef enum
{
	//Related to GPIO
	EXTI0					= 6,
	EXTI1					= 7,
	EXTI2					= 8,
	EXTI3					= 9,
	EXTI4					= 10,
	EXTI5_9					= 23,
	EXTI15_10				= 40,

	//Related to I2C
	I2C1_EV					= 31,
	I2C1_ER					= 32,
	I2C2_EV					= 33,
	I2C2_ER					= 34,
	I2C3_EV					= 72,
	I2C3_ER					= 73,

	//Related to USART/UART
	USART1_IT				= 37,
	USART2_IT				= 38,
	USART3_IT				= 39,
}IRQ_Number_t;

/******************************************************************/
/*				PERIPHERAL REGISTERS BIT POSITIONS				  */
/******************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_SR1_SB				 		0
#define I2C_SR1_ADDR			 		1
#define I2C_SR1_BTF				 		2
#define I2C_SR1_STOPF 			    	4
#define I2C_SR1_RXNE				 	6
#define I2C_SR1_TXE			    		7
#define I2C_SR1_BERR 			    	8
#define I2C_SR1_ARLO				 	9
#define I2C_SR1_AF 			    		10
#define I2C_SR1_OVR				 		11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/*
 * I2C FLAGS
 */
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE					(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE					(1 << I2C_SR1_TXE)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)

/* USART BIT DEFINITION MACROS	*/
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

#define USART_CR2_STOP				12

#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

/******************************************************************/
/*				BASE ADDRESSES OF MEMORIES						  */
/******************************************************************/

/* Memory Base Addresses */

#define FLASH_BASEADDR			0x08000000U	  /* Base address of Flash memory of MCU */
#define SRAM1_BASEADDR			0x20000000U
#define SRAM					SRAM1_BASEADDR /* SRAM1 is the main SRAM	*/
#define SRAM2_BASEADDR			0x2001C000U	  /* SRAM1 base address + size of SRAM1 (112KB)	*/
#define ROM						0x1FFF0000U	  /* System memory in Flash is ROM */


/* AHBx and APBx Peripheral Base addresses */

#define PERIPH_BASEADDR		    0x40000000U
#define APB1_PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR	0x40010000U
#define AHB1_PERIPH_BASEADDR	0x40020000U
#define AHB2_PERIPH_BASEADDR	0x50000000U


/******************************************************************/
/*				BASE ADDRESSES OF PERIPHERALS					  */
/******************************************************************/

/*
 * Base addresses of peripherals hanging on AHB1 bus
 *
 * Only including base addresses of peripherals required for our driver development
 * In case of AHB1 it is only GPIOs and RCC
*/

#define GPIOA_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0000)	/* Base addresses for GPIO peripherals(A to I) */
#define	GPIOB_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0400)
#define	GPIOC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0800)
#define	GPIOD_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0C00)
#define	GPIOE_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1000)
#define	GPIOF_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1400)
#define	GPIOG_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1800)
#define	GPIOH_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1C00)
#define	GPIOI_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x3800)  /* Base address of Reset and Clock Control register */

/* Base addresses of peripherals hanging on APB1 bus */

#define I2C1_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR			(APB1_PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1_PERIPH_BASEADDR + 0x7C00)

/* Base addresses of peripherals hanging on APB2 bus */

#define EXTI_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3C00)	/* Base address of External interrupt/event controller register */
#define SPI1_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR			(APB2_PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR			(APB2_PERIPH_BASEADDR + 0x5400)
#define SYSCFG_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3800)	/* Base address of System Configuration Controller register */
#define USART1_BASEADDR			(APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2_PERIPH_BASEADDR + 0x1400)

/******************************************************************/
/*			PERIPHERAL REGISTER STRUCTURE DEFINITIONS			  */
/******************************************************************/

/* GPIO register definition structure
 *
 * Note: AFR[0] -> Low register 		Address offset: 0x20
 * 		 AFR[1] -> High register 		Address offset: 0x24
 */
typedef struct
{
	__vo uint32_t		MODER; 		/* GPIO port mode register													Address offset : 0x00	*/
	__vo uint32_t		OTYPER;		/* GPIO port output type register											Address offset : 0x04	*/
	__vo uint32_t		OSPEEDR;	/* GPIO port output speed register											Address offset : 0x08	*/
	__vo uint32_t		PUPDR;		/* GPIO port pull-up/pull-down register										Address offset : 0x0C	*/
	__vo uint32_t		IDR;		/* GPIO port input data register											Address offset : 0x10	*/
	__vo uint32_t		ODR;		/* GPIO port output data register											Address offset : 0x14	*/
	__vo uint32_t		BSSR;		/* GPIO port bit set/reset register											Address offset : 0x18	*/
	__vo uint32_t		LCKR;		/* GPIO port configuration lock register									Address offset : 0x1C	*/
	__vo uint32_t		AFR[2];		/* GPIO port Alternate function register ( AFR[0]->Low , AFR[1]->High )		Address offset : 0x20	*/
}GPIO_RegDef_t;


/* Peripheral register definition structure for I2C */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/* Peripheral register definition structure for RCC */
typedef struct
{
	__vo uint32_t CR;            /*!< RCC Clock control register     										Address offset: 0x00 */
	__vo uint32_t PLLCFGR;       /*!< RCC PLL Configuration register     										Address offset: 0x04 */
	__vo uint32_t CFGR;          /*!< RCC Clock configuration register     										Address offset: 0x08 */
	__vo uint32_t CIR;           /*!< RCC Clock interrupt register     										Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register     										Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register     										Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register     										Address offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	__vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register     										Address offset: 0x20 */
	__vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register     										Address offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	__vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register     										Address offset: 0x30 */
	__vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register     										Address offset: 0x34 */
	__vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register     										Address offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	__vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock register     										Address offset: 0x40 */
	__vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock register     										Address offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	__vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	__vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	__vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	__vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	__vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	__vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	__vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	__vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	__vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */
} RCC_RegDef_t;


/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;    /*!< Status Register          	  	    		Address offset: 0x00 */
	__vo uint32_t DR;    /*!< Data Register  		              		Address offset: 0x04 */
	__vo uint32_t BRR;   /*!< Rising Trigger selection register		    Address offset: 0x08 */
	__vo uint32_t CR1;   /*!< Control register 1						Address offset: 0x0C */
	__vo uint32_t CR2;   /*!< Control register 2			   		    Address offset: 0x10 */
	__vo uint32_t CR3;   /*!< Control register 3	                 	Address offset: 0x14 */
	__vo uint32_t GTPR;	 /*!< Guard time and prescalar register         Address offset: 0x18 */
}USART_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Interrupt Mask Register          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< Event Mask Register                			Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< Rising Trigger selection register		    Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< Falling Trigger selection register			Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< Software Interrupt event register   		    Address offset: 0x10 */
	__vo uint32_t PR;     /*!< Pending request register                 	Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*!< Memory remap register          	  	    		Address offset: 0x00 */
	__vo uint32_t PMC;			/*!< Peripheral mode configuration register          	Address offset: 0x04 */
	__vo uint32_t EXTICR[4];	/*!< External interrupt configuration registers         Address offset: 0x08 */
	__vo uint32_t RSRVD[2];		/*!< Reserved         									Address offset: 0x18 */
	__vo uint32_t CMPCR;		/*!< Compensation cell control register          		Address offset: 0x20 */
}SYSCFG_RegDef_t;

#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : -1	)

/* Peripheral definitions (Macros to access peripheral registers directly) */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)
#define UART7			((USART_RegDef_t*)UART7_BASEADDR)
#define UART8			((USART_RegDef_t*)UART8_BASEADDR)
/******************************************************************/
/*			PERIPHERAL CLOCK ENABLE/DISABLE MACROS				  */
/******************************************************************/

/* Clock enable macros for GPIO peripherals */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/* Clock disable macros for GPIO peripherals */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/* Clock enable/disable macros for SYSCFG */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

/* Clock enable macros for USART peripherals */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1<<5))

/* Clock disable macros for USART peripherals */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1<<5))

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
