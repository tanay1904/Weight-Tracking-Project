/*
 * stm32f401ccu6.h
 *
 *  Created on: Dec 26, 2024
 *      Author: CITSC
 */
#include <stdint.h>
#ifndef INC_STM32F401CCU6_H_
#define INC_STM32F401CCU6_H_
//////////////////////////////////////////////////////////
//					Memory base Addresses				//
//////////////////////////////////////////////////////////


#define DRV_FLASH_BASE_ADDR									(0x08000000U)															//Flash Memory Base Address
#define DRV_SRAM1_BASE_ADDR									(0x20000000U)															//SRAM1 Base Address(64kB)
#define DRV_ROM_BASE_ADDR									(0x1FFF0000U)															//ROM(System Memory) Base Address
#define DRV_SRAM_BASEADDR									DRV_SRAM1_BASE_ADDR
//#define SRAM2_BASE_ADDR									(0xXXXXXXXXU)															//SRAM2 Non-existent for the MCU


// MACROS for NVIC and Interrupt based communication protocols if needed
#define NVIC_ISER0											((volatile uint32_t*)0xE000E100U)										// ISER 0 register
#define NVIC_ISER1											((volatile uint32_t*)0xE000E104U)										// ISER 1 register
#define NVIC_ISER2											((volatile uint32_t*)0xE000E108U)										// ISER 2 register
#define NVIC_ISER3											((volatile uint32_t*)0xE000E10CU)										// ISER 3 register
#define NVIC_ISER4											((volatile uint32_t*)0xE000E110U)										// ISER 4 register
#define NVIC_ISER5											((volatile uint32_t*)0xE000E114U)										// ISER 5 register
#define NVIC_ISER6											((volatile uint32_t*)0xE000E118U)										// ISER 6 register
#define NVIC_ISER7											((volatile uint32_t*)0xE000E11CU)										// ISER 7 register

//
#define NVIC_ICER0											((volatile uint32_t*)0xE000E180U)										// ICER 0 register
#define NVIC_ICER1											((volatile uint32_t*)0xE000E184U)										// ICER 1 register
#define NVIC_ICER2											((volatile uint32_t*)0xE000E188U)										// ICER 2 register
#define NVIC_ICER3											((volatile uint32_t*)0xE000E18CU)										// ICER 3 register
#define NVIC_ICER4											((volatile uint32_t*)0xE000E190U)										// ICER 4 register
#define NVIC_ICER5											((volatile uint32_t*)0xE000E194U)										// ICER 5 register
#define NVIC_ICER6											((volatile uint32_t*)0xE000E198U)										// ICER 6 register
#define NVIC_ICER7											((volatile uint32_t*)0xE000E19CU)										// ICER 7 register

//
#define NVIC_IPR0											((volatile uint32_t*)0xE000E400U)										// IPR 0 register


// NVIC PRIORITY MACROS

#define NVIC_PRI_0											0x0																		// Priority 0
#define NVIC_PRI_1											0x1																		// Priority 1
#define NVIC_PRI_2											0x2																		// Priority 2
#define NVIC_PRI_3											0x3																		// Priority 3
#define NVIC_PRI_4											0x4																		// Priority 4
#define NVIC_PRI_5											0x5																		// Priority 5
#define NVIC_PRI_6											0x6																		// Priority 6
#define NVIC_PRI_7											0x7																		// Priority 7
#define NVIC_PRI_8											0x8																		// Priority 8
#define NVIC_PRI_9											0x9																		// Priority 9
#define NVIC_PRI_10											0xA																		// Priority 10
#define NVIC_PRI_11											0xB																		// Priority 11
#define NVIC_PRI_12 										0xC																		// Priority 12
#define NVIC_PRI_13											0xD																		// Priority 13
#define NVIC_PRI_14											0xE																		// Priority 14
#define NVIC_PRI_15											0xF																		// Priority 15


//
#define NVIC_IPR_SFT_AMT									4																		// Offset to available bits in Interrupt Priority Register

//////////////////////////////////////////////////////////
//		AHBx and APBx Peripherals Base Addresses		//
//////////////////////////////////////////////////////////

#define DRV_PERIPHERAL_BASE_ADDR							(0x40000000U)															//Base Address of Peripherals
#define DRV_AHB2_BASE_ADDR									(0x50000000U)															//Base Address of AHB2 Peripheral
#define DRV_AHB1_BASE_ADDR									(0x40020000U)															//Base Address of AHB1 Peripheral
#define DRV_APB2_BASE_ADDR									(0x40010000U)															//Base Address of APB2 Peripheral
#define DRV_APB1_BASE_ADDR									(0x40000000U)															//Base Address of APB1 Peripheral

//////////////////////////////////////////////////////////
//		Base Addresses of Peripherals on AHB1 			//
//////////////////////////////////////////////////////////


//GPIO Pin Addresses on AHB1
#define DRV_GPIOA_BASE_ADDR									(0x40020000U)															//Base Address of GPIOA Peripheral
#define DRV_GPIOB_BASE_ADDR									(0x40020400U)															//Base Address of GPIOB Peripheral
#define DRV_GPIOC_BASE_ADDR									(0x40020800U)															//Base Address of GPIOC Peripheral
#define DRV_GPIOD_BASE_ADDR									(0x40020C00U)															//Base Address of GPIOD Peripheral
#define DRV_GPIOE_BASE_ADDR									(0x40020100U)															//Base Address of GPIOE Peripheral
#define DRV_GPIOH_BASE_ADDR									(0x40021C00U)															//Base Address of GPIOH Peripheral


//Cyclic Redundancy Unit Pin Address on AHB1
#define DRV_CRC_BASE_ADDR									(0x40023000U)															//Base Address of CRC Peripheral


// Reset Control and Clock(RCC) Address on AHB1
#define DRV_RCC_BASE_ADDR									(0x40023800U)															//Base address of RCC Peripheral

// DMA Peripheral Addresses on AHB1 Bus
#define DRV_DMA2_BASE_ADDR									(0x40026400U)															//Base address of DMA2 Peripheral
#define DRV_DMA1_BASE_ADDR									(0x40026000U)															//Base address of DMA1 Peripheral


//////////////////////////////////////////////////////////
//		Base Addresses of Peripherals on APB2 			//
//////////////////////////////////////////////////////////

//Timing peripherals on APB2 bus
#define DRV_TIM11_BASE_ADDR									(0x40014800U)															//Base Address of TIM11 Peripheral
#define DRV_TIM10_BASE_ADDR									(0x40014400U)															//Base Address of TIM10 Peripheral
#define DRV_TIM1_BASE_ADDR									(0x40014000U)															//Base Address of TIM1 Peripheral

// External Interrupt register on APB2 bus
#define DRV_EXTI_BASE_ADDR									(0x40013C00U)															//Base Address of EXTI Peripheral

// System configuration register on APB2 bus
#define DRV_SYSCFG_BASE_ADDR								(0x40013800U)															//Base Address of SYSCFG Peripheral

//SPI peripheral on APB2 bus
#define DRV_SPI4_BASE_ADDR									(0x40013400U)															//Base Address of SPI4 Peripheral
#define DRV_SPI1_BASE_ADDR									(0x40013000U)															//Base Address of SPI1 Peripheral

// ADC peripherals on APB2 bus
#define DRV_ADC_BASE_ADDR									(0x40012000U)															//Base address of ADC1 Peripheral

// USART peripherals on APB2 bus
#define DRV_USART6_BASE_ADDR								(0x40011400U)															//Base address of USART6 Peripheral
#define DRV_USART1_BASE_ADDR								(0x40011000U)															//Base address of USART1 Peripheral




//////////////////////////////////////////////////////////
//		Base Addresses of Peripherals on APB1 			//
//////////////////////////////////////////////////////////


//Timing peripherals on APB1 bus
#define DRV_TIM5_BASE_ADDR									(0x40000C00U)															//Base Address of TIM5 Peripheral
#define DRV_TIM4_BASE_ADDR									(0x40000800U)															//Base Address of TIM4 Peripheral
#define DRV_TIM3_BASE_ADDR									(0x40000400U)															//Base Address of TIM3 Peripheral
#define DRV_TIM2_BASE_ADDR									(0x40000000U)															//Base Address of TIM2 Peripheral


//SPI peripheral on APB1 bus
#define DRV_SPI3_I2S3_BASE_ADDR								(0x40003C00U)															//Base Address of SPI3 Peripheral
#define DRV_SPI2_I2S2_BASE_ADDR								(0x40003800U)															//Base Address of SPI2 Peripheral


// USART peripherals on APB1 bus
#define DRV_USART2_BASE_ADDR								(0x40004400U)															//Base address of USART2 Peripheral


//I2C peripherals on APB1 bus
#define DRV_I2C1_BASE_ADDR									(0x40005400U)															//Base address of I2C1 Peripheral
#define DRV_I2C2_BASE_ADDR									(0x40005800U)															//Base address of I2C2 Peripheral
#define DRV_I2C3_BASE_ADDR									(0x40005C00U)															//Base address of I2C3 Peripheral




/*
 *
 * On board Pin and Port Naming
 *
 */

/*
 * Header 1 Ports
 * #define B10													PB10
 * #define B2													PB2
 * #define B1													PB1
 * #define B0													PB0
 * #define A7													PA7
 * #define A6													PA6
 * #define A5													PA5
 * #define A4													PA4
 * #define A3													PA3
 * #define A2													PA2
 * #define A1													PA1
 * #define A0													PA0
 * #define C15													PC15
 * #define C14													PC14
 * #define C13													PC13
 *
 *
 * Header2 Ports
 * #define B12													PB12
 * #define B13													PB13
 * #define B14													PB14
 * #define B15													PB15
 * #define A8													PA8
 * #define A9													PA9
 * #define A10													PA10
 * #define A11													PA11
 * #define A12													PA12
 * #define A15													PA15
 * #define B3													PB3
 * #define B4													PB4
 * #define B5													PB5
 * #define B6													PB6
 * #define B7													PB7
 * #define B8													PB8
 * #define B9													PB9

*/

//////////////////////////////////////////////////////////
//	  Register definitions for GPIO configurations		//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t MODER;																										//Offset 0x00	GPIO port mode register
	volatile uint32_t OTYPER;																										//Offset 0x04	GPIO port output type register
	volatile uint32_t OSPEEDER;																										//Offset 0x08	GPIO port output speed register
	volatile uint32_t PUPDR;																										//Offset 0x0C	GPIO port pull-up/pull-down register
	volatile uint32_t IDR;																											//Offset 0x10	GPIO port input data register
	volatile uint32_t ODR;																											//Offset 0x14	GPIO port output data register
	volatile uint32_t BSRR;																											//Offset 0x18	GPIO port bit set/reset register
	volatile uint32_t LCKR;																											//Offset 0x1C	GPIO port configuration lock register
	volatile uint32_t AFRL;																											//Offset 0x20	GPIO alternate function low register
	volatile uint32_t AFRH;																											//Offset 0x24	GPIO alternate function high register
}GPIO_RegDef_t;


//////////////////////////////////////////////////////////
//	  Register definitions for SYSCNFG configurations	//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t MEMRMP;																										//Offset 0x00	GPIO port mode register
	volatile uint32_t PMC;																											//Offset 0x04	GPIO port output type register
	volatile uint32_t EXTICR1;																										//Offset 0x08	GPIO port output speed register
	volatile uint32_t EXTICR2;																										//Offset 0x0C	GPIO port pull-up/pull-down register
	volatile uint32_t EXTICR3;																										//Offset 0x10	GPIO port input data register
	volatile uint32_t EXTICR4;																										//Offset 0x14	GPIO port output data register
	volatile uint32_t SYSCNFGReserved0;																								//Offset 0x18	GPIO port bit set/reset register
	volatile uint32_t SYSCNFGReserved1;																								//Offset 0x1C	GPIO port configuration lock register
	volatile uint32_t CMPCR;																										//Offset 0x20	GPIO alternate function low register
}SYSCNFG_RegDef_t;

//////////////////////////////////////////////////////////
//	  	Register definitions for SPI configurations		//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t SPI_CR1;																										//Offset 0x00	SPI control register 1
	volatile uint32_t SPI_CR2;																										//Offset 0x04	SPI control register 2
	volatile uint32_t SPI_SR;																										//Offset 0x08	SPI status register
	volatile uint32_t SPI_DR;																										//Offset 0x0C	SPI data register
	volatile uint32_t SPI_CRCPR;																									//Offset 0x10	SPI CRC polynomial register
	volatile uint32_t SPI_RXCRCR;																									//Offset 0x14	SPI RX CRC register
	volatile uint32_t SPI_TXCRCR;																									//Offset 0x18	SPI TX CRC register
	volatile uint32_t SPI_I2SCFGR;																									//Offset 0x1C	SPI_I2S configuration register
	volatile uint32_t SPI_I2SPR;																									//Offset 0x20	SPI_I2S prescaler register
}SPI_RegDef_t;

//////////////////////////////////////////////////////////
//	  	Register definitions for SPI configurations		//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t UART_SR;																										//Offset 0x00	USART Status register
	volatile uint32_t UART_DR;																										//Offset 0x04	USART Data register
	volatile uint32_t UART_BRR;																										//Offset 0x08	USART Baud rate register
	volatile uint32_t UART_CR1;																										//Offset 0x0C	USART Control register 1
	volatile uint32_t UART_CR2;																										//Offset 0x10	USART Control register 2
	volatile uint32_t UART_CR3;																										//Offset 0x14	USART Control register 3
	volatile uint32_t UART_GTPR;																									//Offset 0x18	USART Guard time and prescaler register
}UART_RegDef_t;


//////////////////////////////////////////////////////////
//	  Register definitions for RCC configurations		//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t CR;																											//Offset 0x00	RCC clock control register
	volatile uint32_t PLLCFGR;																										//Offset 0x04	RCC PLL configuration register
	volatile uint32_t CFGR;																											//Offset 0x08	RCC clock configuration register
	volatile uint32_t CIR;																											//Offset 0x0C	RCC clock interrupt register
	volatile uint32_t AHB1RSTR;																										//Offset 0x10	RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;																										//Offset 0x14	RCC AHB2 peripheral reset register
	volatile uint32_t ReservedRCC1;																									//Offset 0x18	RESERVED
	volatile uint32_t ReservedRCC2;																									//Offset 0x1C	RESERVED
	volatile uint32_t APB1RSTR;																										//Offset 0x20	RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;																										//Offset 0x24	RCC APB2 peripheral reset register
	volatile uint32_t ReservedRCC3;																									//Offset 0x28	RESERVED
	volatile uint32_t ReservedRCC4;																									//Offset 0x2C	RESERVED
	volatile uint32_t AHB1ENR;																										//Offset 0x30	RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;																										//Offset 0x34	RCC AHB2 peripheral clock enable register
	volatile uint32_t ReservedRCC5;																									//Offset 0x38	RESERVED
	volatile uint32_t ReservedRCC6;																									//Offset 0x3C	RESERVED
	volatile uint32_t APB1ENR;																										//Offset 0x40	RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;																										//Offset 0x44	RCC APB2 peripheral clock enable register
	volatile uint32_t ReservedRCC7;																									//Offset 0x48	RESERVED
	volatile uint32_t ReservedRCC8;																									//Offset 0x4C	RESERVED
	volatile uint32_t AHB1LPENR;																									//Offset 0x50	RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;																									//Offset 0x54	RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t ReservedRCC9;																									//Offset 0x58	RESERVED
	volatile uint32_t ReservedRCC10;																								//Offset 0x5C	RESERVED
	volatile uint32_t APB1LPENR;																									//Offset 0x60	RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;																									//Offset 0x64	RCC APB2 peripheral clock enabled in low power mode register
	volatile uint32_t ReservedRCC11;																								//Offset 0x68	RESERVED
	volatile uint32_t ReservedRCC12;																								//Offset 0x6C	RESERVED
	volatile uint32_t BDCR;																											//Offset 0x70	RCC Backup domain control register
	volatile uint32_t CSR;																											//Offset 0x74	RCC clock control & status register
	volatile uint32_t ReservedRCC13;																								//Offset 0x78	RESERVED
	volatile uint32_t ReservedRCC14;																								//Offset 0x7C	RESERVED
	volatile uint32_t SSCGR;																										//Offset 0x80	RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;																									//Offset 0x84	RCC PLLI2S configuration register
	volatile uint32_t ReservedRCC15;																								//Offset 0x88	RESERVED
	volatile uint32_t DCKCFGR;																										//Offset 0x8C	RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;


//////////////////////////////////////////////////////////
//	  Register definitions for EXTI configurations		//
//////////////////////////////////////////////////////////

typedef struct{
	volatile uint32_t EXTI_IMR;																										//Offset 0x00	RCC clock control register
	volatile uint32_t EXTI_EMR;																										//Offset 0x04	RCC PLL configuration register
	volatile uint32_t EXTI_RTSR;																									//Offset 0x08	RCC clock configuration register
	volatile uint32_t EXTI_FTSR;																									//Offset 0x0C	RCC clock interrupt register
	volatile uint32_t EXTI_SWIER;																									//Offset 0x10	RCC AHB1 peripheral reset register
	volatile uint32_t EXTI_PR;																										//Offset 0x14	RCC AHB2 peripheral reset register
}EXTI_RegDef_t;



////////////////////////////////////////////////////////////////////
//Peripheral definitions(Base addresses typecasted to xx_RegDef_t)//
////////////////////////////////////////////////////////////////////


// GPIO base address typecasting to RegDef pointer
#define DRV_GPIOA											((GPIO_RegDef_t*)DRV_GPIOA_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOA RegDef
#define DRV_GPIOB											((GPIO_RegDef_t*)DRV_GPIOB_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOB RegDef
#define DRV_GPIOC											((GPIO_RegDef_t*)DRV_GPIOC_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOC RegDef
#define DRV_GPIOD											((GPIO_RegDef_t*)DRV_GPIOD_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOD RegDef
#define DRV_GPIOE											((GPIO_RegDef_t*)DRV_GPIOE_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOE RegDef
#define DRV_GPIOH											((GPIO_RegDef_t*)DRV_GPIOH_BASE_ADDR)									//Typecasted pointer pointing to Base address of GPIOH RegDef

//RCC Base address typecasting to RegDef Pointer
#define DRV_RCC												((RCC_RegDef_t*)DRV_RCC_BASE_ADDR)										//Typecasted pointer pointing to Base address of RCC RegDef

//EXTI Base address typecasting to RegDef Pointer
#define DRV_EXTI											((EXTI_RegDef_t*)DRV_EXTI_BASE_ADDR)									//Typecasted pointer pointing to Base address of EXTI RegDef

//EXTI Base address typecasting to RegDef Pointer
#define DRV_SYSCNFG											((SYSCNFG_RegDef_t*)DRV_SYSCFG_BASE_ADDR)								//Typecasted pointer pointing to Base address of EXTI RegDef

//SPI Base Address typecasting to RegDef Pointer
#define DRV_SPI1											((SPI_RegDef_t*)DRV_SPI1_BASE_ADDR)										//Typecasted pointer pointing to Base address of SPI RegDef
#define DRV_SPI2_I2S2										((SPI_RegDef_t*)DRV_SPI2_I2S2_BASE_ADDR)								//Typecasted pointer pointing to Base address of SPI RegDef
#define DRV_SPI3_I2S3										((SPI_RegDef_t*)DRV_SPI3_I2S3_BASE_ADDR)								//Typecasted pointer pointing to Base address of SPI RegDef
#define DRV_SPI4											((SPI_RegDef_t*)DRV_SPI4_BASE_ADDR)										//Typecasted pointer pointing to Base address of SPI RegDef


//SPI Base Address typecasting to RegDef Pointer
#define DRV_USART1											((UART_RegDef_t*)DRV_USART1_BASE_ADDR)									//Typecasted pointer pointing to Base address of USART1 RegDef
#define DRV_USART2											((UART_RegDef_t*)DRV_USART2_BASE_ADDR)									//Typecasted pointer pointing to Base address of USART2 RegDef
#define DRV_USART6											((UART_RegDef_t*)DRV_USART6_BASE_ADDR)									//Typecasted pointer pointing to Base address of USART6 RegDef


/*
 *  Returns port code for given GPIOx base address
 */

#define GPIO_BASE_ADDR_TO_CODE(x)							  ((x == DRV_GPIOA) ? 0 : \
															  (x == DRV_GPIOB) ? 1 : \
															  (x == DRV_GPIOC) ? 2 : \
															  (x == DRV_GPIOD) ? 3 : \
															  (x == DRV_GPIOE) ? 4 : \
															  (x == DRV_GPIOH) ? 7 : 0)												//


/*
 * Clock Enable macros for GPIO Peripherals
*/

#define GPIOA_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<0))											//Enable clock for GPIOA Peripheral
#define GPIOB_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<1))											//Enable clock for GPIOB Peripheral
#define GPIOC_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<2))											//Enable clock for GPIOC Peripheral
#define GPIOD_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<3))											//Enable clock for GPIOD Peripheral
#define GPIOE_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<4))											//Enable clock for GPIOE Peripheral
#define GPIOH_PCLK_ENA()									(DRV_RCC->AHB1ENR |= (1<<7))											//Enable clock for GPIOH Peripheral



/*
 * Clock Enable macros for SPI Peripherals
*/
#define SPI1_PCLK_ENA()										(DRV_RCC->APB2ENR |= (1<<12))											//Enable clock for SPI1 Peripheral
#define SPI4_PCLK_ENA()										(DRV_RCC->APB2ENR |= (1<<13))											//Enable clock for SPI4 Peripheral
#define SPI2_I2S2_PCLK_ENA()								(DRV_RCC->APB1ENR |= (1<<14))											//Enable clock for SPI2 Peripheral
#define SPI3_I2S3_PCLK_ENA()								(DRV_RCC->APB1ENR |= (1<<15))											//Enable clock for SPI3 Peripheral

/*
 * Clock Enable macros for I2C Peripherals
*/
#define I2C1_PCLK_ENA()										(DRV_RCC->APB1ENR |= (1<<21))											//Enable clock for I2C1 Peripheral
#define I2C2_PCLK_ENA()										(DRV_RCC->APB1ENR |= (1<<22))											//Enable clock for I2C2 Peripheral
#define I2C3_PCLK_ENA()										(DRV_RCC->APB1ENR |= (1<<23))											//Enable clock for I2C3 Peripheral

/*
 * Clock Enable macros for USART Peripherals
*/
#define USART1_PCLK_ENA()									(DRV_RCC->APB2ENR |= (1<<4))											//Enable clock for USART1 Peripheral
#define USART2_PCLK_ENA()									(DRV_RCC->APB1ENR |= (1<<17))											//Enable clock for USART2 Peripheral
#define USART6_PCLK_ENA()									(DRV_RCC->APB2ENR |= (1<<5))											//Enable clock for USART6 Peripheral

/*
 * Clock Enable macros for SYSCNFG Peripherals
*/
#define SYSCNFG_PCLK_ENA()									(DRV_RCC->APB2ENR |= (1 << 14))											//Enable clock for SYSCNFG Peripheral



/*
 * Clock disable macros for GPIO Peripherals
*/

#define GPIOA_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<0))											//Disable clock for GPIOA Peripheral
#define GPIOB_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<1))											//Disable clock for GPIOB Peripheral
#define GPIOC_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<2))											//Disable clock for GPIOC Peripheral
#define GPIOD_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<3))											//Disable clock for GPIOD Peripheral
#define GPIOE_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<4))											//Disable clock for GPIOE Peripheral
#define GPIOH_PCLK_DENA()									(DRV_RCC->AHB1ENR (~&)= (1<<7))											//Disable clock for GPIOH Peripheral



/*
 * Clock disable macros for SPI Peripherals
*/
#define SPI1_PCLK_DENA()									(DRV_RCC->APB2ENR (~&)= (1<<12))										//Disable clock for SPI1 Peripheral
#define SPI2_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<14))										//Disable clock for SPI2 Peripheral
#define SPI3_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<15))										//Disable clock for SPI3 Peripheral
#define SPI4_PCLK_DENA()									(DRV_RCC->APB2ENR (~&)= (1<<13))										//Disable clock for SPI4 Peripheral

/*
 * Clock disable macros for I2C Peripherals
*/
#define I2C1_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<21))										//Disable clock for I2C1 Peripheral
#define I2C2_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<22))										//Disable clock for I2C2 Peripheral
#define I2C3_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<23))										//Disable clock for I2C3 Peripheral

/*
 * Clock disable macros for USART Peripherals
*/
#define USART1_PCLK_DENA()									(DRV_RCC->APB2ENR (~&)= (1<<4))											//Disable clock for USART1 Peripheral
#define UASRT2_PCLK_DENA()									(DRV_RCC->APB1ENR (~&)= (1<<17))										//Disable clock for USART2 Peripheral
#define USART6_PCLK_DENA()									(DRV_RCC->APB2ENR (~&)= (1<<5))											//Disable clock for USART6 Peripheral

/*
 * Clock disable macros for SYSCNFG Peripherals
*/
#define SYSCNFG_PCLK_DENA()									(DRV_RCC->APB2ENR (~&)= (1<<5))											//Disable clock for SYSCNFG Peripheral


/*
 * GENERIC MACROS
 */
#define ENABLE 												1 																		//ENABLE
#define DISABLE												0 																		//DISABLE
#define SET													ENABLE 																	//EQUIVALENT TO ENABLE
#define RESET												DISABLE																	//EQUIVALENT TO DISABLE
#define GPIO_PIN_SET										ENABLE																	//EQUIVALENT TO ENABLE
#define GPIO_PIN_RST										DISABLE																	//EQUIVALENT TO DISABLE
#define SPI_PIN_SET											ENABLE																	//EQUIVALENT TO ENABLE
#define SPI_PIN_RST											DISABLE																	//EQUIVALENT TO DISABLE


/*
 * GPIO PORT MODE REGISTER MACROS
 */


//Non-interrupt dependent
#define GPIOINPUT											0x0																		// Input Mode(Reset state)
#define GPIOOUTPUT											0x1																		// General purpose Output Mode
#define GPIOALTFN											0x2																		// Alternate function Mode(Reset state)
#define GPIOANALOG											0x3																		// Analog Mode(Reset state)

// Interrupt detection
#define GPIOFALEDGEDET										0x4																		// Detect a falling edge on input pin
#define GPIORISEDGEDET										0x5																		// Detect a rising edge on input pin
#define GPIORISFALDET										0x6																		// Detect rising and falling edges on input pin


/*
 * GPIO port output type register
 */

#define GPIO_OP_TYPE_PUPD									0x0																		// Output push pull(Default)
#define GPIO_OP_TYPE_OPNDRN									0x1																		// Open drain configuration

/*
 * GPIO port output speed register
 */
#define GPIO_SPEED_LOW										0x0																		// Low Speed Mode(Reset state)
#define GPIO_SPEED_MED										0x1																		// Medium Speed Mode(Reset state)
#define GPIO_SPEED_FAS										0x2																		// Fast Speed Mode(Reset state)
#define GPIO_SPEED_HIGH										0x3																		// High Speed Mode(Reset state)

/*
 * GPIO port pull-up/pull-down register
 */
#define NOPUPD												0x0																		// No pull up or pull down
#define PULLUP												0x1																		// Pull up
#define PULLDN												0x2																		// Pull down



/*
 * GPIO alternate function register
 */

#define GPIO_AF0											0x0																		// Alternate Function 0
#define GPIO_AF1											0x1																		// Alternate Function 1
#define GPIO_AF2											0x2																		// Alternate Function 2
#define GPIO_AF3											0x3																		// Alternate Function 3
#define GPIO_AF4											0x4																		// Alternate Function 4
#define GPIO_AF5											0x5																		// Alternate Function 5
#define GPIO_AF6											0x6																		// Alternate Function 6
#define GPIO_AF7											0x7																		// Alternate Function 7
#define GPIO_AF8											0x8																		// Alternate Function 8
#define GPIO_AF9											0x9																		// Alternate Function 9
#define GPIO_AF10											0xA																		// Alternate Function 10
#define GPIO_AF11											0xB																		// Alternate Function 11
#define GPIO_AF12											0xC																		// Alternate Function 12
#define GPIO_AF13											0xD																		// Alternate Function 13
#define GPIO_AF14											0xE																		// Alternate Function 14
#define GPIO_AF15											0xF																		// Alternate Function 15


/*
 *  Pin Naming
 */
#define GPIO_PIN_NO_0										0																		// Pin number 0 for PORTx
#define GPIO_PIN_NO_1										1																		// Pin number 1 for PORTx
#define GPIO_PIN_NO_2										2																		// Pin number 2 for PORTx
#define GPIO_PIN_NO_3										3																		// Pin number 3 for PORTx
#define GPIO_PIN_NO_4										4																		// Pin number 4 for PORTx
#define GPIO_PIN_NO_5										5																		// Pin number 5 for PORTx
#define GPIO_PIN_NO_6										6																		// Pin number 6 for PORTx
#define GPIO_PIN_NO_7										7																		// Pin number 7 for PORTx
#define GPIO_PIN_NO_8										8																		// Pin number 8 for PORTx
#define GPIO_PIN_NO_9										9																		// Pin number 9 for PORTx
#define GPIO_PIN_NO_10										10																		// Pin number 10 for PORTx
#define GPIO_PIN_NO_11										11																		// Pin number 11 for PORTx
#define GPIO_PIN_NO_12										12																		// Pin number 12 for PORTx
#define GPIO_PIN_NO_13										13																		// Pin number 13 for PORTx
#define GPIO_PIN_NO_14										14																		// Pin number 14 for PORTx
#define GPIO_PIN_NO_15										15																		// Pin number 15 for PORTx

/*
 * IRQ Number Macros
 */

#define DRV_EXTI0											6																		// External Interrupt Request Number 0
#define DRV_EXTI1											7																		// External Interrupt Request Number 1
#define DRV_EXTI2											8																		// External Interrupt Request Number 2
#define DRV_EXTI3											9																		// External Interrupt Request Number 3
#define DRV_EXTI4											10																		// External Interrupt Request Number 4
#define DRV_EXTI9_5											23																		// External Interrupt Request Number 5-9
#define DRV_EXTI15_10										40																		// External Interrupt Request Number 10-15
#define DRV_EXTI17											41																		// External Interrupt Request Number 17
#define DRV_EXT18											42																		// External Interrupt Request NUmber 18

/*
 * GPIO Reset Register Macros
 */
#define GPIOA_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<0));} while(0)		//Resets GPIOA Peripheral to default state
#define GPIOB_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<1));} while(0)		//Resets GPIOB Peripheral to default state
#define GPIOC_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<2));} while(0)		//Resets GPIOC Peripheral to default state
#define GPIOD_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<3));} while(0)		//Resets GPIOD Peripheral to default state
#define GPIOE_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<4));} while(0)		//Resets GPIOE Peripheral to default state
#define GPIOH_REG_RST()								do{(DRV_RCC->AHB1RSTR |= (1<<0)); (DRV_RCC->AHB1RSTR |= (1<<7));} while(0)		//Resets GPIOH Peripheral to default state



#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"
#include "stm32f401ccu6_spi_drivers.h"


#endif /* INC_STM32F401CCU6_H_ */
