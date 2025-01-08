/*
 * stm32f401ccu6_gpio_drivers.c
 *
 *  Created on: 28-Dec-2024
 *      Author: CITSC
 */

#include "stm32f401ccu6_gpio_drivers.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
//Peripheral Clock Setup
void GPIO_Peri_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t GPIOCLKEN)																	//Clock Control of Peripheral
{
	if(GPIOCLKEN == ENABLE){
		switch ((uint32_t) pGPIOx) {
			case (uint32_t)DRV_GPIOA: GPIOA_PCLK_ENA();
				break;
			case (uint32_t)DRV_GPIOB: GPIOB_PCLK_ENA();
				break;
			case (uint32_t)DRV_GPIOC: GPIOC_PCLK_ENA();
				break;
			case (uint32_t)DRV_GPIOD: GPIOD_PCLK_ENA();
				break;
			case (uint32_t)DRV_GPIOE: GPIOE_PCLK_ENA();
				break;
			case (uint32_t)DRV_GPIOH: GPIOH_PCLK_ENA();
				break;
		}
	}
	else {
		switch ((uint32_t)pGPIOx){
				case (uint32_t)DRV_GPIOA: GPIOA_PCLK_ENA();
					break;
				case (uint32_t)DRV_GPIOB: GPIOB_PCLK_ENA();
					break;
				case (uint32_t)DRV_GPIOC: GPIOC_PCLK_ENA();
					break;
				case (uint32_t)DRV_GPIOD: GPIOD_PCLK_ENA();
					break;
				case (uint32_t)DRV_GPIOE: GPIOE_PCLK_ENA();
					break;
				case (uint32_t)DRV_GPIOH: GPIOH_PCLK_ENA();
					break;
			}
		}
	}


//Initialisation

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialises peripheral as per GPIO_Handle_t type variable which includes address and initialisation parameters in GPIOx_RegDef_t for the given GPIO port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  Also supports interrupts involving rising and falling edges

 */


//GPIO Initialisation

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//1. Configure the mode of GPIO port
	if(pGPIOHandle->GPIO_PinConf.GPIO_PinMode <= GPIOANALOG){
		pGPIOHandle->pGPIOBasAddr->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
		pGPIOHandle->pGPIOBasAddr->MODER |= (pGPIOHandle->GPIO_PinConf.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	}
	else{
		if (pGPIOHandle->GPIO_PinConf.GPIO_PinMode == GPIORISEDGEDET) {
			{// 1. Configure the Rising trigger selection register
				DRV_EXTI->EXTI_RTSR |= (0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
				DRV_EXTI->EXTI_FTSR &= ~(0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
			}


		}else if (pGPIOHandle->GPIO_PinConf.GPIO_PinMode == GPIOFALEDGEDET) {
			// 1. Configure the Falling trigger selection register
			DRV_EXTI->EXTI_FTSR |= (0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
			DRV_EXTI->EXTI_RTSR &= ~(0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
		}
		else if (pGPIOHandle->GPIO_PinConf.GPIO_PinMode == GPIORISFALDET) {
			// 1. Configure the Rising & Falling trigger selection register
			DRV_EXTI->EXTI_FTSR |= (0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
			DRV_EXTI->EXTI_RTSR |= (0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
		}
		// 2. Configure the GPIO port selection in SYSCNFG_EXTICR
		switch (pGPIOHandle->GPIO_PinConf.GPIO_PinNo/4) {
			case 0:
				DRV_SYSCNFG->EXTICR1 |= (1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 4));
				break;
			case 1:
				DRV_SYSCNFG->EXTICR2 |= (1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 4));
				break;
			case 2:
				DRV_SYSCNFG->EXTICR3 |= (1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 4));
				break;
			case 3:
				DRV_SYSCNFG->EXTICR4 |= (1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 4));
				break;
			default:
				break;
		}

		// 3. Enable the EXTI interrupt delivery using  IMR
		uint8_t port_code;
		port_code = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOBasAddr);
		SYSCNFG_PCLK_ENA();
		DRV_EXTI->EXTI_IMR |= (port_code << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	}

	//2. Speed Configuration
	pGPIOHandle->pGPIOBasAddr->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	pGPIOHandle->pGPIOBasAddr->OSPEEDER |= (pGPIOHandle->GPIO_PinConf.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));

	//3. Pull up pull down configurations
	pGPIOHandle->pGPIOBasAddr->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	pGPIOHandle->pGPIOBasAddr->OTYPER |= (pGPIOHandle->GPIO_PinConf.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	pGPIOHandle->pGPIOBasAddr->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
	pGPIOHandle->pGPIOBasAddr->PUPDR |= (pGPIOHandle->GPIO_PinConf.GPIO_PinPuPdCtrl << (2 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));

	//4. Alternate functionality configuration
	if(pGPIOHandle->GPIO_PinConf.GPIO_PinMode == GPIOALTFN){
		if(pGPIOHandle->GPIO_PinConf.GPIO_PinNo <= 7)
			{
			pGPIOHandle->pGPIOBasAddr->AFRL &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
			pGPIOHandle->pGPIOBasAddr->AFRL |= (pGPIOHandle->GPIO_PinConf.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConf.GPIO_PinNo));
			}
		if(pGPIOHandle->GPIO_PinConf.GPIO_PinNo >= 8 && pGPIOHandle->GPIO_PinConf.GPIO_PinNo <= 15)
			{
			pGPIOHandle->pGPIOBasAddr->AFRL &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 8)));
			pGPIOHandle->pGPIOBasAddr->AFRH |= (pGPIOHandle->GPIO_PinConf.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConf.GPIO_PinNo % 8)));
			}
	}
}
//De-initialisation

/*********************************************************************
 * @fn      		  - GPIO_DInit
 *
 * @brief             - This function initialises peripheral as per GPIO_Handle_t type variable which includes address and initialisation parameters in GPIOx_RegDef_t for the given GPIO port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
//De-initialise GPIO

void GPIO_DInit(GPIO_RegDef_t *pGPIOx)																								//De-Initialise GPIO
{
	switch ((uint32_t) pGPIOx){
				case (uint32_t)DRV_GPIOA: GPIOA_REG_RST();
					break;
				case (uint32_t)DRV_GPIOB: GPIOB_REG_RST();
					break;
				case (uint32_t)DRV_GPIOC: GPIOC_REG_RST();
					break;
				case (uint32_t)DRV_GPIOD: GPIOD_REG_RST();
					break;
				case (uint32_t)DRV_GPIOE: GPIOE_REG_RST();
					break;
				case (uint32_t)DRV_GPIOH: GPIOH_REG_RST();
					break;
			}
}




/*********************************************************************
 * @fn      		  - GPIO_ReadIPin
 *
 * @brief             - This function reads from the configured pin from a particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - Pin Number of the port specific
 *
 * @return            -  0 or 1 from the pin of the port
 *
 * @Note              -  none

 */
// Input Reads
uint8_t GPIO_ReadIPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo)																	//Read from single pin
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> GPIO_PinNo) & (0x1));
	return value;
}



/*********************************************************************
 * @fn      		  - GPIO_ReadIPort
 *
 * @brief             - This function reads from a particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - None
 *
 * @return            -  0 or 1 from each port of the pin
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadIPort(GPIO_RegDef_t *pGPIOx)																						//Read from complete port
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}



// Output Writes
/*********************************************************************
 * @fn      		  - GPIO_WriteOPin
 *
 * @brief             - This function writes to the configured pin from a particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - Pin Number of the port specific
 * @param[in]         - Value to be written
 *
 * @return            -  0 or 1 to the pin of the port
 *
 * @Note              -  none

 */
void GPIO_WriteOPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo, uint8_t GPIOVAL)														//Write to single pin
{
	if(GPIOVAL == SET)
	{
		//write 1 to the output pin
		pGPIOx->ODR |= (1 << GPIO_PinNo);
	}
	else
	{
		//write 0 to the output pin
		pGPIOx->ODR &= ~(0x1 << GPIO_PinNo);
	}
}
/*********************************************************************
 * @fn      		  - GPIO_WriteOPort
 *
 * @brief             - This function writes to the particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - Pin Number of the port specific
 * @param[in]         - Value to be written
 *
 * @return            -  0 or 1 from each pin of the port
 *
 * @Note              -  none

 */
//Write to complete port
void GPIO_WriteOPort(GPIO_RegDef_t *pGPIOx, uint16_t GPIOVAL)																		//Write to complete port
{
	pGPIOx->ODR = GPIOVAL;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOPin
 *
 * @brief             - This function writes to the configured pin from a particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - Pin Number of the port specific
 *
 * @return            -  none
 *
 * @Note              -  none

 */
//Toggle output pins
void GPIO_ToggleOPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo)
{
	pGPIOx->ODR ^= (1 << GPIO_PinNo);
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOPort
 *
 * @brief             - This function toggles the particular port
 *
 * @param[in]         - base address of the particular GPIO peripheral
 * @param[in]         - Pin Number of the port specific
 *
 * @return            -  none
 *
 * @Note              -  none

 */
// Output Ports
void GPIO_ToggleOPort(GPIO_RegDef_t *pGPIOx)																						//Toggle output port
{
	// negate all values
	pGPIOx->ODR = ~(pGPIOx->ODR);
}


// Time delay

void delay(uint32_t microseconds) {
    // Assuming the system clock is running at 84 MHz
    uint32_t count = (84 * microseconds/5); // Adjust the divisor based on your clock speed and optimization level
    for (uint32_t i = 0; i < count; i++) {
        for (uint8_t var = 0; var < 10; ++var) {
			__asm__("nop"); // No Operation (NOP) instruction to waste time
        }
    }
}

/*********************************************************************
 * @fn      		  - GPIO_IRQIntrptConf
 *
 * @brief             - This function configures the interrupt pin to a number
 *
 * @param[in]         - Interrupt number to which it is to be configured
 * @param[in]         - Enable the interrupt
 *
 * @return            -  none
 *
 * @Note              -  none

 */
//Interrupt Setup and Handler
void GPIO_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA)																// ISR Priority setup
{
	if(IRQENA == 1){
	if(IRQNo >= 0 && IRQNo < 32) {
		*NVIC_ISER0 |= (1 << IRQNo);
		} else if(IRQNo >= 32 && IRQNo < 64){
		*NVIC_ISER1 |= (1 << (IRQNo % 32));
		} else if(IRQNo >= 64 && IRQNo < 96){
		*NVIC_ISER2 |= (1 << (IRQNo % 32));
		}else if(IRQNo >= 96 && IRQNo < 128){
		*NVIC_ISER3 |= (1 << (IRQNo % 32));
		}else if(IRQNo >= 128 && IRQNo < 160){
		*NVIC_ISER4 |= (1 << (IRQNo % 32));
		}else if(IRQNo >= 160 && IRQNo < 192){
		*NVIC_ISER5 |= (1 << (IRQNo % 32));
		}else if(IRQNo >= 192 && IRQNo < 224){
		*NVIC_ISER6 |= (1 << (IRQNo % 32));
		}else if(IRQNo >= 224 && IRQNo < 240){
		*NVIC_ISER7 |= (1 << (IRQNo % 32));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriConf
 *
 * @brief             - This function configures the interrupt pin to a priority
 *
 * @param[in]         - Interrupt priority to which it is to be configured
 * @param[in]         - Enable the interrupt
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo)
{
	*(NVIC_IPR0 + (IRQNo / 4)) |= (IRQPriority << ((8 * (IRQNo % 4)) + NVIC_IPR_SFT_AMT));
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHndlr
 *
 * @brief             - This function configures the interrupt pin to a priority
 *
 * @param[in]         - The below function turns on the Interrupt Handler Request for each function
 *
 *
 * @return            -  none
 *
 * @Note              -  EXTI Handlers are to be defined with the below function inside it as the default one is attributed to be weak.

 */
void GPIO_IRQHndlr(uint8_t GPIO_PinNo)
{
	if (DRV_EXTI->EXTI_PR & (1 << GPIO_PinNo)) {
		DRV_EXTI->EXTI_PR |= (1 << GPIO_PinNo);
	}
}
