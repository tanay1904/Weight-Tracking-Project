/*											File name: stm32f401ccu6_gpio_drivers.h
 * 											APIs Supported by the drivers
 * 				For more information on the functions refer to function definitions in stm32f401ccu6_gpio_drivers.c
 */

#ifndef INC_STM32F401CCU6_GPIO_DRIVERS_H_
#define INC_STM32F401CCU6_GPIO_DRIVERS_H_

#include <stdint.h>
#include "stm32f401ccu6.h"



/*
 * Pin configuration structure defined for pin handling structure
 */
typedef struct
{
	uint8_t GPIO_PinNo;																												//Select pin
	uint8_t GPIO_PinMode;																											//Setup Pin
	uint8_t GPIO_PinSpeed;																											//Set Pin speed
	uint8_t GPIO_PinPuPdCtrl;																										//Push-Pull Control of pin
	uint8_t GPIO_PinOPType;																											// Setup output type of pin
	uint8_t GPIO_PinAltFunMode;																										// Alternate function setup
}GPIO_PinConf_t;


/*
 * Handle Structure for GPIO Pin
 */
typedef struct
{
	//pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOBasAddr; 								       																//Holds the base address of GPIO port pin
	GPIO_PinConf_t GPIO_PinConf;																									//Holds the pin configuration of GPIO port pin
}GPIO_Handle_t;




//Peripheral Clock Setup
void GPIO_Peri_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t GPIOCLKEN);																	//Clock Control of Peripheral


//Initialisation
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);																							//Initialise GPIO
void GPIO_DInit(GPIO_RegDef_t *pGPIOx);																								//De-Initialise GPIO


// Input Reads
uint8_t GPIO_ReadIPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo);																	//Read from single pin
uint16_t GPIO_ReadIPort(GPIO_RegDef_t *pGPIOx);																						//Read from complete port


// Output Writes
void GPIO_WriteOPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo, uint8_t GPIOVAL);																				//Write to single pin
void GPIO_WriteOPort(GPIO_RegDef_t *pGPIOx, uint16_t GPIOVAL);																		//Write to complete port


// Output Ports
void GPIO_ToggleOPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNo);
void GPIO_ToggleOPort(GPIO_RegDef_t *pGPIOx);																						//Toggle output port


//Interrupt Setup and Handler
void GPIO_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA);																				// ISR Priority setup
void GPIO_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo);																			// ISR Priority setup
void GPIO_IRQHndlr(uint8_t GPIO_PinNo);																								// ISR Handler

// Time delay
void delay(uint32_t microseconds);																									// General delay in microseconds




#endif /* INC_STM32F401CCU6_GPIO_DRIVERS_H_ */
