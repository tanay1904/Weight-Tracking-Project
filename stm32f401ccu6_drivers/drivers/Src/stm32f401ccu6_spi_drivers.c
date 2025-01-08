/*
 * stm32f401ccu6_spi_drivers.c
 *
 *  Created on: 01-Jan-2025
 *      Author: CITSC
 */
#include <stdint.h>
#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"
#include "stm32f401ccu6_spi_drivers.h"



// Function to get flag status(SPI Header file "has" to be included in the main file due to inclusion of this function in the main file directly)
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// Pin Initialisation for Alternate function of GPIO pins
void SPI_PinInit(GPIO_Handle_t GPIOPin, GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t AltFn);


/*********************************************************************
 * @fn      		  - SPI_Peri_ClkCtrl
 *
 * @brief             - This function enables the peripheral clock
 *
 * @param[in]         - base address of the particular SPI peripheral
 * @param[in]         - ENABLE or DISABLE
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
//Peripheral Clock Setup

void SPI_Peri_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t SPICLKEN)																		//Clock Control of Peripheral
{
	if(SPICLKEN == 1){
		if(pSPIx == DRV_SPI1)
			SPI1_PCLK_ENA();
		else if (pSPIx == DRV_SPI2_I2S2)
			SPI2_I2S2_PCLK_ENA();
		else if (pSPIx == DRV_SPI3_I2S3)
			SPI3_I2S3_PCLK_ENA();
		else if (pSPIx == DRV_SPI4)
			SPI4_PCLK_ENA();
	}
}




/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initialises the Peripheral according to the Pin configuration provided
 *
 *
 * @param[in]         - base address of the particular SPI Handle structure
 * @param[in]         - Software pull up or pull down of NSS pin when Software Slave Mode is Active(Can be left 0 or 1 if not SW configured)
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */


//Initialisation
void SPI_Init(SPI_Handle_t *pSPIHandle, uint8_t SSI)  																							//Initialise SPI
{
	SPI_Peri_ClkCtrl(pSPIHandle->pSPIx, ENABLE);
	pSPIHandle->pSPIx->SPI_CR1 &= ~((1 << 2) | (1 << 15) | (0x7 << 3) | (1 << 1) | (1 << 0) | (1 << 6) | (1 << 9) | (1 << 14) | (1 << 10));
	if(SSI){
		if (pSPIHandle->SPI_Config.SPI_SSM)
			{
				pSPIHandle->pSPIx->SPI_CR1 |= (SSI << 8);
			}
		else
			{
				pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 8);
			}
		}

	if((pSPIHandle->SPI_Config.SPIBusMode == SPI_FULL_DUP) || (pSPIHandle->SPI_Config.SPIBusMode == SPI_HALF_DUP) || (pSPIHandle->SPI_Config.SPIBusMode == SPI_SLV_TXS) || (pSPIHandle->SPI_Config.SPIBusMode == SPI_MAS_TXS))
	{
		pSPIHandle->pSPIx->SPI_CR1 |= ((pSPIHandle->SPI_Config.SPIDevMode << 2) | (pSPIHandle->SPI_Config.SPIBusMode << 15) | (pSPIHandle->SPI_Config.SPISCKSpeed << 3) | (pSPIHandle->SPI_Config.SPI_CPOL << 1) | (pSPIHandle->SPI_Config.SPI_CPHA << 0) | (1 << 6) | (pSPIHandle->SPI_Config.SPI_SSM << 9));
	}
	if ((pSPIHandle->SPI_Config.SPIDevMode == SPI_MAST_DEV) || (pSPIHandle->SPI_Config.SPIDevMode == SPI_SLAV_DEV))
	{
		pSPIHandle->pSPIx->SPI_CR1 |= ((pSPIHandle->SPI_Config.SPIDevMode << 2) | (pSPIHandle->SPI_Config.SPIBusMode << 15) | (pSPIHandle->SPI_Config.SPISCKSpeed << 3) | (pSPIHandle->SPI_Config.SPI_CPOL << 1) | (pSPIHandle->SPI_Config.SPI_CPHA << 0) | (1 << 6) | (pSPIHandle->SPI_Config.SPI_SSM << 9) | (1 << 10));
	}
}


/*********************************************************************
 * @fn      		  - SPI_PinInit
 *
 * @brief             - This function initialises the GPIO pin to the respective alternate function
 *
 * @param[in]         - base address of the particular GPIO Handle Structure
 * @param[in]         - base address of the particular GPIO Register Definition Structure
 * @param[in]         - Pin Number of the port specific
 * @param[in]         - Alternate function number
 *
 * @return            -  0 or 1 to the pin of the port
 *
 * @Note              -  none
 *
 */
void SPI_PinInit(GPIO_Handle_t GPIOPin, GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t AltFn)
{
	GPIOPin.pGPIOBasAddr = pGPIOx;
	GPIOPin.GPIO_PinConf.GPIO_PinNo = PinNo;
	GPIOPin.GPIO_PinConf.GPIO_PinMode = GPIOALTFN;
	GPIOPin.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_MED;
	GPIOPin.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
	GPIOPin.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	GPIOPin.GPIO_PinConf.GPIO_PinAltFunMode = AltFn;
	GPIO_Init(&GPIOPin);
}


/*********************************************************************
 * @fn      		  -  SPI_DInit
 *
 *
 * @param[in]         -  base address of the particular GPIO Register Definition Structure
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
void SPI_DInit(SPI_RegDef_t *Peripheral)																								//De-Initialise SPI
{
	RCC_RegDef_t *pRCCx = DRV_RCC;
	if(Peripheral == DRV_SPI2_I2S2)
		pRCCx->APB1RSTR |= (1 << 14);
	else if (Peripheral == DRV_SPI3_I2S3)
		pRCCx->APB1RSTR |= (1 << 15);
	else if (Peripheral == DRV_SPI1)
		pRCCx->APB2RSTR |= (1 << 12);
	else if (Peripheral == DRV_SPI4)
		pRCCx->APB2RSTR |= (1 << 13);
}



// Sending Data
/*********************************************************************
 * @fn      		  - SPI_DataSend
 *
 * @brief             - This function writes to the configured pin from a particular port
 *
 * @param[in]         - base address of the particular SPI peripheral
 * @param[in]         - pointer to the Transmitting Variable
 * @param[in]         - length of the data to be sent
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTX_Buffer, uint32_t length)
{
	while(length > 0)
	{
		// Wait until TX Register is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE));
		// Check for DFF bit in CR1
		if((pSPIx->SPI_CR1) & (0x1 << SPI_CR1_DFF))
		{
			pSPIx->SPI_DR = *((uint16_t*)pTX_Buffer);
			length--;
			length--;
			(uint16_t*)pTX_Buffer++;
		}
		else
		{
			pSPIx->SPI_DR = *((uint8_t*)pTX_Buffer);
			length--;
			(uint8_t*)pTX_Buffer++;
		}
	}
}



/*********************************************************************
 * @fn      		  - SPI_DataReceive
 *
 * @brief             - This function writes to the configured pin from a particular port
 *
 * @param[in]         - base address of the particular SPI peripheral
 * @param[in]         - pointer to the Transmitting Variable
 * @param[in]         - length of the data to be sent
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
// Receiving
void SPI_DataReceive(SPI_RegDef_t *pSPIx, uint8_t *pRX_Buffer, uint32_t length)
{
	while(length > 0)
		{
			// Wait until RX Register is full
			while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE));
			// Check for DFF bit in CR1
			if((pSPIx->SPI_CR1) & (0x1 << SPI_CR1_DFF))
			{
				*((uint16_t*)pRX_Buffer) = pSPIx->SPI_DR;
				length--;
				length--;
				(uint16_t*)pRX_Buffer++;
			}
			else
			{
				*((uint8_t*)pRX_Buffer) = pSPIx->SPI_DR;
				length--;
				(uint8_t*)pRX_Buffer++;
			}
		}
}




/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function fetches the status of the flag
 *
 * @param[in]         - base address of the particular SPI peripheral
 * @param[in]         - Name of the flag
 *
 * @return            -  0 or 1 depending on flag status
 *
 * @Note              -  none

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(!((pSPIx->SPI_SR) & (0x1 << FlagName))) return SPI_PIN_RST;
	else return SPI_PIN_SET;
}






/*
 * Will be implemented if needed later
 */

// IRQ Configuration and ISR Handling

void SPI_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA);																				// ISR Number setup

void SPI_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo);																			// ISR Priority setup


void SPI_IRQHndlr(uint8_t *pHandle);																								// ISR Handler







