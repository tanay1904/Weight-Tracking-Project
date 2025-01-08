/*
 * stm32f401ccu6_uart_drivers.c
 *
 *  Created on: 05-Jan-2025
 *      Author: CITSC
 */

#include <stdint.h>
#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"
#include "stm32f401ccu6_uart_drivers.h"

// Pin Initialisation
void UART_PinInit(GPIO_Handle_t UARTPin, GPIO_RegDef_t *pUARTx, uint8_t PinNo, uint8_t AltFn);

#define UART_CR1_REG										0x0
#define UART_CR2_REG										0x1
#define UART_CR3_REG										0x2
#define UART_GTPR_REG										0x3

/*********************************************************************
 * @fn      		  - UART_Peri_ClkCtrl
 *
 * @brief             - This function turns on the peripheral clock.
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
void UART_Peri_ClkCtrl(UART_RegDef_t *pUARTx, uint8_t UARTCLKEN)																	// Clock Control of Peripheral
{
	switch ((uint32_t) pUARTx) {
		case (uint32_t)DRV_USART1:
			USART1_PCLK_ENA();
			break;
		case (uint32_t)DRV_USART2:
			USART2_PCLK_ENA();
			break;
		case (uint32_t)DRV_USART6:
			USART6_PCLK_ENA();
			break;
		default:
			break;
	}
}

/*********************************************************************
 * @fn      		  - UART_Init
 *
 * @brief             - This function initialises the Peripheral according to the Pin configuration provided
 *
 *
 * @param[in]         - base address of the particular SPI Handle structure
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */

//Initialisation
void UART_Init(UART_Handle_t *pUARTHandle)																						// Initialise SPI
{
	UART_Peri_ClkCtrl(pUARTHandle->pUARTx, ENABLE);

	pUARTHandle->pUARTx->UART_CR1 &= ~(1 << UART_CR1_UE);
	pUARTHandle->pUARTx->UART_CR1 |= (1 << UART_CR1_UE);

	pUARTHandle->pUARTx->UART_CR1 &= ~(1 << UART_CR1_M);
	pUARTHandle->pUARTx->UART_CR1 |= (pUARTHandle->UART_Config.UART_WordLen << UART_CR1_M);
	// Set Desired baud rate

	pUARTHandle->pUARTx->UART_CR1 &= ~(1 << UART_CR1_RE);
	pUARTHandle->pUARTx->UART_CR1 |= (pUARTHandle->UART_Config.UART_Mode << UART_CR1_RE);

	pUARTHandle->pUARTx->UART_CR2 &= ~(0x3 << UART_CR2_STOP);
	pUARTHandle->pUARTx->UART_CR2 |= (pUARTHandle->UART_Config.UART_StpBits << UART_CR2_STOP);

	pUARTHandle->pUARTx->UART_CR2 &= ~(0x3 << UART_CR1_PS);
	pUARTHandle->pUARTx->UART_CR2 &= ~(pUARTHandle->UART_Config.UART_ParityControl << UART_CR1_PS);

	pUARTHandle->pUARTx->UART_CR2 &= ~(0x3 << UART_CR1_PS);
	pUARTHandle->pUARTx->UART_CR2 |= (pUARTHandle->UART_Config.UART_HWFlowCtrlDets << UART_CR3_RTSE);

	pUARTHandle->pUARTx->UART_CR1 &= ~(1 << UART_CR1_RE);
	pUARTHandle->pUARTx->UART_CR1 |= (pUARTHandle->UART_Config.UART_Mode << UART_CR1_OVER8);
}

/*********************************************************************
 * @fn      		  -  UART_DInit
 *
 *
 * @param[in]         -  base address of the particular GPIO Register Definition Structure
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
void UART_DInit(UART_RegDef_t *Peripheral)																							// De-Initialise SPI
{
	RCC_RegDef_t *pRCCx = DRV_RCC;
	if(Peripheral == DRV_USART2)
		pRCCx->APB1RSTR |= (1 << 4);
	else if (Peripheral == DRV_USART2)
		pRCCx->APB1RSTR |= (1 << 17);
	else if (Peripheral == DRV_USART6)
		pRCCx->APB2RSTR |= (1 << 5);
}


/*********************************************************************
 * @fn      		  - UART_PinInit
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
void UART_PinInit(GPIO_Handle_t UARTPin, GPIO_RegDef_t *pUARTx, uint8_t PinNo, uint8_t AltFn)										// Initialise GPIO pin to alt function
{
	UARTPin.pGPIOBasAddr = pUARTx;
	UARTPin.GPIO_PinConf.GPIO_PinNo = PinNo;
	UARTPin.GPIO_PinConf.GPIO_PinMode = GPIOALTFN;
	UARTPin.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_MED;
	UARTPin.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
	UARTPin.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	UARTPin.GPIO_PinConf.GPIO_PinAltFunMode = AltFn;
	GPIO_Init(&UARTPin);
}


/*********************************************************************
 * @fn      		  - UART_GetFlgStat
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
// Get flag Status

uint8_t UART_GetFlgStat(UART_RegDef_t *pUART, uint16_t StatuesFlagName)															// Function for Getting flag status of UART
{
	uint8_t Flg = (pUART->UART_SR & (1 << StatuesFlagName));
	return Flg;
}

/*****************************************************************************
 * @fn      		  - UART_ClrFlag
 *
 * @brief             - This function clears the flag in the Control Registers
 *
 * @param[in]         - base address of the particular SPI peripheral
 * @param[in]         - Control Register(integer)
 * @param[in]         - length of the data to be sent
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
// Clear Flag Status

void UART_ClrFlag(UART_RegDef_t *pUART, uint8_t Reg, uint16_t ClrFlag)																// Function for Clearing flag status of UART
{
	if(Reg == UART_CR1_REG)
		pUART->UART_CR1 &= ~(1 << ClrFlag);
	else if (Reg == UART_CR2_REG)
		pUART->UART_CR2 &= ~(1 << ClrFlag);
	else if (Reg == UART_CR3_REG)
		pUART->UART_CR3 &= ~(1 << ClrFlag);
	else if (Reg == UART_GTPR_REG)
		pUART->UART_GTPR &= ~(0xFF << ClrFlag);
}

/****************************************************************************************
 * @fn      		  - UART_DataSend
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
// Sending Data
void UART_DataSend(UART_Handle_t *pUARTHan, uint8_t *pTX_Buffer, uint32_t length)
{
	uint16_t *pdata;

	for (uint32_t var = 0; var < length; ++var) {

		// wait until TXE flag is enabled

		while(! UART_GetFlgStat(pUARTHan->pUARTx, UART_SR_TXE));

		if (pUARTHan->UART_Config.UART_WordLen == UART_WRD_LEN_9bit)
		{

			pdata = (uint16_t*)pTX_Buffer;
			pUARTHan->pUARTx->UART_DR = (*pdata & (uint16_t)0x01FF);

			if(pUARTHan->UART_Config.UART_ParityControl == UART_PAR_DI)
			{
				// no parity is used, sending 9 bits of data
				// Code to increment the pointer to next memory location
				pTX_Buffer += 2;
			}

			else
			{
				// parity bit makes the data 8 bit
				pTX_Buffer++;
			}
		}
		else
			if(pUARTHan->UART_Config.UART_ParityControl == UART_PAR_DI)
			{
			// 8 bit data transfer
			pUARTHan->pUARTx->UART_DR = (*pTX_Buffer & (uint8_t)0xFF);

			// Increase pointer to change to next memory location
			pTX_Buffer++;
			}
			else
			{
				// 8 bit data transfer
				pUARTHan->pUARTx->UART_DR = (*pTX_Buffer & (uint8_t)0x7F);

				// Increase pointer to change to next memory location
				pTX_Buffer++;
			}
	}
	// wait till transmission is complete via TC flag of SR
	while(!(UART_GetFlgStat(pUARTHan->pUARTx, UART_SR_TC)));
}


/*********************************************************************
 * @fn      		  - UART_DataReceive
 *
 * @brief             - This function writes to the configured pin from a particular port
 *
 * @param[in]         -  base address of the particular SPI Handle Variable
 * @param[in]         -  base address of the particular SPI peripheral
 * @param[in]         -  pointer to the Transmitting Variable
 * @param[in]         -  length of the data to be sent
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */
// Receiving
void UART_DataReceive(UART_Handle_t *pUARTHan, UART_RegDef_t *pUARTx, uint8_t *pRX_Buffer, uint32_t length)
{
   //Loop over until "length" number of bits are transferred
	for(uint32_t i = 0 ; i < length; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! UART_GetFlgStat(pUARTHan->pUARTx,UART_SR_RXNE));

		//Check the UART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUARTHan->UART_Config.UART_WordLen == UART_WRD_LEN_9bit)
		{
			//We are going to receive 9bit data in a frame

			//Now, check are we using UART_ParityControl control or not
			if(pUARTHan->UART_Config.UART_ParityControl == UART_PAR_DI)
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRX_Buffer = (pUARTHan->pUARTx->UART_DR  & (uint8_t)0xFF);
				 pRX_Buffer++;

			}
			else
			{
				//No parity is used , so all 9bits will be of user data
				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRX_Buffer) = (pUARTHan->pUARTx->UART_DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRX_Buffer++;
				pRX_Buffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//Now, check are we using UART_ParityControl control or not
			if(pUARTHan->UART_Config.UART_ParityControl == UART_PAR_DI)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRX_Buffer = (uint8_t) (pUARTHan->pUARTx->UART_DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRX_Buffer = (uint8_t) (pUARTHan->pUARTx->UART_DR  & (uint8_t)0x7F);
			}

			//Now , increment the pRxBuffer
			pRX_Buffer++;
		}
	}

}


// IRQ Configuration and ISR Handling

void UART_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA);																			// ISR Number setup
void UART_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo);																			// ISR Priority setup
void UART_IRQHndlr(uint8_t *pHandle);																								// ISR Handler

/***********************
 * @fn      		  -  UART_SetBaudRate
 *
 * @brief             -	 This function sets the baud rate for the transmission
 *
 * @param[in]         -  UART peripheral address
 * @param[in]         -  Baudrate Macros
 *
 * @return            -  none
 *
 * @Note              -  Only can be used for particular macros defined. Needs tweaking to find the actual frequency of the APB 2 in variable scenarios

 */
void UART_SetBaudRate(UART_RegDef_t *pUARTx, uint32_t BaudRate)
{

		//Variable to hold the APB clock
		uint32_t PCLKx;

		uint32_t UARTdiv;

		//variables to hold Mantissa and Fraction values
		uint32_t M_part,F_part;

		uint32_t tempreg=0;

		  //Get the value of APB bus clock in to the variable PCLKx
		if(pUARTx == DRV_USART1 || pUARTx == DRV_USART6)
		{
			   //UART1 and UART6 are hanging on APB2 bus
			   PCLKx = 16000000;
		}
		else
		{
			PCLKx = 16000000;
		}

		//Check for OVER8 configuration bit
		if(pUARTx->UART_CR1 & (1 << UART_CR1_OVER8))
		{
			//OVER8 = 1 , over sampling by 8
			UARTdiv = ((25 * PCLKx) / (2 * BaudRate));
		}else
		{
			//over sampling by 16
			UARTdiv = ((25 * PCLKx) / (4 * BaudRate));
		}

		//Calculate the Mantissa part
		M_part = UARTdiv/100;

		//Place the Mantissa part in appropriate bit position . refer UART_BRR
		tempreg |= M_part << 4;

		//Extract the fraction part
		F_part = (UARTdiv - (M_part * 100));

		//Calculate the final fractional
		if(pUARTx->UART_CR1 & ( 1 << UART_CR1_OVER8))
		{
			//OVER8 = 1 , over sampling by 8
			F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

		}else
		{
			//over sampling by 16
			F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
		}

		//Place the fractional part in appropriate bit position . refer UART_BRR
		tempreg |= F_part;

		//copy the value of tempreg in to BRR register
		pUARTx->UART_BRR = tempreg;
}
