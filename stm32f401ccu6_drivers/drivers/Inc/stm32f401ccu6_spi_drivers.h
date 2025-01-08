/*
 * stm32f401ccu6_spi_drivers.h
 *
 *  Created on: 01-Jan-2025
 *      Author: CITSC
 */

#ifndef INC_STM32F401CCU6_SPI_DRIVERS_H_
#define INC_STM32F401CCU6_SPI_DRIVERS_H_


#include <stdint.h>
#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"

// Forward declaration typedef struct GPIO_Handle_t GPIO_Handle_t;

typedef struct{
	uint8_t SPIDevMode;																												//Master or Slave
	uint8_t SPIBusMode;																												// Duplex Mode
	uint8_t SPISCKSpeed;																											// Slave clock speed
	uint8_t SPIDFF;																													// Data Length (8 or 16 bits)
	uint8_t SPI_CPOL;																												// Clock polarity (0 - 0 is default or 1 - 1 is default)
	uint8_t SPI_CPHA;																												// Rising edge or falling else
	uint8_t SPI_SSM;																												// Software enable
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

/*
 * SPI Related MACROS
 */

// MASTER SLAVE Configuration
#define SPI_MAST_DEV										0x1																		// Master device configuration
#define SPI_SLAV_DEV										0x0																		// Slave device configuration

// Duplex modes
#define SPI_FULL_DUP										0x0																		// Full Duplex Mode
#define SPI_HALF_DUP										0x1																		// Half Duplex Mode
#define SPI_MAS_RXS											0x2																		// Simplex Receive only(Master)
#define SPI_MAS_TXS											0x3																		// Simplex Transmit only(Master)
#define SPI_SLV_TXS											0x4																		// Simplex Transmit only(Slave)
#define SPI_SLV_RXS											0x5																		// Simplex Receive only (Slave)

// Slave clock speed fP/
#define SPI_SPD_FBY2										0x0																		// APB frequency divided by 2
#define SPI_SPD_FBY4										0x1																		// APB frequency divided by 4
#define SPI_SPD_FBY8										0x2																		// APB frequency divided by 8
#define SPI_SPD_FBY16										0x3																		// APB frequency divided by 16
#define SPI_SPD_FBY32										0x4																		// APB frequency divided by 32
#define SPI_SPD_FBY64										0x5																		// APB frequency divided by 64
#define SPI_SPD_FBY128										0x6																		// APB frequency divided by 128
#define SPI_SPD_FBY256										0x7																		// APB frequency divided by 256

// Data length
#define SPI_DATALEN8										0x0																		// 8 bit data length
#define SPI_DATALEN16										0x1																		// 16 bit data length

// Clock polarity of default state
#define SPI_POL_LOW											0x0																		// 0 is default
#define SPI_POL_HIGH										0x1																		// 1 is default

// Clock phase
#define SPI_LEAD_EDG										0x1																		// 1 is for leading edge
#define SPI_LAG_EDGE										0x0																		// 0 is for lagging edge

// Software slave management
#define SPI_SSM_EN											0x1																		// Software slave management
#define SPI_SSM_DI											0x0																		// Hardware slave management


/*
 * MACROS for Bit field positions of SPI Control Register 1
 */
#define SPI_CR1_BIDIMODE									15																		// CR1 Register 15th bit position
#define SPI_CR1_BIDIOE										14																		// CR1 Register 15th bit position
#define SPI_CR1_CRCEN										13																		// CR1 Register 15th bit position
#define SPI_CR1_CRCNEXT										12																		// CR1 Register 15th bit position
#define SPI_CR1_DFF											11																		// CR1 Register 15th bit position
#define SPI_CR1_RXONLY										10																		// CR1 Register 15th bit position
#define SPI_CR1_SSM											9																		// CR1 Register 15th bit position
#define SPI_CR1_SSI											8																		// CR1 Register 15th bit position
#define SPI_CR1_LSBFIRST									7																		// CR1 Register 15th bit position
#define SPI_CR1_SPE											6																		// CR1 Register 15th bit position
#define SPI_CR1_BR											3																		// CR1 Register 15th bit position
#define SPI_CR1_MSTR										2																		// CR1 Register 15th bit position
#define SPI_CR1_CPOL										1																		// CR1 Register 15th bit position
#define SPI_CR1_CPHA										0																		// CR1 Register 15th bit position

/*
 * MACROS for Bit field positions of SPI Control Register 2
 */
#define SPI_CR2_TXEIE										7																		// CR2 Register 7th bit position
#define SPI_CR2_RXNEIE										6																		// CR2 Register 6th bit position
#define SPI_CR2_ERRIE										5																		// CR2 Register 5th bit position
#define SPI_CR2_FRF											4																		// CR2 Register 4th bit position
#define SPI_CR2_SSOE										2																		// CR2 Register 2nd bit position
#define SPI_CR2_TXDMAEN										1																		// CR2 Register 1st bit position
#define SPI_CR2_RXDMAEN										0																		// CR2 Register 0th bit position


/*
 * MACROS for Bit field positions of SPI Status Register
 */
#define SPI_SR_FRE											4																		// CR2 Register 4th bit position
#define SPI_SR_BSY											4																		// CR2 Register 4th bit position
#define SPI_SR_OVR											4																		// CR2 Register 4th bit position
#define SPI_SR_MODF											4																		// CR2 Register 4th bit position
#define SPI_SR_CRCERR										4																		// CR2 Register 4th bit position
#define SPI_SR_UDR											4																		// CR2 Register 4th bit position
#define SPI_SR_CHSIDE										4																		// CR2 Register 4th bit position
#define SPI_SR_TXE											4																		// CR2 Register 4th bit position
#define SPI_SR_RXNE											4																		// CR2 Register 4th bit position

/*
 *
 */



//Peripheral Clock Setup
void SPI_Peri_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t SPICLKEN);																		//Clock Control of Peripheral


//Initialisation
void SPI_Init(SPI_Handle_t *pSPIHandle, uint8_t SSI);																				//Initialise SPI

//De-initialisation
void SPI_DInit(SPI_RegDef_t *Peripheral);																							//De-Initialise SPI

// Sending Data
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTX_Buffer, uint32_t length);

// Receiving
void SPI_DataReceive(SPI_RegDef_t *pSPIx, uint8_t *pRX_Buffer, uint32_t length);





/*
 * Will be implemented if needed
 */
// IRQ Configuration and ISR Handling
void SPI_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA);																				// ISR Number setup
void SPI_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo);																			// ISR Priority setup
void SPI_IRQHndlr(uint8_t *pHandle);																								// ISR Handler


#endif /* INC_STM32F401CCU6_SPI_DRIVERS_H_ */

