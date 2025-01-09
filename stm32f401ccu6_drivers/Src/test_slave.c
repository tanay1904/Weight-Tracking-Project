/*
 * slave_uart_spi.c
 *
 *  Created on: 05-Jan-2025
 *      Author: CITSC
 */

/*
 * SPI1:
 * NSS --> A4
 * SCK --> A5
 * MISO --> A6
 * MOSI --> A7
 *
 * USART2:
 * RX --> A3
 * TX --> A2
 */

#include <stdint.h>
#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"
#include "stm32f401ccu6_spi_drivers.h"
#include "stm32f401ccu6_uart_drivers.h"

void SPI_PinInit(GPIO_Handle_t GPIOPin, GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t AltFn);
void UART_PinInit(GPIO_Handle_t GPIOPin, GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t AltFn);


// String to pass to the sensor
const char sensorsend[] = "S98:MSV?";


#define MASTER_TRIGGER										0x2F																	// Master sends trigger

#define SLAVE1_ACK_TRIG										0x8F																	// Slave acknowledges Master trigger
#define SLAVE2_ACK_TRIG										0xAF																	// Slave acknowledges Master trigger
#define SLAVE3_ACK_TRIG										0xCF																	// Slave acknowledges Master trigger
#define SLAVE4_ACK_TRIG										0xEF																	// Slave acknowledges Master trigger

#define SLAVE1_NACK_TRIG									0x8A																	// Slave responds by "not acknowledge" to master trigger
#define SLAVE2_NACK_TRIG									0xAA																	// Slave responds by "not acknowledge" to master trigger
#define SLAVE3_NACK_TRIG									0xCA																	// Slave responds by "not acknowledge" to master trigger
#define SLAVE4_NACK_TRIG									0xEA																	// Slave responds by "not acknowledge" to master trigger

#define DATA_DMND_SLV										0x4A																	// Demand data from slave by master
#define DUMMY_BYTE											0xEE																	// Dummy byte to receive ack or nack from slave

#define SPI_STP_COMM_CMD									0xFF																	// Stop communication with the slave command

#define SLAVES												0x4																		// Number of slaves used


int main()
{
	/*
	 * initialise the pins
	 */

	// SPI Pins
	GPIO_Handle_t NSS_A4, SCK_A5, MISO_A6, MOSI_A7;
	// UART Pins
	GPIO_Handle_t RX_A3, TX_A2;


	SPI_Handle_t SPI1;
	UART_Handle_t UART2;



	//PERIPHERAL CLOCK
	GPIO_Peri_ClkCtrl(DRV_GPIOA, ENABLE);
	GPIO_Peri_ClkCtrl(DRV_GPIOB, ENABLE);

	// SPI Pin Configurations
	SPI_PinInit(MOSI_A7, DRV_GPIOA, GPIO_PIN_NO_7, GPIO_AF5);
	SPI_PinInit(MISO_A6, DRV_GPIOA, GPIO_PIN_NO_6, GPIO_AF5);
	SPI_PinInit(SCK_A5, DRV_GPIOA, GPIO_PIN_NO_5, GPIO_AF5);
	SPI_PinInit(NSS_A4, DRV_GPIOA, GPIO_PIN_NO_4, GPIO_AF5);

	// UART Pin Configurations
	UART_PinInit(RX_A3, DRV_GPIOA, GPIO_PIN_NO_3, GPIO_AF7);
	UART_PinInit(TX_A2, DRV_GPIOA, GPIO_PIN_NO_2, GPIO_AF7);


	//SPI Setup
	SPI1.pSPIx = DRV_SPI1;
	SPI1.SPI_Config.SPIBusMode = SPI_FULL_DUP;
	SPI1.SPI_Config.SPIDFF = SPI_DATALEN8;
	SPI1.SPI_Config.SPIDevMode = SPI_SLAV_DEV;
	SPI1.SPI_Config.SPISCKSpeed = SPI_SPD_FBY64;
	SPI1.SPI_Config.SPI_CPHA = SPI_LEAD_EDG;
	SPI1.SPI_Config.SPI_CPOL = SPI_POL_LOW;
	SPI1.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI1, 1);

	// UART Setup
	UART2.pUARTx = DRV_USART2;
	UART2.UART_Config.UART_Baud = UART_BAUD_9600;
	UART2.UART_Config.UART_HWFlowCtrlDets = UART_HW_FLOWCTRL_CTSRTS;
	UART2.UART_Config.UART_Mode = UART_MODE_TXRX_EN;
	UART2.UART_Config.UART_ParityControl = UART_PAR_EN_EV;
	UART2.UART_Config.UART_StpBits = UART_STP_Bits1;
	UART2.UART_Config.UART_WordLen = UART_WRD_LEN_9bit;

	UART_Init(&UART2);
	UART_SetBaudRate(UART_RegDef_t *DRV_SPI1, uint32_t 9600);

	while(1)
	{
		// Variable which acknowledges the trigger
		uint8_t initate_Trig;

		// Ack byte to check if the slave is ready to acknowledge
		uint8_t ack = SLAVE1_ACK_TRIG;

		// Variable that stores the ack byte
		uint8_t mas_dem;

		// nominal iterations
		uint8_t nom_iter;

		// Any number of readings can be taken which are initialised to 0, so that readings that are garbage can be eliminated before parsing them to master
		uint32_t sensorRead[8] = 0;

		// Function to receive trigger
		SPI_DataReceive(DRV_SPI1, &initate_Trig, sizeof(initate_Trig));

		// Trigger is checked if it is from the master and then verified if the byte is initiating byte
		if(initate_Trig == MASTER_TRIGGER)
		{

			// Ack bit is sent from the slave
			SPI_DataSend(DRV_SPI1, &ack, sizeof(ack));

			// The master demanding the data from slave is verified and the sensor value acquisition is initiated
			SPI_DataReceive(DRV_SPI1, &mas_dem, sizeof(mas_dem));

			if(mas_dem == DATA_DMND_SLV)
			{
				for (int var = 0; var < nom_iter; ++var)
				{
					// the required word is sent to the sensor
					UART_DataSend(&UART2, (uint8_t*)sensorsend, sizeof(sensorsend));

					// the data is received from the
					UART_DataReceive(&UART2, DRV_USART2, (uint8_t*)sensorRead[var], sizeof(sensorRead));

					// condition for nominal values of iterations
					// use if statements for eliminating garbage sensor values
				}
			}
		}
	}
}
