/*
 * master_spi.c
 *
 *  Created on: 03-Jan-2025
 *      Author: CITSC
 */
#include <stdio.h>
#include <stdint.h>
#include "stm32f401ccu6.h"
#include "stm32f401ccu6_gpio_drivers.h"
#include "stm32f401ccu6_spi_drivers.h"

/*
 * SPI Pins:
 * MOSI --> A7
 * MISO --> A6
 * SCK --> A5
 *
 * UART Pins:
 * TX --> A9
 * RX --> A10
 *
 * GPIO:
 * Button --> A0
 *
 * NSS for Slaves:
 * Slave1 : B4
 * Slave2 : B5
 * Slave3 : B6
 * Slave4 : B7
 *
 */
#define SLAVE_ENABLE										0x0																		//Takes the NSS Pin low making the slave active
#define SLAVE_DISABLE										0x1																		// Takes the NSS Pin High making the slave inactive

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


/*
 * Global Variables Used
 */


uint8_t initate_Trig = MASTER_TRIGGER;

uint8_t SlaveAck;

uint8_t Data_Trig = DATA_DMND_SLV;

uint16_t Slav_weight[4];

uint8_t stopcmd;

uint16_t Average_Weight = 0;

/*
 * Used function definitions
 */
void ReqSlave(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, SPI_RegDef_t *pSPIx, uint8_t slav_ack);
void RecvSlvData_closetx (GPIO_RegDef_t *pGPIOx, uint8_t PinNo, SPI_RegDef_t *pSPIx, uint8_t slv_no);
void SPI_PinInit(GPIO_Handle_t GPIOPin, GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t AltFn);


int main(void)
{
	/*
	 * Initialisation of Pins
	*/

	// SPI Pins
	GPIO_Handle_t MOSI_A7;
	GPIO_Handle_t MISO_A6;
	GPIO_Handle_t SCK_A5;
	SPI_Handle_t SPI1;

	// UART Pins

	// GPIO NSS Slave Pins
	GPIO_Handle_t Slave1_B4, Slave2_B5, Slave3_B6, Slave4_B7;

	// GPIO Button
	GPIO_Handle_t Button_A0;

	//PERIPHERAL CLOCK
	GPIO_Peri_ClkCtrl(DRV_GPIOA, ENABLE);
	GPIO_Peri_ClkCtrl(DRV_GPIOB, ENABLE);

	// Pin Configurations
	SPI_PinInit(MOSI_A7, DRV_GPIOA, GPIO_PIN_NO_7, GPIO_AF5);
	SPI_PinInit(MISO_A6, DRV_GPIOA, GPIO_PIN_NO_6, GPIO_AF5);
	SPI_PinInit(SCK_A5, DRV_GPIOA, GPIO_PIN_NO_5, GPIO_AF5);



	//SPI Setup
	SPI1.pSPIx = DRV_SPI1;
	SPI1.SPI_Config.SPIBusMode = SPI_FULL_DUP;
	SPI1.SPI_Config.SPIDFF = SPI_DATALEN8;
	SPI1.SPI_Config.SPIDevMode = SPI_MAST_DEV;
	SPI1.SPI_Config.SPISCKSpeed = SPI_SPD_FBY64;
	SPI1.SPI_Config.SPI_CPHA = SPI_LEAD_EDG;
	SPI1.SPI_Config.SPI_CPOL = SPI_POL_LOW;
	SPI1.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI1, 1);

	Button_A0.pGPIOBasAddr = DRV_GPIOA;
	Button_A0.GPIO_PinConf.GPIO_PinMode = GPIOINPUT;
	Button_A0.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_0;
	Button_A0.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	Button_A0.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
	Button_A0.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	Slave1_B4.pGPIOBasAddr = DRV_GPIOB;
	Slave1_B4.GPIO_PinConf.GPIO_PinMode = GPIOOUTPUT;
	Slave1_B4.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_4;
	Slave1_B4.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	Slave1_B4.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
	Slave1_B4.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	//
	Slave2_B5.pGPIOBasAddr = DRV_GPIOB;
	Slave2_B5.GPIO_PinConf.GPIO_PinMode = GPIOOUTPUT;
	Slave2_B5.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_5;
	Slave2_B5.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	Slave2_B5.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
	Slave2_B5.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	//
	Slave3_B6.pGPIOBasAddr = DRV_GPIOB;
	Slave3_B6.GPIO_PinConf.GPIO_PinMode = GPIOOUTPUT;
	Slave3_B6.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_6;
	Slave3_B6.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	Slave3_B6.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
	Slave3_B6.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	Slave4_B7.pGPIOBasAddr = DRV_GPIOB;
	Slave4_B7.GPIO_PinConf.GPIO_PinMode = GPIOOUTPUT;
	Slave4_B7.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_7;
	Slave4_B7.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	Slave4_B7.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
	Slave4_B7.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(&Button_A0);
	GPIO_Init(&Slave1_B4);
	GPIO_Init(&Slave2_B5);
	GPIO_Init(&Slave3_B6);
	GPIO_Init(&Slave4_B7);

	// Initialise the slaves disabled at first(active low)
	uint8_t buttonStatReg = 0x0;
	GPIO_WriteOPin(DRV_GPIOB, GPIO_PIN_NO_4, 1);
	GPIO_WriteOPin(DRV_GPIOB, GPIO_PIN_NO_5, 1);
	GPIO_WriteOPin(DRV_GPIOB, GPIO_PIN_NO_6, 1);
	GPIO_WriteOPin(DRV_GPIOB, GPIO_PIN_NO_7, 1);
	// infinite looping
	for(;;)
	{
	// Read the trigger from button
	uint8_t buttonRead = GPIO_ReadIPin(DRV_GPIOA, GPIO_PIN_NO_0);
	delay(600);
		if(buttonRead == 0)
		{
			buttonStatReg = ~buttonStatReg;
		}
		if(buttonStatReg)
		{
			/*
			 * Section 1: From initial Trigger to Data Request Trigger
			 */

			ReqSlave(DRV_GPIOB, GPIO_PIN_NO_4, DRV_SPI1, SLAVE1_ACK_TRIG);
			ReqSlave(DRV_GPIOB, GPIO_PIN_NO_5, DRV_SPI1, SLAVE2_ACK_TRIG);
			ReqSlave(DRV_GPIOB, GPIO_PIN_NO_6, DRV_SPI1, SLAVE3_ACK_TRIG);
			ReqSlave(DRV_GPIOB, GPIO_PIN_NO_7, DRV_SPI1, SLAVE4_ACK_TRIG);

			/*
			 * Data Receive trigger resulting in transmission of data by slave
			 */

			RecvSlvData_closetx(DRV_GPIOB, GPIO_PIN_NO_4, DRV_SPI1, 1);
			RecvSlvData_closetx(DRV_GPIOB, GPIO_PIN_NO_5, DRV_SPI1, 2);
			RecvSlvData_closetx(DRV_GPIOB, GPIO_PIN_NO_6, DRV_SPI1, 3);
			RecvSlvData_closetx(DRV_GPIOB, GPIO_PIN_NO_7, DRV_SPI1, 4);


			/*
			 * Compilation and averaging the data for the weight
			 */
			for (int var = 0; var < SLAVES; ++var)
			{
				Average_Weight += Slav_weight[var];
			}
			Average_Weight /= SLAVES;
		}
	}
}




void ReqSlave(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, SPI_RegDef_t *pSPIx, uint8_t slav_ack)
{
	// Set NSS low to activate slave
	GPIO_WriteOPin(pGPIOx, PinNo, SLAVE_ENABLE);

	// Send Trigger
	SPI_DataSend(pSPIx, &initate_Trig, sizeof(initate_Trig));

	// dummy read to remove contents of slave RX
	uint8_t dummyread;
	SPI_DataReceive(pSPIx, &dummyread, sizeof(dummyread));

	// Send a dummy byte to receive ack or nack from slave
	uint8_t dummyWrite = DUMMY_BYTE;
	SPI_DataSend(pSPIx, &dummyWrite, sizeof(dummyWrite));

	// Receive Data from slave
	SPI_DataReceive(DRV_SPI1, &SlaveAck, sizeof(SlaveAck));

	// Check for ACK byte
	if((SlaveAck == SLAVE1_ACK_TRIG))
	{
		// Send Trigger to request data
		SPI_DataSend(pSPIx, &Data_Trig, sizeof(Data_Trig));

		// Receive dummy read of data request trigger
		SPI_DataReceive(pSPIx, &dummyread, sizeof(dummyread));

		uint8_t temp = ((pSPIx->SPI_SR) & (0x1 << SPI_SR_BSY));

		// wait until Transmission is over
		while(temp == 0);
	}

	// Disable slave input to give it time to request data from sensor and process it
	GPIO_WriteOPin(pGPIOx, PinNo, GPIO_PIN_SET);
}


void RecvSlvData_closetx (GPIO_RegDef_t *pGPIOx, uint8_t PinNo, SPI_RegDef_t *pSPIx, uint8_t slv_no)
{
	// Enable slave and receive data
	GPIO_WriteOPin(pGPIOx, GPIO_PIN_NO_4, SLAVE_ENABLE);

	// Send dummy data to receive the weight acquired by slave
	uint16_t DummyWrite[4];
	SPI_DataSend(pSPIx, (uint8_t*)(DummyWrite- 1 + slv_no), sizeof(DummyWrite[slv_no - 1]));

	// Receive data from the slave
	SPI_DataReceive(pSPIx, (uint8_t*)(Slav_weight- 1 + slv_no), sizeof(Slav_weight[slv_no - 1]));

	// If the transmission is complete end communication
	while(!(pSPIx->SPI_SR & (0x1 << SPI_SR_BSY)));
	GPIO_WriteOPin(pGPIOx, GPIO_PIN_NO_4, GPIO_PIN_SET);
}


/*
	MOSI_A7.pGPIOBasAddr = DRV_GPIOA;
	MOSI_A7.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_5;
	MOSI_A7.GPIO_PinConf.GPIO_PinMode = GPIO_PIN_NO_7;
	MOSI_A7.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_MED;
	MOSI_A7.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
	MOSI_A7.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	MOSI_A7.GPIO_PinConf.GPIO_PinAltFunMode = GPIO_AF5;
	GPIO_Init(&MOSI_A7);

	MISO_A6.pGPIOBasAddr = DRV_GPIOA;
	MISO_A6.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_6;
	MISO_A6.GPIO_PinConf.GPIO_PinMode = GPIOALTFN;
	MISO_A6.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_MED;
	MISO_A6.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
	MISO_A6.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	MISO_A6.GPIO_PinConf.GPIO_PinAltFunMode = GPIO_AF5;
	GPIO_Init(&MISO_A6);

	SCK_A5.pGPIOBasAddr = DRV_GPIOA;
	SCK_A5.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_5;
	SCK_A5.GPIO_PinConf.GPIO_PinMode = GPIOALTFN;
	SCK_A5.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_MED;
	SCK_A5.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
	SCK_A5.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
	SCK_A5.GPIO_PinConf.GPIO_PinAltFunMode = GPIO_AF5;
	GPIO_Init(&SCK_A5);
*/
