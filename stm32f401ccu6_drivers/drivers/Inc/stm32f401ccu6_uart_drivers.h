/*
 * stm32f401ccu6_uart_drivers.c
 *
 *  Created on: 05-Jan-2025
 *      Author: CITSC
 */

#ifndef INC_STM32F401CCU6_UART_DRIVERS_C_
#define INC_STM32F401CCU6_UART_DRIVERS_C_



/*
 * UART Status Register MACROS. Offset : 0x00
 */
#define UART_SR_CTS										0x9
#define UART_SR_LBD										0x8
#define UART_SR_TXE										0x7
#define UART_SR_TC											0x6
#define UART_SR_RXNE										0x5
#define UART_SR_IDLE										0x4
#define UART_SR_ORE										0x3
#define UART_SR_NF											0x2
#define UART_SR_FE											0x1
#define UART_SR_PE											0x0

/*
 * UART Data register MACROS. Offset : 0x04
 */
#define UART_DR_DR_8_0										0x0

/*
 * Baud rate register MACROS. Offset : 0x08
 */
#define UART_BRR_DIV_Mantissa_15_4							0x4
#define UART_BRR_DIV_Fraction_3_0							0x0

/*
 * UART Control register 1 MACROS. Offset : 0x0C
 */
#define UART_CR1_OVER8										0xF
#define UART_CR1_UE										0xD
#define UART_CR1_M											0xC
#define UART_CR1_WAKE										0xB
#define UART_CR1_PCE										0xA
#define UART_CR1_PS										0x9
#define UART_CR1_PEIE										0x8
#define UART_CR1_TXEIE										0x7
#define UART_CR1_TCIE										0x6
#define UART_CR1_RXNEIE									0x5
#define UART_CR1_IDLEIE									0x4
#define UART_CR1_TE										0x3
#define UART_CR1_RE										0x2
#define UART_CR1_RWU										0x1
#define UART_CR1_SBK										0x0

/*
 * UART Control register 2 MACROS. Offset : 0x10
 */
#define UART_CR2_ADD_3_0									0x0
#define UART_CR2_LBDL										0x5
#define UART_CR2_LBDIE										0x6
#define UART_CR2_LBCL										0x8
#define UART_CR2_CPHA										0x9
#define UART_CR2_CPOL										0xA
#define UART_CR2_CLKEN										0xB
#define UART_CR2_STOP										0xC
#define UART_CR2_LINEN										0xE

/*
 * UART Control register 3 MACROS. Offset : 0x14
 */

#define UART_CR3_EIE										0x0
#define UART_CR3_IREN										0x1
#define UART_CR3_IRLP										0x2
#define UART_CR3_HDSEL										0x3
#define UART_CR3_NACK										0x4
#define UART_CR3_SCEN										0x5
#define UART_CR3_DMAR										0x6
#define UART_CR3_DMAT										0x7
#define UART_CR3_RTSE										0x8
#define UART_CR3_CTSE										0x9
#define UART_CR3_CTSIE										0xA
#define UART_CR3_ONEBIT									0xB


/*
 * UART Guard time and prescaler register MACROS. Offset : 0x18
 */
#define UART_GTPR_PSC										0x0
#define UART_GTPR_GT										0x8



/*
 * UART Pin Config modes and options
 */

/*
 * UART_Mode Options
 */

#define UART_MODE_TXRX_DI									0x0
#define UART_MODE_RXONLY									0x1
#define UART_MODE_TXONLY									0x2
#define UART_MODE_TXRX_EN									0x3

/*
 * UART Baud Rate Options
 */

#define UART_BAUD_1200										(uint16_t)1200
#define UART_BAUD_2400										(uint16_t)2400
#define UART_BAUD_9600										(uint16_t)9600
#define UART_BAUD_19200										(uint16_t)19200
#define UART_BAUD_38400										(uint16_t)38400
#define UART_BAUD_57600										(uint16_t)57600
#define UART_BAUD_115200									(uint16_t)115200
#define UART_BAUD_230400									(uint16_t)230400
#define UART_BAUD_460800									(uint16_t)460800
#define UART_BAUD_921600									(uint16_t)921600
#define UART_BAUD_1M										(uint16_t)1000000
#define UART_BAUD_2M										(uint16_t)2000000


/*
 * UART_StpBits
 */

#define UART_STP_Bits1										0x0
#define UART_STP_Bits0_5									0x1
#define UART_STP_bits2										0x2
#define UART_STP_Bits1_5									0x3


/*
 * UART_WordLen
 */

#define UART_WRD_LEN_8bit									0x0
#define UART_WRD_LEN_9bit									0x1

/*
 * UART_ParityControl
 */

#define UART_PAR_DI										0x0
#define UART_PAR_EN_EV										0x2
#define UART_PAR_EN_OD										0x3

/*
 * UART_HWFlowCtrlDets
 */

#define UART_HW_FLOWCTRL_NONE								0x0
#define UART_HW_FLOWCTRL_RTS								0x1
#define UART_HW_FLOWCTRL_CTS								0x2
#define UART_HW_FLOWCTRL_CTSRTS								0x3


typedef struct{
	uint8_t UART_Mode;
	uint32_t UART_Baud;
	uint8_t UART_StpBits;
	uint8_t UART_WordLen;
	uint8_t UART_ParityControl;
	uint8_t UART_HWFlowCtrlDets;
}UART_PinConf_t;



typedef struct{
	UART_RegDef_t *pUARTx;
	UART_PinConf_t UART_Config;
}UART_Handle_t;


//Peripheral Clock Setup
void UART_Peri_ClkCtrl(UART_RegDef_t *pUARTx, uint8_t UARTCLKEN);																	// Clock Control of Peripheral


//Initialisation
void UART_Init(UART_Handle_t *pUARTHandle);																							// Initialise SPI
void UART_DInit(UART_RegDef_t *Peripheral);																							// De-Initialise SPI


// Get flag Status
uint8_t UART_GetFlgStat(UART_RegDef_t *pUART, uint16_t StatuesFlagName);															// Function for Getting flag status of UART

// Clear Flag Status

void UART_ClrFlag(UART_RegDef_t *pUART, uint8_t Reg,uint16_t ClrFlag);																// Function for Clearing flag status of UART



// Sending Data
void UART_DataSend(UART_Handle_t *pUARTHan, uint8_t *pTX_Buffer, uint32_t length);


// Receiving
void UART_DataReceive(UART_Handle_t *pUARTHan, UART_RegDef_t *pUARTx, uint8_t *pRX_Buffer, uint32_t length);


// Baud rate configuration
void UART_SetBaudRate(UART_RegDef_t *pUARTx, uint32_t BaudRate);


/*
 * Will be implemented if needed
 */

// IRQ Configuration and ISR Handling

void UART_IRQIntrptConf(uint8_t IRQNo, uint8_t IRQENA);																				// ISR Number setup
void UART_IRQPriConf(uint32_t IRQPriority, uint8_t IRQNo);																			// ISR Priority setup
void UART_IRQHndlr(uint8_t *pHandle);																								// ISR Handler





#endif /* INC_STM32F401CCU6_UART_DRIVERS_C_ */
