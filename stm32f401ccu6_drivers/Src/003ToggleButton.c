///*
// * 002ToggleButton.c
// *
// *  Created on: 30-Dec-2024
// *      Author: CITSC
// */
//
//#include <stdint.h>
//#include "stm32f401ccu6.h"
//#include "stm32f401ccu6_gpio_drivers.h"
//
//int main(){
//
//	/*
//	 * Setup of pins
//	 */
//	GPIO_Handle_t button;
//	GPIO_Handle_t led;
//
//	// Peripheral base addresses
//	button.pGPIOBasAddr = DRV_GPIOA;
//	led.pGPIOBasAddr = DRV_GPIOC;
//
//	// Button pin configuration
//	button.GPIO_PinConf.GPIO_PinMode = GPIOINPUT;
//	button.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_0;
//	button.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
//	button.GPIO_PinConf.GPIO_PinPuPdCtrl = PULLUP;
//	button.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//
//	// LED pin configuration
//	led.GPIO_PinConf.GPIO_PinMode = GPIOOUTPUT;
//	led.GPIO_PinConf.GPIO_PinNo = GPIO_PIN_NO_13;
//	led.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PUPD;
//	led.GPIO_PinConf.GPIO_PinPuPdCtrl = NOPUPD;
//	led.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//
//	// initialise pins
//	GPIO_Peri_ClkCtrl(DRV_GPIOA, ENABLE);
//	GPIO_Peri_ClkCtrl(DRV_GPIOC, ENABLE);
//	GPIO_Init(&led);
//	GPIO_Init(&button);
//
//	for(;;){
//		uint8_t tempbut = GPIO_ReadIPin(DRV_GPIOA, GPIO_PIN_NO_0);
//		delay(250);
//		if(tempbut == DISABLE)
//			GPIO_ToggleOPin(DRV_GPIOC, GPIO_PIN_NO_13);
//	}
//}
