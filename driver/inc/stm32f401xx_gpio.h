/*
 * stm32f401xx_gpio.h
 *
 *  Created on: 05-May-2021
 *      Author: supratim
 */

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_

#include "stm32f401xx.h"

/*structure to hold GPIO pin configuration*/
typedef struct {
	uint8_t			GPIO_PinNumber;
	uint8_t			GPIO_PinMode;
	uint8_t			GPIO_PinOutType;
	uint8_t			GPIO_PinSpeed;
	uint8_t			GPIO_PinPuPdConfig;
	uint8_t			GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

/*Structure for a GPIO handle*/
typedef struct {
	GPIO_RegDef_t 		*pGPIOx;
	GPIO_PinConfig_t	config;
}GPIO_Handle_t;

/*Driver API Function prototypes*/
void 		GPIO_Init(GPIO_Handle_t* gpioHandle);
void		GPIO_IRQ_Config(GPIO_Handle_t* gpioHandle, uint8_t en);
void		GPIO_IRQ_Priority(GPIO_Handle_t* gpioHandle, uint8_t Priority);
uint8_t 	GPIO_Read_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin);
uint16_t 	GPIO_Read_Port(GPIO_Handle_t* gpioHandle);
void 		GPIO_Write_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin, uint8_t value);
void 		GPIO_Write_Port(GPIO_Handle_t* gpioHandle, uint16_t value);
void 		GPIO_Toggle_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin);
void		GPIO_Reset(GPIO_Handle_t* gpioHandle);
void		GPIO_IRQ_Handler(uint8_t pinNumber);

#endif /* INC_STM32F401XX_GPIO_H_ */
