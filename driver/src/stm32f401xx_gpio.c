/*
 * stm32f4xx_gpio.c
 *
 *  Created on: 05-May-2021
 *      Author: supratim
 */

#include "stm32f401xx_gpio.h"

/**
 * @brief GPIO Peripheral clock control. Enables/Disables GPIO Peripheral Clock
 *
 * @param GPIO Handle pointer
 * @param en/dis
 *
 * @return void
 */
void	GPIO_Peripheral_Clk_Ctrl(GPIO_Handle_t *gpioHandle, uint8_t en) {
	GPIO_RegDef_t *pGPIOx = gpioHandle->pGPIOx;
	if(pGPIOx == GPIOA) {
		if(en) {
			GPIOA_PCLK_EN();
		}else{
			GPIOA_PCLK_DIS();
		}
	}else if(pGPIOx == GPIOB) {
		if(en) {
			GPIOB_PCLK_EN();
		}else{
			GPIOB_PCLK_DIS();
		}
	}else if(pGPIOx == GPIOC) {
		if(en) {
			GPIOC_PCLK_EN();
		}else{
			GPIOC_PCLK_DIS();
		}
	}else if(pGPIOx == GPIOD) {
		if(en) {
			GPIOD_PCLK_EN();
		}else{
			GPIOD_PCLK_DIS();
		}
	}else if(pGPIOx == GPIOE) {
		if(en) {
			GPIOE_PCLK_EN();
		}else{
			GPIOE_PCLK_DIS();
		}
	}else if(pGPIOx == GPIOH) {
		if(en) {
			GPIOH_PCLK_EN();
		}else{
			GPIOH_PCLK_DIS();
		}
	}
}

/**
 * @brief GPIO Initialization. Initializes a GPIO with given settings
 *
 * @param GPIO Handle pointer
 *
 * @return void
 */

void	GPIO_Init(GPIO_Handle_t* gpioHandle) {
	uint8_t pinNumber = gpioHandle->config.GPIO_PinNumber;
	uint8_t tmp,tmp1,tmp2,portCode;

	//mode configuration
	tmp = gpioHandle->config.GPIO_PinMode;
	tmp1 = pinNumber/4;
	tmp2 = pinNumber%4;

	GPIO_Peripheral_Clk_Ctrl(gpioHandle, ENABLE);

	portCode = GPIO_BASEADDR_TO_CODE(gpioHandle->pGPIOx);
	switch(tmp) {
		case GPIO_MODE_INPUT:
		case GPIO_MODE_OUTPUT:
		case GPIO_MODE_ALTFUNC:
		case GPIO_MODE_ANALOG:
			gpioHandle->pGPIOx->MODER &= ~(0x03 << (2*pinNumber)); //clear
			gpioHandle->pGPIOx->MODER |= (tmp << (2*pinNumber)); //set
			break;
		case GPIO_MODE_IT_FT:
			gpioHandle->pGPIOx->MODER &= ~(0x03 << (2*pinNumber)); //clear
			gpioHandle->pGPIOx->MODER |= (GPIO_MODE_INPUT << (2*pinNumber)); //set
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR &= ~(1 << pinNumber);
			EXTI->IMR |= (1 << pinNumber);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[tmp1] =  portCode << (tmp2*4);
			break;
		case GPIO_MODE_IT_RT:
			gpioHandle->pGPIOx->MODER &= ~(0x03 << (2*pinNumber)); //clear
			gpioHandle->pGPIOx->MODER |= (GPIO_MODE_INPUT << (2*pinNumber)); //set
			EXTI->FTSR &= ~(1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
			EXTI->IMR |= (1 << pinNumber);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[tmp1] =  portCode << (tmp2*4);
			break;
		case GPIO_MODE_IT_FRT:
			gpioHandle->pGPIOx->MODER &= ~(0x03 << (2*pinNumber)); //clear
			gpioHandle->pGPIOx->MODER |= (GPIO_MODE_INPUT << (2*pinNumber)); //set
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
			EXTI->IMR |= (1 << pinNumber);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[tmp1] =  portCode << (tmp2*4);
			break;
	}

	//output type configuration
	tmp = gpioHandle->config.GPIO_PinOutType;
	gpioHandle->pGPIOx->OTYPER &= ~(0x01 << (pinNumber)); //clear
	gpioHandle->pGPIOx->OTYPER |= (tmp << (pinNumber)); //set

	//output speed configuration
	tmp = gpioHandle->config.GPIO_PinSpeed;
	gpioHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2*pinNumber)); //clear
	gpioHandle->pGPIOx->OSPEEDR |= (tmp << (2*pinNumber)); //set

	//port push/pull configuration
	tmp = gpioHandle->config.GPIO_PinPuPdConfig;
	gpioHandle->pGPIOx->PUPDR &= ~(0x03 << (2*pinNumber)); //clear
	gpioHandle->pGPIOx->PUPDR |= (tmp << (2*pinNumber)); //set

	//alt functionality configuration
	if(gpioHandle->config.GPIO_PinMode == GPIO_MODE_ALTFUNC) {
		if(pinNumber < 8) {
			tmp = gpioHandle->config.GPIO_PinAltFuncMode;
			gpioHandle->pGPIOx->AFRL &= ~(0x0f << (4*pinNumber)); //clear
			gpioHandle->pGPIOx->AFRL |= (tmp << (4*pinNumber)); //set
		}else{
			tmp = gpioHandle->config.GPIO_PinAltFuncMode;
			gpioHandle->pGPIOx->AFRH &= ~(0x0f << (4*pinNumber)); //clear
			gpioHandle->pGPIOx->AFRH |= (tmp << (4*pinNumber)); //set
		}
	}

}

/**
 * @brief returns the IRQ line for a GPIO config
 *
 * @param GPIO handle pointer
 *
 * @return int
 */

uint8_t GPIO_IRQNumber(GPIO_Handle_t * gpioHandle) {
	uint8_t IRQNumber = 0;
	switch(gpioHandle->config.GPIO_PinNumber) {
		case 0:
			IRQNumber = IRQ_EXTI0;
			break;
		case 1:
			IRQNumber = IRQ_EXTI1;
			break;
		case 2:
			IRQNumber = IRQ_EXTI2;
			break;
		case 3:
			IRQNumber = IRQ_EXTI3;
			break;
		case 4:
			IRQNumber = IRQ_EXTI4;
			break;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			IRQNumber = IRQ_EXTI9_5;
			break;
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
			IRQNumber = IRQ_EXTI15_10;
			break;
	}

	return IRQNumber;
}
/**
 * @brief Configures the IRQ line for GPIO
 *
 * @param IRQ number
 * @param enable/disable
 *
 * @return void
 */

void	GPIO_IRQ_Config(GPIO_Handle_t* gpioHandle, uint8_t en) {
	int IRQNumber = GPIO_IRQNumber(gpioHandle);

	if(en) {
		if(IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64) {
			IRQNumber = IRQNumber%32;
			*NVIC_ISER1 |= (1 << IRQNumber);
		}else if(IRQNumber >= 64 && IRQNumber < 96) {
			IRQNumber = IRQNumber%64;
			*NVIC_ISER2 |= (1 << IRQNumber);
		}
	}else{
		if(IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64) {
			IRQNumber = IRQNumber%32;
			*NVIC_ICER1 |= (1 << IRQNumber);
		}else if(IRQNumber >= 64 && IRQNumber < 96) {
			IRQNumber = IRQNumber%64;
			*NVIC_ICER2 |= (1 << IRQNumber);
		}
	}
}

/**
 * @brief sets the IRQ priority
 *
 * @param IRQ number
 * @param priority
 *
 * @return void
 */

void	GPIO_IRQ_Priority(GPIO_Handle_t* gpioHandle, uint8_t Priority) {
	uint8_t IRQNumber = GPIO_IRQNumber(gpioHandle);
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_offset = IRQNumber%4;
	*(NVIC_IPR_BASEADDR + (iprx*4)) |= ((Priority << (iprx_offset * 8)) << 4);
}

/**
 * @brief reads the pin value of a GPIO port
 *
 * @param GPIO handle pointer
 * @param pin number
 *
 * @return int
 */
uint8_t	GPIO_Read_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin) {
	uint16_t rdata = gpioHandle->pGPIOx->IDR;
	uint8_t  pinVal = ((rdata & (1 << pin)) != 0);
	return pinVal;
}

/**
 * @brief reads the port value of a GPIO port
 *
 * @param GPIO handle pointer
 *
 * @return uint16_t
 */
uint16_t	GPIO_Read_Port(GPIO_Handle_t* gpioHandle) {
	uint16_t rdata = gpioHandle->pGPIOx->IDR;
	return rdata;
}

/**
 * @brief write the pin value of a GPIO port
 *
 * @param GPIO handle pointer
 * @param pin number
 * @param pin value
 *
 * @return void
 */
void	GPIO_Write_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin, uint8_t value) {
	value = !(value == 0); //ensure that it's only a single bit
	if(value) {
		gpioHandle->pGPIOx->ODR |= (1 << pin);
	}else{
		gpioHandle->pGPIOx->ODR &= ~(1 << pin);
	}
}

/**
 * @brief write the pin value of a GPIO port
 *
 * @param GPIO handle pointer
 * @param port number
 * @param port value
 *
 * @return void
 */
void	GPIO_Write_Port(GPIO_Handle_t* gpioHandle, uint16_t value) {
	gpioHandle->pGPIOx->ODR = value;
}

/**
 * @brief toggle the pin value of a GPIO port
 *
 * @param GPIO handle pointer
 * @param pin number
 *
 * @return void
 */
void 		GPIO_Toggle_Pin(GPIO_Handle_t* gpioHandle, uint8_t pin) {
	gpioHandle->pGPIOx->ODR ^= (1 << pin);
}

/**
 * @brief reset the GPIO peripheral
 *
 * @param GPIO handle pointer
 *
 * @return void
 */
void		GPIO_Reset(GPIO_Handle_t* gpioHandle) {
	GPIO_RegDef_t* pGPIOx = gpioHandle->pGPIOx;
	if(pGPIOx == GPIOA) {
		GPIOA_RESET();
	}else if(pGPIOx == GPIOB) {
		GPIOB_RESET();
	}else if(pGPIOx == GPIOC) {
		GPIOC_RESET();
	}else if(pGPIOx == GPIOD) {
		GPIOD_RESET();
	}else if(pGPIOx == GPIOE) {
		GPIOE_RESET();
	}else if(pGPIOx == GPIOH) {
		GPIOH_RESET();
	}
}

/**
 * @brief GPIO IRQ Handler. clears the interrupt pending flag
 *
 * @param pin number
 *
 * @return void
 */
void		GPIO_IRQ_Handler(uint8_t pinNumber) {
	if(EXTI->PR & (1 << pinNumber)) {
		EXTI->PR |= (1 << pinNumber);
	}
}

