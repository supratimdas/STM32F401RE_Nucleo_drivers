/*
 * stm32f401xx.h
 *
 *  Created on: May 4, 2021
 *      Author: supratim
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>
/***************CortexM4 related details**********************************/
#define NVIC_ISER0 		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 		((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0 		((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2 		((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3 		((volatile uint32_t*)0xE000E18C)

#define NVIC_IPR_BASEADDR	((volatile uint32_t*)0xE000E400)

/*************************************************************************/

//Base addresses of various major sections
#define FLASH_BASEADDR			0x08000000UL
#define SRAM_BASEADDR			0x20000000UL
#define SRAM					SRAM_BASEADDR
#define ROM						0x1FFF0000UL
#define APB1_BASEADDR			0x40000000UL
#define APB2_BASEADDR			0x40010000UL
#define AHB1_BASEADDR			0x40020000UL
#define AHB2_BASEADDR			0x50000000UL

//Base addresses of peripherals on APB1
#define TIM2_BASEADDR			(APB1_BASEADDR + 0x0000)
#define TIM3_BASEADDR			(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR			(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR			(APB1_BASEADDR + 0x0C00)
#define RTC_BASEADDR			(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR			(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR			(APB1_BASEADDR + 0x3000)
#define I2S2_EXT_BASEADDR		(APB1_BASEADDR + 0x3400)
#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00)
#define I2S3_EXT_BASEADDR		(APB1_BASEADDR + 0x4000)
#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400)
#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00)
#define PWR_BASEADDR			(APB1_BASEADDR + 0x7000)


//Base addresses of peripherals on APB2
#define TIM1_BASEADDR			(APB2_BASEADDR + 0x0000)
#define USART1_BASEADDR			(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2_BASEADDR + 0x1400)
#define ADC1_BASEADDR			(APB2_BASEADDR + 0x2000)
#define SDIO_BASEADDR			(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR			(APB2_BASEADDR + 0x3C00)
#define TIM9_BASEADDR			(APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR			(APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR			(APB2_BASEADDR + 0x4800)


//Base addresses of peripherals on AHB1
#define GPIOA_BASEADDR			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00)
#define CRC_BASEADDR			(AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR			(AHB1_BASEADDR + 0x3800)
#define FLASH_INTF_BASEADDR  	(AHB1_BASEADDR + 0x3C00)
#define DMA1_BASEADDR			(AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR			(AHB1_BASEADDR + 0x6400)


//Base addresses of peripherals on AHB2
#define USB_OTG_BASEADDR			(AHB2_BASEADDR + 0x0000)

//Peripheral register definition structure for GPIO
typedef struct {
	volatile uint32_t MODER;	/*Mode Register						Address Offset: 0x00*/
	volatile uint32_t OTYPER;	/*Output Type Register				Address Offset: 0x04*/
	volatile uint32_t OSPEEDR;	/*Output Speed Register				Address Offset: 0x08*/
	volatile uint32_t PUPDR;    /*Pull-up/down Register				Address Offset: 0x0C*/
	volatile uint32_t IDR;		/*Input Data Register				Address Offset: 0x10*/
	volatile uint32_t ODR;		/*Output Data Register				Address Offset: 0x14*/
	volatile uint32_t BSRR;		/*Bit Set/Reset Register			Address Offset: 0x18*/
	volatile uint32_t LCKR;		/*Config Lock Register				Address Offset: 0x1C*/
	volatile uint32_t AFRL;		/*AltFunc Low Register				Address Offset: 0x20*/
	volatile uint32_t AFRH;		/*AltFunc High Register				Address Offset: 0x24*/
} GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

typedef struct {
	volatile uint32_t CR; 			/*RCC Control Register				Address Offset: 0x00*/
	volatile uint32_t PLLCFGR; 		/*RCC PLL Config Register   		Address Offset: 0x04*/
	volatile uint32_t CFGR;			/*RCC Clock Config Register			Address Offset: 0x08*/
	volatile uint32_t CIR; 			/*RCC Clock Interrupt Register		Address Offset: 0x0C*/
	volatile uint32_t AHB1RSTR; 	/*AHB1 periph rst register			Address Offset: 0x10*/
	volatile uint32_t AHB2RSTR; 	/*AHB2 periph rst register			Address Offset: 0x14*/
	volatile uint32_t RSVD0; 		/*RESERVED							Address Offset: 0x18*/
	volatile uint32_t RSVD1; 		/*RESERVED							Address Offset: 0x1C*/
	volatile uint32_t APB1RSTR; 	/*APB1 periph rst register			Address Offset: 0x20*/
	volatile uint32_t APB2RSTR; 	/*APB2 periph rst register			Address Offset: 0x24*/
	volatile uint32_t RSVD2; 		/*RESERVED							Address Offset: 0x28*/
	volatile uint32_t RSVD3; 		/*RESERVED							Address Offset: 0x2C*/
	volatile uint32_t AHB1ENR; 		/*AHB1 periph clk en register		Address Offset: 0x30*/
	volatile uint32_t AHB2ENR; 		/*AHB2 periph clk en register		Address Offset: 0x34*/
	volatile uint32_t RSVD4; 		/*RESERVED							Address Offset: 0x38*/
	volatile uint32_t RSVD5; 		/*RESERVED							Address Offset: 0x3C*/
	volatile uint32_t APB1ENR; 		/*APB1 periph clk en register		ddress Offset: 0x40*/
	volatile uint32_t APB2ENR; 		/*APB2 periph clk en register		Address Offset: 0x44*/
	volatile uint32_t RSVD6; 		/*RESERVED							Address Offset: 0x48*/
	volatile uint32_t RSVD7; 		/*RESERVED							Address Offset: 0x4C*/
	volatile uint32_t AHB1LPENR;	/*AHB1 low pwr clk en register		Address Offset: 0x50*/
	volatile uint32_t AHB2LPENR;	/*AHB2 low pwr clk en register		Address Offset: 0x54*/
	volatile uint32_t RSVD8; 		/*RESERVED							Address Offset: 0x58*/
	volatile uint32_t RSVD9; 		/*RESERVED							Address Offset: 0x5C*/
	volatile uint32_t APB1LPENR;	/*APB1 low pwr clk en register		Address Offset: 0x60*/
	volatile uint32_t APB2LPENR;	/*APB2 low pwr clk en register		Address Offset: 0x64*/
	volatile uint32_t RSVD10; 		/*RESERVED							Address Offset: 0x68*/
	volatile uint32_t RSVD11; 		/*RESERVED							Address Offset: 0x6C*/
	volatile uint32_t BDCR; 		/*RCC Bkup domain Control Register	Address Offset: 0x70*/
	volatile uint32_t CSR; 			/*RCC clk Control & status Register	Address Offset: 0x74*/
	volatile uint32_t RSVD12; 		/*RESERVED							Address Offset: 0x78*/
	volatile uint32_t RSVD13; 		/*RESERVED							Address Offset: 0x7C*/
	volatile uint32_t SSCGR; 		/*Spread Spectrum clk gen Register	Address Offset: 0x80*/
	volatile uint32_t PLLI2SCFGR;	/*PLLI2S config register			Address Offset: 0x84*/
	volatile uint32_t RSVD14; 		/*RESERVED							Address Offset: 0x88*/
	volatile uint32_t DCKCFGR; 		/*Dedicated clk config Register		Address Offset: 0x8C*/
} RCC_RegDef_t;

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;

#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)

typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RSVD1;
	uint32_t RSVD2;
	volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

typedef struct {
    volatile uint32_t   CR1;
    volatile uint32_t	CR2;
    volatile uint32_t   SR;
    volatile uint32_t   DR;
    volatile uint32_t   CRCPR;
    volatile uint32_t   RXCRCR;
    volatile uint32_t   TXCRCR;
    volatile uint32_t   I2SCFGR;
    volatile uint32_t   I2SPR;
} SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASEADDR)

//SPI Register Bit Fields
#define SPICR1_CPHA			0
#define SPICR1_CPOL 		1
#define SPICR1_MSTR 		2
#define SPICR1_BR_LSB		3
#define SPICR1_BR_MSB		5
#define SPICR1_SPE			6
#define SPICR1_LSB_FIRST	7
#define SPICR1_SSI			8
#define SPICR1_SSM			9
#define SPICR1_RXONLY		10
#define SPICR1_DFF			11
#define SPICR1_CRC_NEXT		12
#define SPICR1_CRC_EN		13
#define SPICR1_BIDI_OE		14
#define SPICR1_BIDI_MODE	15

#define SPISR_RXNE			0
#define SPISR_TXE			1
#define SPISR_CHSIDE		2
#define SPISR_UDR			3
#define SPISR_CRC_ERR		4
#define SPISR_MODF			5
#define SPISR_OVR			6
#define SPISR_BSY			7
#define SPISR_FRE			8

/*Clock Enable Macro*/

//GPIO
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

//SPI
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

//USART
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))


//SYSCFG
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*Clock Disable Macro*/
//GPIO
#define GPIOA_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 7))

//SPI
#define SPI1_PCLK_DIS()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()		(RCC->APB2ENR &= ~(1 << 13))

//USART
#define USART1_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 5))


//SYSCFG
#define SYSCFG_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 14))


/*Macros to reset peripherals*/
#define GPIOA_RESET()		{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}
#define GPIOB_RESET()		{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}
#define GPIOC_RESET()		{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}
#define GPIOD_RESET()		{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}
#define GPIOE_RESET()		{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}
#define GPIOH_RESET()		{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}


#define SPI1_RESET()		{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}
#define SPI4_RESET()		{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}
#define SPI2_RESET()		{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}
#define SPI3_RESET()		{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}

/*Macros for GPIO MODE*/
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFUNC 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_FRT	6


/*Macros for GPIO Output mode*/
#define GPIO_OTYPE_PP		0
#define GPIO_OTYPE_OD		1

/*Macros for GPIO Output Speed*/
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_ULTRA	3

/*Macros for GPIO Port Pull-up/Pull-down type*/
#define GPIO_NO_PUPD 	0
#define GPIO_PU			1
#define GPIO_PD			2

/*Macors for GPIO port Alternate Functions*/
#define GPIO_AF0		0
#define GPIO_AF1		1
#define GPIO_AF2		2
#define GPIO_AF3		3
#define GPIO_AF4		4
#define GPIO_AF5		5
#define GPIO_AF6		6
#define GPIO_AF7		7
#define GPIO_AF8		8
#define GPIO_AF9		9
#define GPIO_AF10		10
#define GPIO_AF11		11
#define GPIO_AF12		12
#define GPIO_AF13		13
#define GPIO_AF14		14
#define GPIO_AF15		15

//Interrrupt IRQ numbers
#define IRQ_WWDG				0
#define IRQ_EXTI16				1
#define IRQ_PVD					1
#define IRQ_EXTI21				2
#define IRQ_TAMP_STAMP			2
#define IRQ_EXTI22				3
#define IRQ_RTC_WKUP			3
#define IRQ_FLASH				4
#define IRQ_RCC					5
#define IRQ_EXTI0				6
#define IRQ_EXTI1				7
#define IRQ_EXTI2				8
#define IRQ_EXTI3				9
#define IRQ_EXTI4				10
#define IRQ_DMA1_STREAM0		11
#define IRQ_DMA1_STREAM1		12
#define IRQ_DMA1_STREAM2		13
#define IRQ_DMA1_STREAM3		14
#define IRQ_DMA1_STREAM4		15
#define IRQ_DMA1_STREAM5		16
#define IRQ_DMA1_STREAM6		17
#define IRQ_ADC					18
#define IRQ_EXTI9_5				23
#define IRQ_TIM1_BRK_TIM9		24
#define IRQ_TIM1_UP_TIM10		25
#define IRQ_TIM1_TRG_COM_TIM11	26
#define IRQ_TIM1_CC				27
#define IRQ_TIM2				28
#define IRQ_TIM3				29
#define IRQ_TIM4				30
#define IRQ_I2C1_EV				31
#define IRQ_I2C1_ER				32
#define IRQ_I2C2_EV				33
#define IRQ_I2C2_ER				34
#define IRQ_SPI					35
#define IRQ_SP2					36
#define IRQ_USART1				37
#define IRQ_USART2				38
#define IRQ_EXTI15_10			40
#define IRQ_EXTI17				41
#define IRQ_RTC_ALARM			41
#define IRQ_EXTI18				42
#define IRQ_OTG_FS_WKUP			42
#define IRQ_DMA1_STREAM			47
#define IRQ_SDIO				49
#define IRQ_TIM5				50
#define IRQ_SPI3				51
#define IRQ_DMA2_STREAM0		56
#define IRQ_DMA2_STREAM1		57
#define IRQ_DMA2_STREAM2		58
#define IRQ_DMA2_STREAM3		59
#define IRQ_DMA2_STREAM4		60
#define IRQ_OTG_FS				67
#define IRQ_DMA2_STREAM5		68
#define IRQ_DMA2_STREAM6		69
#define IRQ_DMA2_STREAM7		70
#define IRQ_USART6				71
#define IRQ_I2C3_EV				72
#define IRQ_I2C3_ER				73
#define IRQ_FPU					81
#define IRQ_SPI4				84




/*Enable/Disable macro*/
#define ENABLE          1
#define DISABLE         0

#define GPIO_BASEADDR_TO_CODE(x)	(x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOH) ? 7 : 0


#define SET_BIT(X,N)        X |= (1<<N)
#define CLEAR_BIT(X,N)      X &= ~(1<<N)

#endif /* INC_STM32F401XX_H_ */
