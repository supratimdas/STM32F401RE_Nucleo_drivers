/*
 * main_spi.c
 *
 *  Created on: 20-May-2021
 *      Author: supratim
 */


#include "stm32f401xx_gpio.h"
#include "stm32f401xx_spi.h"
#include <stdio.h>

//PA7 -> SPI1_MOSI
//PA5 -> SPI1_SCK

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

void SPI1_GPIOInit() {
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.config.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	SPIPins.config.GPIO_PinAltFuncMode = GPIO_AF5;
	SPIPins.config.GPIO_PinOutType = GPIO_OTYPE_PP;
	SPIPins.config.GPIO_PinPuPdConfig = GPIO_NO_PUPD;
	SPIPins.config.GPIO_PinSpeed = GPIO_SPEED_ULTRA;

	//configure MOSI
	SPIPins.config.GPIO_PinNumber = 7;
	GPIO_Init(&SPIPins);
	//configure MISO
	SPIPins.config.GPIO_PinNumber = 6;
	GPIO_Init(&SPIPins);
	//configure SCK
	SPIPins.config.GPIO_PinNumber = 5;
	GPIO_Init(&SPIPins);
}

void SPI1_Init(SPI_Handle_t *spi1) {
	spi1->pSPIx = SPI1;
	spi1->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi1->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi1->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	spi1->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi1->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi1->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi1->SPIConfig.SPI_SSM = SPI_SSM_SW;
	SPI1_GPIOInit();
	SPI_Init(spi1);
}

void delay(int d) {
	for(int i=0; i<d; i++);
}

extern void initialise_monitor_handles(void);
int main() {
	SPI_Handle_t spi1;
	uint8_t led_pin = 9;
	uint8_t led_state = LED_ON;
	uint8_t cmd = 0;
	uint8_t spi_rcv = 0;
	initialise_monitor_handles();
	printf("Starting...\n");

	SPI1_Init(&spi1);
	spi1.pSPIx->CR1  |= (1 << SPICR1_SSI);
	SPI_Peripheral_Ctrl(&spi1, ENABLE);
	delay(500000);
	delay(500000);
	delay(500000);
	delay(500000);
	delay(500000);
	delay(500000);


	while(1) {
		uint8_t dummyDat;
		printf("loop\n");
		cmd = COMMAND_LED_CTRL;
		//Send LED Control Cmd
		SPI_Send_Data_Blocking(&spi1, &cmd, 1);
		//SPI_Recv_Data_Blocking(&spi1, &dummyDat, 1); //dummy Read

		//Check Ack/Nack
		//SPI_Send_Data_Blocking(&spi1, &dummyDat, 1);
		SPI_Recv_Data_Blocking(&spi1, &spi_rcv, 1);


		printf("Received: %02x\n", spi_rcv);

		if(spi_rcv == ACK) {
			SPI_Send_Data_Blocking(&spi1, &led_pin, 1);
			//SPI_Recv_Data_Blocking(&spi1, &dummyDat, 1); //dummy Read
			SPI_Send_Data_Blocking(&spi1, &led_state, 1);
			//SPI_Recv_Data_Blocking(&spi1, &dummyDat, 1); //dummy Read
			if(led_state == LED_ON) {
				led_state = LED_OFF;
			}else{
				led_state = LED_ON;
			}
		}else{
			continue;
		}

		delay(500000);
		delay(500000);
		delay(500000);
		delay(500000);
	}
}
