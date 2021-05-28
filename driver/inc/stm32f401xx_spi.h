/*
 * stm32f401xx_spi.h
 *
 *  Created on: 19-May-2021
 *      Author: supratim
 */

#ifndef INC_STM32F401XX_SPI_H_
#define INC_STM32F401XX_SPI_H_


#include "stm32f401xx.h"

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

#define SPI_BUS_CONFIG_FD               0
#define SPI_BUS_CONFIG_HD               1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   2

#define SPI_SCLK_SPEED_DIV2         0
#define SPI_SCLK_SPEED_DIV4         1
#define SPI_SCLK_SPEED_DIV8         2
#define SPI_SCLK_SPEED_DIV16        3
#define SPI_SCLK_SPEED_DIV32        4
#define SPI_SCLK_SPEED_DIV64        5
#define SPI_SCLK_SPEED_DIV128       6
#define SPI_SCLK_SPEED_DIV256       7

#define SPI_DFF_8BITS               0
#define SPI_DFF_16BITS              1

#define SPI_CPOL_HIGH               1
#define SPI_CPOL_LOW                0

#define SPI_CPHA_HIGH               1
#define SPI_CPHA_LOW                0

#define SPI_SSM_HW                  1
#define SPI_SSM_SW                  0

void		SPI_Peripheral_Clk_Ctrl(SPI_Handle_t* spiHandle, uint8_t en);
void		SPI_Peripheral_Ctrl(SPI_Handle_t* spiHandle, uint8_t en);
void 		SPI_Init(SPI_Handle_t* spiHandle);
void		SPI_Reset(SPI_Handle_t* spiHandle);
void		SPI_IRQ_Config(SPI_Handle_t* spiHandle, uint8_t en);
void		SPI_IRQ_Priority(SPI_Handle_t* spiHandle, uint8_t Priority);
void		SPI_IRQ_Handler(uint8_t pinNumber);

void	    SPI_Send_Data_Blocking(SPI_Handle_t* spiHandle, uint8_t* txBuff, uint8_t dataLen);
void 	    SPI_Recv_Data_Blocking(SPI_Handle_t* spiHandle, uint8_t* rxBuff, uint8_t dataLen);


#endif /* INC_STM32F401XX_SPI_H_ */
