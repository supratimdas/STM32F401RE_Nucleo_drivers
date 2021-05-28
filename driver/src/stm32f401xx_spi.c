/*
 * stm32f401xx_spi.c
 *
 *  Created on: 19-May-2021
 *      Author: supratim
 */
#include "stm32f401xx_spi.h"

void    SPI_Peripheral_Clk_Ctrl(SPI_Handle_t *spiHandle, uint8_t en) {
    SPI_RegDef_t *pSPIx = spiHandle->pSPIx;
    if(pSPIx == SPI1) {
        if(en) {
            SPI1_PCLK_EN();
        }else{
            SPI1_PCLK_DIS();
        }
    }else if(pSPIx == SPI2) {
        if(en) {
            SPI2_PCLK_EN();
        }else{
            SPI2_PCLK_DIS();
        }
    }else if(pSPIx == SPI3) {
        if(en) {
            SPI3_PCLK_EN();
        }else{
            SPI3_PCLK_DIS();
        }
    }else if(pSPIx == SPI4) {
        if(en) {
            SPI4_PCLK_EN();
        }else{
            SPI4_PCLK_DIS();
        }
    } 
}

void SPI_Peripheral_Ctrl(SPI_Handle_t *spiHandle, uint8_t en) {
	if(en) {
		spiHandle->pSPIx->CR1 |= (1 << SPICR1_SPE);
	}else{
		spiHandle->pSPIx->CR1 &= ~(1 << SPICR1_SPE);
	}
}

void 	SPI_Init(SPI_Handle_t* spiHandle) {
	//enable peripheral clock
	SPI_Peripheral_Clk_Ctrl(spiHandle, ENABLE);

    //configure device_mode
    if(spiHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
        SET_BIT(spiHandle->pSPIx->CR1, SPICR1_MSTR);
    }else if(spiHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE) {
        CLEAR_BIT(spiHandle->pSPIx->CR1, SPICR1_MSTR);
    }

    //configure bus config
    if(spiHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_BIDI_MODE);
    }else if(spiHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_BIDI_MODE);
    }else if(spiHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_BIDI_MODE);
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_RXONLY);
    }

    
    //configure SCLK speed
    spiHandle->pSPIx->CR1 &= ~(0x07 << SPICR1_BR_LSB);
    spiHandle->pSPIx->CR1 |= (spiHandle->SPIConfig.SPI_SclkSpeed << SPICR1_BR_LSB);

    //configure DFF
    if(spiHandle->SPIConfig.SPI_DFF == SPI_DFF_8BITS) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_DFF);
    }else if(spiHandle->SPIConfig.SPI_DFF == SPI_DFF_16BITS) {
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_DFF);
    }

    //configure CPOL
    if(spiHandle->SPIConfig.SPI_CPOL == SPI_CPOL_LOW) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_CPOL);
    }else if(spiHandle->SPIConfig.SPI_CPOL == SPI_CPOL_HIGH) {
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_CPOL);
    }

    //configure CPHA
    if(spiHandle->SPIConfig.SPI_CPHA == SPI_CPHA_LOW) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_CPHA);
    }else if(spiHandle->SPIConfig.SPI_CPHA == SPI_CPHA_HIGH) {
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_CPHA);
    }

    //configure SSM
    if(spiHandle->SPIConfig.SPI_SSM == SPI_SSM_HW) {
        CLEAR_BIT(spiHandle->pSPIx->CR1,SPICR1_SSM);
    }else if(spiHandle->SPIConfig.SPI_SSM == SPI_SSM_SW) {
        SET_BIT(spiHandle->pSPIx->CR1,SPICR1_SSM);
    }

}

void	SPI_Reset(SPI_Handle_t* spiHandle) {
    SPI_RegDef_t *pSPIx = spiHandle->pSPIx;
    if(pSPIx == SPI1) {
        SPI1_RESET();
    }else if(pSPIx == SPI2) {
        SPI2_RESET();
    }else if(pSPIx == SPI3) {
        SPI3_RESET();
    }else if(pSPIx == SPI4) {
        SPI4_RESET();
    } 
}

uint8_t SPI_IRQNumber(SPI_Handle_t* spiHandle) {
	SPI_RegDef_t *pSPIx = spiHandle->pSPIx;
	uint8_t IRQNumber = 0;
	if(pSPIx == SPI1) {
		IRQNumber = 35;
	}else if(pSPIx == SPI2) {
		IRQNumber = 36;
	}else if(pSPIx == SPI3) {
		IRQNumber = 51;
	}else if(pSPIx == SPI4) {
		IRQNumber = 84;
	}
	return IRQNumber;
}

void	SPI_IRQ_Config(SPI_Handle_t* spiHandle, uint8_t en) {
	uint8_t IRQNumber = SPI_IRQNumber(spiHandle);

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

void	SPI_IRQ_Priority(SPI_Handle_t* spiHandle, uint8_t Priority) {
	uint8_t IRQNumber = SPI_IRQNumber(spiHandle);

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_offset = IRQNumber%4;
	*(NVIC_IPR_BASEADDR + (iprx*4)) |= ((Priority << (iprx_offset * 8)) << 4);
}

void	SPI_IRQ_Handler(uint8_t pinNumber) {

}

void	SPI_Send_Data_Blocking(SPI_Handle_t* spiHandle, uint8_t* txBuff, uint8_t dataLen) {
    while(dataLen > 0) {
        //wait until TXE is set
        while(!(spiHandle->pSPIx->SR & (1<<SPISR_TXE)));

        //load data to DR
        if(spiHandle->pSPIx->CR1 & (1 << SPICR1_DFF)) { //16bit
            spiHandle->pSPIx->DR = *((uint16_t*)txBuff);
            dataLen -= 2;
            txBuff += 2;
        }else{ //8bit
            spiHandle->pSPIx->DR = *((uint8_t*)txBuff);
            dataLen -= 1;
            txBuff += 1;
        }

    	if((spiHandle->pSPIx->CR1 & (1 << SPICR1_MSTR))) {	//if master. Master can only initiate transfer, discard any bytes received
            //wait until TXE is set
            while(!(spiHandle->pSPIx->SR & (1<<SPISR_RXNE)));

    		uint16_t dummyRead = spiHandle->pSPIx->DR; //clear RXNE flag
    		(void)dummyRead;
    	}
    }
}

void 	SPI_Recv_Data_Blocking(SPI_Handle_t* spiHandle, uint8_t* rxBuff, uint8_t dataLen) {
    while(dataLen > 0) {
    	if((spiHandle->pSPIx->CR1 & (1 << SPICR1_MSTR))) {	//if master. Master can only initiate transfer, by sending dummy bytes to received
            //wait until TXE is set
            while(!(spiHandle->pSPIx->SR & (1<<SPISR_TXE)));

    		spiHandle->pSPIx->DR = 0;
    	}

    	//wait until RXNE is set
        while(!(spiHandle->pSPIx->SR & (1<<SPISR_RXNE)));

        //load data to DR
        if(spiHandle->pSPIx->CR1 & (1 << SPICR1_DFF)) { //16bit
        	*((uint16_t*)rxBuff) = spiHandle->pSPIx->DR;
            dataLen -= 2;
            rxBuff += 2;
        }else{ //8bit
        	*((uint8_t*)rxBuff) = spiHandle->pSPIx->DR;
            dataLen -= 1;
            rxBuff += 1;
        }
    }
}
