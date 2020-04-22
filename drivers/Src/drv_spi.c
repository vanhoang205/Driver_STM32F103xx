#include "stm32f103xx.h"
#include "drv_spi.h"


void SPI_Remap(SPI_Handle_t *spi) {
	if (spi->SPIConfig.remapName == SPI_REMAP_NONE) {
		AFIO->MAPR &= ~(1 << SPI_MAPR_SPI1REMAP_BIT);
	} else if (spi->SPIConfig.remapName == SPI_REMAP_SPI1) {
		AFIO->MAPR |= (1 << SPI_MAPR_SPI1REMAP_BIT);
	}
}

uint8_t GetFlagStatus(SPI_RegDef_t *pSPI, uint8_t flagType) {
	if (pSPI->SR & flagType) {
		return SPI_FLAG_SET;
	}
	return SPI_FLAG_RESET;
}
/* Control Peripheral clock */
void SPI_PeriClockControl(SPI_RegDef_t *pSPI, uint8_t isEnabled) {
	if (isEnabled == TRUE) {
		if (pSPI == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPI == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPI == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		// Do nothing
	}
}

/* Initialization and Deinitialization Pin */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	uint32_t tempReg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPI, TRUE);
	// configure device type
	tempReg |= pSPIHandle->SPIConfig.deviceType << SPI_CR1_MSTR_BIT;

	// thiet lap ssm
	tempReg |= pSPIHandle->SPIConfig.slaveSelectType << SPI_CR1_SSM_BIT;

	// thiet lap baund rate
	tempReg |= pSPIHandle->SPIConfig.baundRate << SPI_CR1_BR0_BIT;

	// thiet lap dataframe
	tempReg |= pSPIHandle->SPIConfig.dataFrameFormat << SPI_CR1_DFF_BIT;

	// thiet lap transmission format
	// note: defaule mode is transmit mode full duplex
	if (pSPIHandle->SPIConfig.transmitMode == SPI_TRANSMOD_HALFDUP) {
		tempReg |= (1 << SPI_CR1_BIDIMODE_BIT);
	} else if (pSPIHandle->SPIConfig.transmitMode == SPI_TRANSMOD_HALFDUP) {
		tempReg |= (1 << SPI_CR1_RXONLY_BIT);
	}
	pSPIHandle->pSPI->CR[0] = tempReg;
	SPI_Remap(pSPIHandle);
}
void SPI_DeInit(SPI_RegDef_t *pSPI) {

}

/*
 * Data transmit and receive
 */
void SPI_TransmitData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t len) {
	while (len) {
		while (GetFlagStatus(pSPI, SPI_SR_TXE_BIT) == SPI_FLAG_RESET);

		if (pSPI->CR[0] & (1 << SPI_CR1_DFF_BIT)) {
			pSPI->DR = *((uint16_t*)(pTxBuffer));
			len -= 2;
			(uint16_t*)pTxBuffer++;
		} else {
			pSPI->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t len) {

}

/* config interrupt */
void SPI_ConfigIRQ(uint8_t IRQNum, uint8_t isEnabled) {

}
void SPI_SetPriorityIRQ(uint8_t IRQNum, uint32_t priority) {

}
void SPI_IRQHandling(SPI_Handle_t *pHandle) {

}

