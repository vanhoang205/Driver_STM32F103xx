/*
 * DRV_GPIO.c
 *
 *  Created on: Apr 14, 2020
 *      Author: vanho
 */

#include "stm32f103xx.h"
#include "drv_gpio.h"

static void ConfigCNFbit_CR(uint8_t pin, uint8_t cnf) {
	uint32_t temp;
	if (pin / 8) {
		temp = pPortHandle->pGPIO->CR[1];
		temp &= ~(MASK_2BIT << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
		temp |= (MODE_ANALOG << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
		pPortHandle->pGPIO->CR[1] = temp;
	} else {
		temp = pPortHandle->pGPIO->CR[0];
		temp &= ~(MASK_2BIT << (4 * pPortHandle->pinConfig.pinNumber));
		temp |= (MODE_ANALOG << (4 * pPortHandle->pinConfig.pinNumber));
		pPortHandle->pGPIO->CR[0] = temp;
	}
}
/* Control Peripheral clock */
void GPIO_PeriClockControl(GPIO_RegDef_t *pPort, uint8_t isEnabled) {
	if (isEnabled == TRUE) {
		if (pPort == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pPort == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pPort == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pPort == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pPort == GPIOE) {
			GPIOE_PCLK_EN();
		}
	} else {
		// Do nothing
	}
}

/* Initialization and Deinitialization Pin */
void GPIO_Init(GPIO_Handle_t *pPortHandle) {
	/*!< 1. Configure the mode of GPIO >*/
	uint32_t temp = 0;
	if (pPortHandle->pinConfig.pinMode < MODE_OUTPUT) { // Choosing analog mode or input mode

		if (pPortHandle->pinConfig.pinMode == MODE_ANALOG) {
			if (pPortHandle->pinConfig.pinNumber / 8) {
				temp = pPortHandle->pGPIO->CR[1];
				temp &= ~(MASK_2BIT << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
				temp |= (MODE_ANALOG << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
				pPortHandle->pGPIO->CR[1] = temp;
			} else {
				temp = pPortHandle->pGPIO->CR[0];
				temp &= ~(MASK_2BIT << (4 * pPortHandle->pinConfig.pinNumber));
				temp |= (MODE_ANALOG << (4 * pPortHandle->pinConfig.pinNumber));
				pPortHandle->pGPIO->CR[0] = temp;
			}
			temp = 0;
		} else if (pPortHandle->pinConfig.pinMode == MODE_INPUT) {
			if (pPortHandle->pinConfig.pinInType != INPUT_TYPE_FLOAT) {
				if (pPortHandle->pinConfig.pinNumber / 8) {
					temp = pPortHandle->pGPIO->CR[1];
					temp &= ~(MASK_2BIT << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
					temp |= (0x02 << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
					pPortHandle->pGPIO->CR[1] = temp;
				} else {
					temp = pPortHandle->pGPIO->CR[0];
					temp &= ~(MASK_2BIT << (4 * pPortHandle->pinConfig.pinNumber));
					temp |= (0x02 << (4 * pPortHandle->pinConfig.pinNumber));
					pPortHandle->pGPIO->CR[0] = temp;
				}
			}
			temp = 0;
		}
	} else {						// Choosing output mode or altenative mode
		if (pPortHandle->pinConfig.pinNumber / 8) {
			pPortHandle->pGPIO->CR[1] |= (0x01
					<< (4 * (pPortHandle->pinConfig.pinNumber % 8)));
		} else {
			pPortHandle->pGPIO->CR[0] = (0x01
					<< (4 * pPortHandle->pinConfig.pinNumber));
		}
		if (pPortHandle->pinConfig.pinMode == MODE_ALTFUNC) {
			if (pPortHandle->pinConfig.pinOutType == OUTPUT_TYPE_OPENDR) {
				if (pPortHandle->pinConfig.pinNumber / 8) {
					temp = pPortHandle->pGPIO->CR[1];
					temp &= ~(MASK_2BIT << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
					temp |= (0x03 << (4 * (pPortHandle->pinConfig.pinNumber % 8)));
					pPortHandle->pGPIO->CR[1] = temp;
				} else {
					temp = pPortHandle->pGPIO->CR[0];
					temp &= ~(MASK_2BIT << (4 * pPortHandle->pinConfig.pinNumber));
					temp |= (0x03 << (4 * pPortHandle->pinConfig.pinNumber));
					pPortHandle->pGPIO->CR[0] = temp;
				}
			}
		}
	}

	//2. Configure the speed

	//3. Configure the pull up/ pull down resistor

	//4. Configure the output type

	//5. Configure the alt functionality
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}

/* Read and Write data */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum) {

}

void GPIO_ReadPort(GPIO_RegDef_t *pGPIOx) {

}

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t value) {

}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum) {

}

/* config interrupt */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPrior, uint8_t isEnabled) {

}

void GPIO_IRQHandling(uint8_t pinNum) {

}
