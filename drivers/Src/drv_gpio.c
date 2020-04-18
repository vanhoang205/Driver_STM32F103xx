/*
 * DRV_GPIO.c
 *
 *  Created on: Apr 14, 2020
 *      Author: vanho
 */

#include "stm32f103xx.h"
#include "drv_gpio.h"


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

	// Configure mode, speed, output type and interrupt for pin
	uint32_t temp = 0;

	if (pPortHandle->pinConfig.pinMode <= MODE_ALT_OPENDR) {		// not using interrupt
		uint8_t temp1 = pPortHandle->pinConfig.pinNumber / 8;
		uint8_t temp2 = pPortHandle->pinConfig.pinNumber % 8;
		if (pPortHandle->pinConfig.pinMode <= MODE_INPUT_PUPDR) {
			// default choose input and analog mode function

			//choose analog mode or floating input or pull-up/pull down
			temp = pPortHandle->pGPIO->CR[temp1];
			temp &= ~(MASK_BIT2_3 << (4 * temp2));
			temp |= pPortHandle->pinConfig.pinMode << ((4 * temp2) + 2);
			pPortHandle->pGPIO->CR[temp1] = temp;
		} else {
			// choose output and alternative function mode
			// At the same time, choose speed output
			pPortHandle->pGPIO->CR[temp1] |= pPortHandle->pinConfig.pinSpeed << (4 * temp2);

			// choose alternative or output mode with open-drain/push pull capacity
			temp = pPortHandle->pGPIO->CR[temp1];
			temp &= ~(MASK_BIT2_3 << (4 * temp2));
			temp |= ((pPortHandle->pinConfig.pinMode - 4) << ((4 * temp2) + 2));
			pPortHandle->pGPIO->CR[temp1] = temp;
		}
	} else {	// using interrupt
		uint8_t temp1 = pPortHandle->pinConfig.pinNumber / 4;
		uint8_t temp2 = pPortHandle->pinConfig.pinNumber % 4;
		if (pPortHandle->pinConfig.pinMode == MODE_IT_FALL_EDGE) {
			// configure the FTSR
			EXTI->FTSR |= 1 << pPortHandle->pinConfig.pinNumber;
			EXTI->RTSR &= ~(1 << pPortHandle->pinConfig.pinNumber);
		} else if (pPortHandle->pinConfig.pinMode == MODE_IT_RISE_EDGE) {
			EXTI->RTSR |= 1 << pPortHandle->pinConfig.pinNumber;
			EXTI->FTSR &= ~(1 << pPortHandle->pinConfig.pinNumber);
		} else if (pPortHandle->pinConfig.pinMode == MODE_IT_RISE_FALL_EDGE) {
			EXTI->FTSR |= 1 << pPortHandle->pinConfig.pinNumber;
			EXTI->RTSR |= 1 << pPortHandle->pinConfig.pinNumber;
		}
		//2. configure the GPIO port selection in SYSCFG_EXTIR
		AFIO_PCLK_EN();
		AFIO->EXTICR[temp1] |= DECODE_PORT_EXTI(pPortHandle->pGPIO) << (4 * temp2);
		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pPortHandle->pinConfig.pinNumber;
	}

	// configure the pull/pull down resistor
	temp = 0;
	if (pPortHandle->pinConfig.pinMode != MODE_INPUT_FLOAT) {
		pPortHandle->pGPIO->ODR |= (pPortHandle->pinConfig.pinResType << pPortHandle->pinConfig.pinNumber);
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pPort) {
	if (pPort == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pPort == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pPort == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pPort == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pPort == GPIOE) {
		GPIOE_REG_RESET();
	}
}

/* Read and Write data */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pPort, uint8_t pinNum) {
	uint8_t value;
	value = (uint8_t)((pPort->IDR >> pinNum) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pPort) {
	uint16_t value;
	value = (uint16_t)(pPort->IDR);
	return value;
}

void GPIO_WritePin(GPIO_RegDef_t *pPort, uint8_t pinNum, uint8_t value) {
	if (value == HIGH) {
		pPort->ODR |= (value << pinNum);
	} else {
		pPort->ODR &= ~(value << pinNum);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pPort, uint16_t value) {
	pPort->ODR = value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pPort, uint8_t pinNum) {
	pPort->ODR ^= (1 << pinNum);
}

/* config interrupt */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPrior, uint8_t isEnabled) {
	if (isEnabled == TRUE) {
		if (IRQNum <= 31) {
			*NVIC_ISER0 |= 1 << IRQNum;
		} else if (IRQNum > 31 && IRQNum <= 64) {
			*NVIC_ISER1 |= 1 << (IRQNum % 32);
		}
	} else {
		if (IRQNum <= 31) {
			*NVIC_ICER0 |= 1 << IRQNum;

		} else if (IRQNum > 31 && IRQNum <= 64) {
			*NVIC_ICER1 |= 1 << (IRQNum % 32);

		}
	}
}

void GPIO_IRQHandling(uint8_t pinNum) {

}
