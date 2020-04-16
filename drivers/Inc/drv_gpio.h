/*
 * DRV_GPIO.h
 *
 *  Created on: Apr 14, 2020
 *      Author: vanho
 */

#ifndef INC_DRV_GPIO_H_
#define INC_DRV_GPIO_H_

#define MASK_2BIT		0x0C

/*
 * configure mode for GPIO
 */
#define MODE_ANALOG							0
#define MODE_INPUT							1
#define MODE_OUTPUT							2
#define MODE_ALTFUNC						3

/*
 * configure input type for GPIO
 */
#define INPUT_TYPE_FLOAT					0
#define INPUT_TYPE_PUR						1
#define INPUT_TYPE_PDR						2

/*
 * configure output type for GPIO
 */
#define OUTPUT_TYPE_PUPU					0
#define OUTPUT_TYPE_OPENDR					1

/*
 * Choosing output speed
 */
#define OUTPUT_SPEED_10Mhz					1
#define OUTPUT_SPEED_2Mhz					2
#define OUTPUT_SPEED_50Mhz					3

/*
 * Pin number
 */
#define GPIO_PIN_0							0
#define GPIO_PIN_1							1
#define GPIO_PIN_2							2
#define GPIO_PIN_3							3
#define GPIO_PIN_4							4
#define GPIO_PIN_5							5
#define GPIO_PIN_6							6
#define GPIO_PIN_7							7
#define GPIO_PIN_8							8
#define GPIO_PIN_9							9
#define GPIO_PIN_10							10
#define GPIO_PIN_11							11
#define GPIO_PIN_12							12
#define GPIO_PIN_13							13
#define GPIO_PIN_14							14
#define GPIO_PIN_15							15

typedef struct {
	uint8_t pinNumber;
	uint8_t pinMode;
	uint8_t pinSpeed;
	uint8_t pinInType;
	uint8_t pinOutType;
	uint8_t pinAltFunMode;
} GPIO_PinConfig_t;

/* Define Handle structure for GPIO pin */
typedef struct {
	GPIO_RegDef_t *pGPIO;
	GPIO_PinConfig_t pinConfig;

} GPIO_Handle_t;

/* Control Peripheral clock */
void GPIO_PeriClockControl(GPIO_RegDef_t *pPort, uint8_t isEnabled);

/* Initialization and Deinitialization Pin */
void GPIO_Init(GPIO_Handle_t *pPortHandle);
void GPIO_DeInit(GPIO_RegDef_t *pPort);

/* Read and Write data */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pPort, uint8_t pin);
void GPIO_ReadPort(GPIO_RegDef_t *pPort);
void GPIO_WritePin(GPIO_RegDef_t *pPort, uint8_t pin, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pPort, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pPort, uint8_t pin);

/* config interrupt */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPrior, uint8_t isEnabled);
void GPIO_IRQHandling(uint8_t pin);
#endif /* INC_DRV_GPIO_H_ */
