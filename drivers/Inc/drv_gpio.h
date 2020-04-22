/*
 * DRV_GPIO.h
 *
 *  Created on: Apr 14, 2020
 *      Author: vanho
 */

#ifndef INC_DRV_GPIO_H_
#define INC_DRV_GPIO_H_

#define MASK_BIT2_3					0x0C

#define SPI_MAPR_SPI1REMAP_BIT		0

/*
 * configure mode for GPIO
 */
#define MODE_ANALOG					0
#define MODE_INPUT_FLOAT			1
#define MODE_INPUT_PUPDR			2
#define MODE_OUTPUT_PUPU			4
#define MODE_OUTPUT_OPENDR			5
#define MODE_ALT_PUPU				6
#define MODE_ALT_OPENDR				7
#define MODE_IT_RISE_EDGE			8
#define MODE_IT_FALL_EDGE			9
#define MODE_IT_RISE_FALL_EDGE		10
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

#define INPUT_PULLUP						1
#define INPUT_PULLDOWN						0
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
	uint8_t pinResType;
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
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pPort, uint8_t pinNum);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pPort);
void GPIO_WritePin(GPIO_RegDef_t *pPort, uint8_t pinNum, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pPort, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pPort, uint8_t pinNum);

/* config interrupt */
void GPIO_ConfigIRQ(uint8_t IRQNum, uint8_t isEnabled);
void GPIO_SetPriorityIRQ(uint8_t IRQNum, uint32_t priority);
void GPIO_IRQHandling(uint8_t pinNum);
#endif /* INC_DRV_GPIO_H_ */
