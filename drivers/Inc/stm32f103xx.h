/*
 * stm32f103xx.h
 *
 *  Created on: Apr 14, 2020
 *      Author: vanho
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include "stdio.h"

#define __vo 	volatile
#define TRUE 	1
#define FALSE	0
#define HIGH	1
#define LOW		0

#define FLASH_BASEADDR				0x08000000U			/* Địa chỉ đầu tiên cùa bộ nhớ FLASH - 64KB*/
#define SRAM_BASEADDR				0x20000000U			/* Địa chỉ đầu tiên của bộ nhớ SRAM - 20KB */
#define ROM_BASEADDR				0x1FFFF000U			/* Thuộc vùng nhớ information block */

#define PERIPH_BASEADDR				0x40000000U			/* Địa chỉ bắt đầu của các ngoại vi */
#define AHB_BASEADDR				0x40018000U			/* Địa chỉ của các ngoại vi được nối đến BUS AHB */
#define APB2_BASEADDR				0x40010000U			/* Địa chỉ của các ngoại vi được nối đến BUS APB2 */
#define APB1_BASEADDR				PERIPH_BASEADDR		/* Địa chỉ của các ngoại vi được nối đến BUS APB1 */

/*
 * Định địa chỉ BASE ADDRESS cho các ngoại vi được nối đến bus APB2
 */
#define GPIOA_BASEADDR				(APB2_BASEADDR + 0x0800)
#define GPIOB_BASEADDR				(APB2_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR				(APB2_BASEADDR + 0x1000)
#define GPIOD_BASEADDR				(APB2_BASEADDR + 0x1400)
#define GPIOE_BASEADDR				(APB2_BASEADDR + 0x1800)

#define USART1_BASEADDR				(APB2_BASEADDR + 0x3800)

#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)

#define EXTI_BASEADDR				(APB2_BASEADDR + 0x0400)

/*
 * Định địa chỉ BASE ADDRESS cho các ngoại vi được nối đến bus APB1
 */
#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)

#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1_BASEADDR + 0x5000)

#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00)

/*
 * Định địa chỉ BASE ADDRESS cho các ngoại vi được nối đến AHB
 */
#define RCC_BASEADDR			(AHB_BASEADDR + 0x9000)
/*
 * Định nghĩa cấu trúc thanh ghi của GPIO Driver
 */
typedef struct
{
	__vo uint32_t CR[2];		/* Port configuration register */
	__vo uint32_t IDR;			/* Port input data register */
	__vo uint32_t ODR;			/* Port output data register */
	__vo uint32_t BSRR;			/* Port bit set/reset register */
	__vo uint32_t BRR;			/* Port bit reset register */
	__vo uint32_t LCKR;			/* Port configuration lock register */
}GPIO_RegDef_t;

/*
 * Định nghĩa các macro khai báo truy cập đến GPIO
 */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)


/*
 * Định nghĩa cấu trúc thanh ghi của SPI Driver
 */
typedef struct
{
	__vo uint32_t CR[2];		/* SPI control register */
	__vo uint32_t SR;			/* SPI status register */
	__vo uint32_t DR;			/* SPI status register */
	__vo uint32_t CRCPR;		/* SPI CRC polynomial register */
	__vo uint32_t RXCRCR;		/* SPI RX CRC register */
	__vo uint32_t TXCRCR;		/* SPI TX CRC register */
	__vo uint32_t I2SCFGR;		/* SPI_I2S configuration register */
	__vo uint32_t I2SPR;		/* SPI_I2S prescaler register */
}SPI_RegDef_t;

/*
 * Định nghĩa các macro khai báo truy cập đến SPI
 */
#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * Định nghĩa cấu trúc thanh ghi của I2C Driver
 */
typedef struct
{
	__vo uint32_t CR[2];
	__vo uint32_t OAR[2];
	__vo uint32_t DR;
	__vo uint32_t SR[2];
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
}I2C_RegDef_t;
/*
 * Định nghĩa các macro khai báo truy cập đến I2C
 */
#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)

/*
 * Định nghĩa cấu trúc thanh ghi USART
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR[3];
	__vo uint32_t GTPR;
}USART_RegDef_t;
/*
 * Định nghĩa các macro khai báo truy cập đến USART
 */
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)

/*
 * Định nghĩa cấu trúc thanh ghi RCC Driver
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBSTR;
	__vo uint32_t CFGR2;
}RCC_RegDef_t;

/* Định nghĩa các macro khai báo truy cập đến RCC Driver */
#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Định nghĩa macro enable/disable clock cho ngoại vi GPIO
 */
#define GPIOA_PCLK_EN()		(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))

#define GPIOA_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 6))

/*
 * Định nghĩa macro enable/disable clock cho ngoại vi USART
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))

/*
 * Định nghĩa macro enable/disable clock cho ngoại vi I2C
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))


/*
 * Định nghĩa macro enable/disable clock cho ngoại vi SPI
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

#endif /* INC_STM32F103XX_H_ */
