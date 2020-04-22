#include "stm32f103xx.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "string.h"

/*
 * ****1. xac dinh chan GPIO cho SPI function
 * PA4 --> NSS
 * PA5 --> SCK
 * PA6 --> MISO
 * PA7 --> MOSI
 * 2. Khoi tao SPI
 * 3. truyen du lieu moi 500ms
 */
void SPI1_InitGPIO() {
	GPIO_Handle_t GPIOForSPI;

	GPIOForSPI.pGPIO = GPIOA;
	GPIOForSPI.pinConfig.pinMode = MODE_ALT_PUPU;

	// config NSS
	GPIOForSPI.pinConfig.pinNumber = GPIO_PIN_4;
	GPIO_Init(&GPIOForSPI);

	// config SCLK
	GPIOForSPI.pinConfig.pinNumber = GPIO_PIN_5;
	GPIO_Init(&GPIOForSPI);

	// config MISO
	GPIOForSPI.pinConfig.pinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOForSPI);

	// config MOSI
	GPIOForSPI.pinConfig.pinNumber = GPIO_PIN_7;
	GPIO_Init(&GPIOForSPI);
}

void SPI1_Init() {
	SPI_Handle_t spi;
	spi.pSPI= SPI1;
	spi.SPIConfig.baundRate = SPI_BAUND_PCLK_DIV2;
	spi.SPIConfig.clockPhase = SPI_PHASE_LEADING;
	spi.SPIConfig.clockPolarity = SPI_POL_HIGH;
	spi.SPIConfig.dataFrameFormat = SPI_DFF_8BIT;
	spi.SPIConfig.deviceType = SPI_DEVTYPE_MASTER;
	spi.SPIConfig.remapName = SPI_REMAP_NONE;
	spi.SPIConfig.slaveSelectType = SPI_SSM_HARDWARE;
	spi.SPIConfig.transmitMode = SPI_TRANSMOD_FULLDUP;
	SPI_Init(&spi);
}
int main() {
	char user_data[] = "hello world";
	SPI1_InitGPIO();
	SPI1_Init();
	SPI_TransmitData(SPI1, (uint8_t*)user_data, strlen(user_data));



	while(1) {

	}
	return 0;
}
