#ifndef INC_DRV_SPI_H_
#define INC_DRV_SPI_H_

/*
 * define SPI Remap
 */
#define SPI_REMAP_NONE				0
#define SPI_REMAP_SPI1				1

#define SPI_MAPR_SPI1REMAP_BIT		0
/*
 * define type of devices in SPI communication
 */
#define SPI_DEVTYPE_SLAVE			0
#define SPI_DEVTYPE_MASTER			1

/*
 * define type of slave select management
 */
#define SPI_SSM_SOFTWARE			1
#define SPI_SSM_HARDWARE			0

/*
 * define type level when idle clk line
 */
#define SPI_POL_HIGH				1
#define SPI_POL_LOW					0

/*
 * define type phase
 */
#define SPI_PHASE_LEADING			0
#define SPI_PHASE_TRAILING			1

/*
 * define baund rate
 */
#define SPI_BAUND_PCLK_DIV2			0
#define SPI_BAUND_PCLK_DIV4			1
#define SPI_BAUND_PCLK_DIV8			2
#define SPI_BAUND_PCLK_DIV16		3
#define SPI_BAUND_PCLK_DIV32		4
#define SPI_BAUND_PCLK_DIV64		5
#define SPI_BAUND_PCLK_DIV128		6
#define SPI_BAUND_PCLK_DIV256		7


/*
 * define transmission mode
 */
#define SPI_TRANSMOD_FULLDUP		0
#define SPI_TRANSMOD_HALFDUP		1
#define SPI_TRANSMOD_RXONLY			3

/*
 * define data frame
 */
#define SPI_DFF_8BIT				0
#define SPI_DFF_16BIT				1


// define macro bits in SPI driver
#define SPI_CR1_CPHA_BIT				0
#define SPI_CR1_CPOL_BIT				1
#define SPI_CR1_MSTR_BIT				2
#define SPI_CR1_BR0_BIT					3
#define SPI_CR1_SPE_BIT					6
#define SPI_CR1_SSI_BIT					8
#define SPI_CR1_SSM_BIT					9
#define SPI_CR1_RXONLY_BIT				10
#define SPI_CR1_DFF_BIT					11
#define SPI_CR1_BIDIMODE_BIT			15

// define macro status flag

#define SPI_FLAG_SET			1
#define SPI_FLAG_RESET			0

#define SPI_SR_TXE_BIT			1
#define SPI_SR_RXNE_BIT			0
#define SPI_SR_BSY_BIT			7

#define SPI_TXE_FLAG			(1 << SPI_SR_TXE_BIT)
#define SPI_RXE_FLAG			(1 << SPI_SR_RXNE_BIT)
#define SPI_BUSY_FLAG			(11 << SPI_SR_BSY_BIT)

typedef struct {
	uint8_t deviceType;
	uint8_t transmitMode;
	uint8_t baundRate;
	uint8_t dataFrameFormat;
	uint8_t clockPolarity;
	uint8_t clockPhase;
	uint8_t slaveSelectType;
	uint8_t remapName;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPI;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/* Control Peripheral clock */
void SPI_PeriClockControl(SPI_RegDef_t *pSPI, uint8_t isEnabled);

/* Initialization and Deinitialization Pin */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

/*
 * Data transmit and receive
 */
void SPI_TransmitData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t len);

/* config interrupt */
void SPI_ConfigIRQ(uint8_t IRQNum, uint8_t isEnabled);
void SPI_SetPriorityIRQ(uint8_t IRQNum, uint32_t priority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
#endif
